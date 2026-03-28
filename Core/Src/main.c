/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Positionsregelung mit FOC + FreeRTOS (CMSIS-RTOS V2)
  *
  * Betriebsmodus: Positionsregelung (Kaskade Position → Geschwindigkeit → Uq)
  *
  * Button (PC13):
  *   Kurz (<800 ms) : nächste Zielposition aus angle_table
  *   Lang (≥800 ms) : zurück zu 0°
  *
  * Zielwinkel ändern: angle_table[] in rad anpassen
  * Regler tunen    : PI_VEL_KP/KI, PI_POS_KP, PI_POS_VEL_LIMIT in #defines
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "foc.h"
#include "as5600.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef struct {
    float    angle;           /* gemessener Winkel [rad]    */
    float    target;          /* Sollwinkel [rad]           */
    float    velocity;        /* gefilterte Geschw. [rad/s] */
    float    Uq;              /* q-Achsen-Spannung [V]      */
    float    pos_error;       /* Positionsfehler [rad]      */
    uint8_t  stalled;
    uint32_t timestamp_ms;
} LogMsg_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define MOTOR_POLE_PAIRS      7
#define MOTOR_VOLTAGE_SUPPLY  12.0f
#define PWM_ARR               4499U
#define CTRL_TS_S             (1.0f / 20000.0f)
#define SENSOR_DIV            10
#define SENSOR_TS_S           (CTRL_TS_S * SENSOR_DIV)   /* 500 µs */

/* Geschwindigkeitsregler */
#define PI_VEL_KP             0.4f
#define PI_VEL_KI             0.3f

/* Positionsregler */
#define PI_POS_KP             1.0f    /* erhöhen → schneller, aber Überschwingen */
#define PI_POS_KI             0.0f    /* meist 0 – Geschwindigkeitsregler übernimmt */
//#define PI_POS_VEL_LIMIT      40.0f   /* max. Ausgang Positionsregler [rad/s]       */
#define PI_POS_VEL_LIMIT   30.0f   // diese Geschwindigkeit wird konstant gehalten
/* Stall-Erkennung */
#define STALL_VEL_THRESHOLD   0.3f    /* rad/s – unter diesem Wert gilt Motor als still */
#define STALL_POS_THRESHOLD   0.1f    /* rad  – ~6°, ab hier gilt Position als "nicht erreicht" */
#define STALL_TIME_MS         800
#define RESTART_TIME_MS       2000

#define FOC_FLAG_TICK         0x0001U
#define LOG_DIV               200U
#define LOG_QUEUE_DEPTH       8U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static FOC_t    foc;
static AS5600_t sensor;
static uint16_t tick_div     = 0;
static float    angle_prev   = 0.0f;
static float    vel_filtered = 0.0f;

static volatile uint32_t btn_press_time = 0;
static volatile uint8_t  btn_pressed    = 0;

static uint32_t stall_timer        = 0;
static uint8_t  stall_timer_active = 0;
static uint8_t  stalled            = 0;

/*
 * Zielpositionen in Grad → Umrechnung: rad = Grad * PI / 180
 * Beispiel: 0°, 90°, 180°, 270°
 * Anpassen nach Bedarf – beliebig viele Einträge möglich.
 */


#define POS_HOLD_MS    100U   /* wie lange jede Position gehalten wird */

static const float angle_table[] = {
    0.0f,               /*   0° */
    FOC_PI,             /* 180° */
	  0.0f,               /*   0° */
    10*FOC_TWO_PI,         /* 360° */
};
static uint8_t angle_idx = 0;

static uint32_t log_div_cnt = 0;

/* CMSIS-RTOS V2 Handles */
static osThreadId_t foc_task_id = NULL;
static const osThreadAttr_t foc_task_attr = {
    .name       = "FOC_Task",
    .priority   = osPriorityRealtime,
    .stack_size = 512 * 4
};

static osThreadId_t log_task_id = NULL;
static const osThreadAttr_t log_task_attr = {
    .name       = "Log_Task",
    .priority   = osPriorityNormal,
    .stack_size = 512 * 4
};

static osMessageQueueId_t log_queue_id = NULL;
static const osMessageQueueAttr_t log_queue_attr = {
    .name = "LogQueue"
};

static osThreadId_t pos_task_id = NULL;
static const osThreadAttr_t pos_task_attr = {
    .name       = "Pos_Task",
    .priority   = osPriorityNormal,
    .stack_size = 256 * 4
};
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void FOC_Task    (void *arg);
static void Log_Task    (void *arg);
static void sensor_tick (void);
static void stall_update(void);
static void commutate   (void);
static void Pos_Task    (void *arg);
/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();

    /* USER CODE BEGIN 2 */

    /* ── LED-Blitz ───────────────────────────────────────────── */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    /* ── AS5600 ──────────────────────────────────────────────── */
    if (AS5600_Init(&sensor, &hi2c1, 0.03f) != HAL_OK) {
        while (1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(100);
        }
    }
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(250);
    }

    /* ── FOC Init ────────────────────────────────────────────── */
    FOC_Init(&foc,
             &htim1,
             TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
             GPIOB, GPIO_PIN_10,
             MOTOR_VOLTAGE_SUPPLY,
             MOTOR_POLE_PAIRS,
             PWM_ARR,
             PI_VEL_KP, PI_VEL_KI,
             PI_POS_KP, PI_POS_KI, PI_POS_VEL_LIMIT);

    /* ── Kalibrierung ────────────────────────────────────────── */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    FOC_Enable(&foc);
    if (FOC_Calibrate(&foc, &sensor) != HAL_OK) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

    /* Startposition = aktuelle Istposition → kein Ruck beim Start */
    foc.target_angle = AS5600_GetAngle(&sensor);
    angle_prev       = foc.target_angle;

    /* ── FreeRTOS ────────────────────────────────────────────── */
    osKernelInitialize();

    pos_task_id = osThreadNew(Pos_Task, NULL, &pos_task_attr);
    if (pos_task_id == NULL) Error_Handler();

    log_queue_id = osMessageQueueNew(LOG_QUEUE_DEPTH, sizeof(LogMsg_t),
                                     &log_queue_attr);
    if (log_queue_id == NULL) Error_Handler();

    foc_task_id = osThreadNew(FOC_Task, NULL, &foc_task_attr);
    if (foc_task_id == NULL) Error_Handler();

    log_task_id = osThreadNew(Log_Task, NULL, &log_task_attr);
    if (log_task_id == NULL) Error_Handler();

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    osKernelStart();

    /* USER CODE END 2 */
    while (1) {}
}

/* ════════════════════════════════════════════════════════════
   FOC_Task  (osPriorityRealtime, 20 kHz)
   ════════════════════════════════════════════════════════════ */
static void FOC_Task(void *arg)
{
    (void)arg;
    for (;;)
    {
        osThreadFlagsWait(FOC_FLAG_TICK, osFlagsWaitAny, osWaitForever);

        tick_div++;
        if (tick_div >= SENSOR_DIV) {
            tick_div = 0;
            sensor_tick();
            stall_update();
        }
        commutate();
    }
}

static void Pos_Task(void *arg)
{
    (void)arg;
    uint8_t idx = 0;
    uint8_t n   = sizeof(angle_table) / sizeof(angle_table[0]);

    for (;;)
    {
        foc.target_angle = angle_table[idx];

        /* Warten bis Position erreicht (Fehler < Deadband) */
        float error;
        do {
            osDelay(10);
            error = foc.target_angle - AS5600_GetAngle(&sensor);
        } while (fabsf(error) > FOC_POS_DEADBAND * 2.0f);

        osDelay(500);   /* kurz halten, dann nächste Position */

        idx = (idx + 1) % n;
    }
}

/* ════════════════════════════════════════════════════════════
   Log_Task  (osPriorityNormal)
   ════════════════════════════════════════════════════════════ */
static void Log_Task(void *arg)
{
    (void)arg;
    LogMsg_t msg;
    char     buf[160];

    for (;;)
    {
        if (osMessageQueueGet(log_queue_id, &msg, NULL, osWaitForever) == osOK)
        {
            int len = snprintf(buf, sizeof(buf),
                               "--- t=%lu ms ---\r\n"
                               "  angle : %.3f rad (%.1f deg)\r\n"
                               "  target: %.3f rad (%.1f deg)\r\n"
                               "  error : %.3f rad\r\n"
                               "  vel   : %.2f rad/s\r\n"
                               "  Uq    : %.2f V\r\n"
                               "  stall : %u\r\n",
                               (unsigned long)msg.timestamp_ms,
                               msg.angle,   msg.angle   * 57.2957f,
                               msg.target,  msg.target  * 57.2957f,
                               msg.pos_error,
                               msg.velocity,
                               msg.Uq,
                               msg.stalled);

            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 200);
        }
    }
}

/* ════════════════════════════════════════════════════════════
   sensor_tick  (2 kHz)
   Positionsregelung: Position → Geschwindigkeit → Uq
   ════════════════════════════════════════════════════════════ */
static void sensor_tick(void)
{
    if (!foc.calibrated) return;

    AS5600_StartRead(&sensor);

    /* Winkel lesen + Geschwindigkeit berechnen (festes dt = SENSOR_TS_S) */
    float mech = AS5600_GetAngle(&sensor);
    float d    = mech - angle_prev;   /* unwrapped → kein Wrap-Clip nötig */
    angle_prev = mech;

    vel_filtered += 0.05f * (d / SENSOR_TS_S - vel_filtered);

    /* Kaskadenregler: Position → Sollgeschwindigkeit → Uq */
    FOC_PositionLoop(&foc, mech, vel_filtered, SENSOR_TS_S);

    /* Logging */
    log_div_cnt++;
    if (log_div_cnt >= LOG_DIV) {
        log_div_cnt = 0;

        float pos_error = foc.target_angle - mech;
        while (pos_error >  FOC_PI) pos_error -= FOC_TWO_PI;
        while (pos_error < -FOC_PI) pos_error += FOC_TWO_PI;

        LogMsg_t msg = {
            .angle        = mech,
            .target       = foc.target_angle,
            .velocity     = vel_filtered,
            .Uq           = foc.Uq,
            .pos_error    = pos_error,
            .stalled      = stalled,
            .timestamp_ms = HAL_GetTick()
        };
        osMessageQueuePut(log_queue_id, &msg, 0, 0);
    }
}

/* ════════════════════════════════════════════════════════════
   stall_update  (2 kHz)
   Stall: Positionsfehler > Schwelle UND Geschwindigkeit ~0
   ════════════════════════════════════════════════════════════ */
static void stall_update(void)
{
    float pos_error = foc.target_angle - AS5600_GetAngle(&sensor);
    while (pos_error >  FOC_PI) pos_error -= FOC_TWO_PI;
    while (pos_error < -FOC_PI) pos_error += FOC_TWO_PI;

    if (!stalled) {
        if (fabsf(vel_filtered)  < STALL_VEL_THRESHOLD &&
            fabsf(pos_error)     > STALL_POS_THRESHOLD) {
            if (!stall_timer_active) {
                stall_timer_active = 1;
                stall_timer = HAL_GetTick();
            } else if (HAL_GetTick() - stall_timer > STALL_TIME_MS) {
                stalled            = 1;
                stall_timer_active = 1;
                stall_timer        = HAL_GetTick();
                FOC_Disable(&foc);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
            }
        } else {
            stall_timer_active = 0;
            stall_timer        = 0;
        }
    } else {
        /* Stall-LED blinkt */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
            (HAL_GetTick() / 200) % 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (HAL_GetTick() - stall_timer > RESTART_TIME_MS) {
            stalled            = 0;
            stall_timer_active = 0;
            stall_timer        = 0;

            /* Positionsregler und Integratoren zurücksetzen */
            PI_Reset(&foc.pi_velocity);
            PI_Reset(&foc.pi_position);

            /* Ziel = aktuelle Istposition → kein Ruck beim Neustart */
            foc.target_angle = AS5600_GetAngle(&sensor);
            angle_prev       = foc.target_angle;

            FOC_Enable(&foc);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
    }
}

/* ════════════════════════════════════════════════════════════
   commutate  (20 kHz)
   ════════════════════════════════════════════════════════════ */
static void commutate(void)
{
    if (!foc.calibrated) return;

    float elec_angle = FOC_ElecAngle(&foc, AS5600_GetAngle(&sensor));
    FOC_SetPhaseVoltage(&foc, foc.Uq, foc.Ud, elec_angle);
}

/* ════════════════════════════════════════════════════════════
   System Clock
   ════════════════════════════════════════════════════════════ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 8;
    RCC_OscInitStruct.PLL.PLLN            = 180;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    if (HAL_PWREx_EnableOverDrive() != HAL_OK)           Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

/* ════════════════════════════════════════════════════════════
   TIM-Callback
   TIM6 → HAL_IncTick()
   TIM1 → FOC_Task wecken (20 kHz)
   ════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }
    if (htim->Instance == TIM1) {
        if (foc_task_id != NULL) {
            osThreadFlagsSet(foc_task_id, FOC_FLAG_TICK);
        }
    }
}

/* ── I2C Callback ─────────────────────────────────────────── */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        AS5600_ProcessRead(&sensor);
    }
}

/* ── Button EXTI ──────────────────────────────────────────── */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        uint32_t now = HAL_GetTick();

        /* Debounce 50 ms */
        if (!btn_pressed && (now - btn_press_time < 50)) return;

        UBaseType_t uxSaved = taskENTER_CRITICAL_FROM_ISR();

        if (!btn_pressed) {
            btn_pressed    = 1;
            btn_press_time = now;
        } else {
            uint32_t duration = now - btn_press_time;
            btn_pressed = 0;

            if (duration < 800) {
                /* Kurzer Druck: nächste Zielposition */
                angle_idx = (angle_idx + 1) %
                            (uint8_t)(sizeof(angle_table) / sizeof(angle_table[0]));
                foc.target_angle = angle_table[angle_idx];
            } else {
                /* Langer Druck: zurück zu 0° */
                angle_idx        = 0;
                foc.target_angle = angle_table[0];
            }

            /* Integratoren nicht zurücksetzen – sanfter Übergang */
        }

        taskEXIT_CRITICAL_FROM_ISR(uxSaved);
    }
}
/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
