/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body – mit FreeRTOS (CMSIS-RTOS V2)
  *
  * BUGFIXES gegenüber vorheriger Version:
  *   FIX 1: HAL_TIM_PeriodElapsedCallback – TIM6-Zweig mit HAL_IncTick()
  *           ergänzt. Ohne diesen Aufruf gibt HAL_GetTick() immer 0 zurück
  *           → HAL_Delay() hängt ewig → LED bleibt dauerhaft AN.
  *   FIX 2: freertos.c Task-Stubs (StartFOC_Task / StartLog_Task) bleiben
  *           leer – MX_FREERTOS_Init() wird NICHT aufgerufen, da Tasks
  *           vollständig in main.c verwaltet werden.
  *   FIX 3: configUSE_NEWLIB_REENTRANT in FreeRTOSConfig.h auf 0 setzen
  *           wenn -specs=nano.specs verwendet wird.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
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
    float    velocity;
    float    target;
    float    Uq;
    uint8_t  stalled;
    uint32_t timestamp_ms;
} LogMsg_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define MOTOR_POLE_PAIRS     7
#define MOTOR_VOLTAGE_SUPPLY 12.0f
#define PWM_ARR              4499U
#define CTRL_TS_S            (1.0f / 20000.0f)
#define SENSOR_DIV           10
#define SENSOR_TS_S          (CTRL_TS_S * SENSOR_DIV)

#define RAMP_RATE            5.0f
#define STALL_VEL_THRESHOLD  2.0f
#define STALL_TIME_MS        500
#define RESTART_TIME_MS      2000

#define FOC_FLAG_TICK        0x0001U
#define LOG_DIV              200U
#define LOG_QUEUE_DEPTH      8U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static FOC_t    foc;
static AS5600_t sensor;
static uint16_t tick_div     = 0;
static float    angle_prev   = 0.0f;
static float    vel_filtered = 0.0f;
static float    vel_ramp     = 0.0f;

static volatile uint32_t btn_press_time = 0;
static volatile uint8_t  btn_pressed    = 0;

static uint32_t stall_timer        = 0;
static uint8_t  stall_timer_active = 0;
static uint8_t  stalled            = 0;

static const float speed_table[] = { 5.24f, 10.47f, 20.94f, 31.42f };
static uint8_t     speed_idx     = 2;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void FOC_Task    (void *arg);
static void Log_Task    (void *arg);
static void sensor_tick (void);
static void stall_update(void);
static void commutate   (void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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

  /* ── FOC Init + Kalibrierung ─────────────────────────────── */
  FOC_Init(&foc,
           &htim1,
           TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           GPIOB, GPIO_PIN_10,
           MOTOR_VOLTAGE_SUPPLY,
           MOTOR_POLE_PAIRS,
           PWM_ARR);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  FOC_Enable(&foc);
  if (FOC_Calibrate(&foc, &sensor) != HAL_OK) {
      Error_Handler();
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  vel_ramp            = 0.0f;
  foc.target_velocity = 0.0f;

  /* ── FreeRTOS Kernel + Objekte erstellen ─────────────────── */
  osKernelInitialize();

  log_queue_id = osMessageQueueNew(LOG_QUEUE_DEPTH, sizeof(LogMsg_t),
                                   &log_queue_attr);
  if (log_queue_id == NULL) Error_Handler();

  foc_task_id = osThreadNew(FOC_Task, NULL, &foc_task_attr);
  if (foc_task_id == NULL) Error_Handler();

  log_task_id = osThreadNew(Log_Task, NULL, &log_task_attr);
  if (log_task_id == NULL) Error_Handler();

  /* HINWEIS: MX_FREERTOS_Init() wird hier NICHT aufgerufen!
   * Die Tasks in freertos.c (StartFOC_Task, StartLog_Task) sind
   * leere Stubs von CubeMX – sie laufen nicht. Alle echten Tasks
   * werden oben mit osThreadNew() gestartet. */

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

/* ════════════════════════════════════════════════════════════
   Log_Task  (osPriorityNormal)
   ════════════════════════════════════════════════════════════ */
static void Log_Task(void *arg)
{
    (void)arg;
    LogMsg_t msg;
    char     buf[120];  /* vergrößert für mehrzeiliges Format */

    for (;;)
    {
        if (osMessageQueueGet(log_queue_id, &msg, NULL, osWaitForever) == osOK)
        {
            int len = snprintf(buf, sizeof(buf),
                               "--- t=%lu ms ---\r\n"
                               "  vel  : %.2f rad/s\r\n"
                               "  tgt  : %.2f rad/s\r\n"
                               "  Uq   : %.2f V\r\n"
                               "  stall: %u\r\n",
                               (unsigned long)msg.timestamp_ms,
                               msg.velocity,
                               msg.target,
                               msg.Uq,
                               msg.stalled);

            HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);
        }
    }
}

/* ════════════════════════════════════════════════════════════
   sensor_tick  (2 kHz)
   ════════════════════════════════════════════════════════════ */
static void sensor_tick(void)
{
    AS5600_StartRead(&sensor);

    float mech = AS5600_GetAngle(&sensor);
    float d    = mech - angle_prev;
    if (d >  FOC_PI) d -= FOC_TWO_PI;
    if (d < -FOC_PI) d += FOC_TWO_PI;
    angle_prev = mech;

    vel_filtered += 0.1f * (d / SENSOR_TS_S - vel_filtered);

    float target_abs = speed_table[speed_idx];
    float sign       = (foc.target_velocity < 0.0f) ? -1.0f : 1.0f;
    vel_ramp += RAMP_RATE * SENSOR_TS_S;
    if (vel_ramp > target_abs) vel_ramp = target_abs;
    foc.target_velocity = sign * vel_ramp;

    FOC_VelocityLoop(&foc, vel_filtered, SENSOR_TS_S);

    log_div_cnt++;
    if (log_div_cnt >= LOG_DIV) {
        log_div_cnt = 0;
        LogMsg_t msg = {
            .velocity     = vel_filtered,
            .target       = foc.target_velocity,
            .Uq           = foc.Uq,
            .stalled      = stalled,
            .timestamp_ms = HAL_GetTick()
        };
        osMessageQueuePut(log_queue_id, &msg, 0, 0);
    }
}

/* ════════════════════════════════════════════════════════════
   stall_update  (2 kHz)
   ════════════════════════════════════════════════════════════ */
static void stall_update(void)
{
    if (!stalled) {
        if (fabsf(vel_filtered) < STALL_VEL_THRESHOLD &&
            fabsf(foc.target_velocity) > STALL_VEL_THRESHOLD) {
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
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,
            (HAL_GetTick() / 200) % 2 ? GPIO_PIN_SET : GPIO_PIN_RESET);

        if (HAL_GetTick() - stall_timer > RESTART_TIME_MS) {
            stalled            = 0;
            stall_timer_active = 0;
            stall_timer        = 0;
            vel_ramp           = 0.0f;
            PI_Reset(&foc.pi_velocity);
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
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */

/* ══════════════════════════════════════════════════════════════
   HAL_TIM_PeriodElapsedCallback
   !! KRITISCH: Beide Timer müssen hier behandelt werden !!
   TIM6 → HAL_IncTick()  (HAL-Timebase, sonst hängt HAL_Delay)
   TIM1 → FOC_Task wecken (20 kHz Regelinterrupt)
   ══════════════════════════════════════════════════════════════ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ── FIX 1: TIM6 = HAL Timebase → Tick inkrementieren ─── */
    if (htim->Instance == TIM6) {
        HAL_IncTick();
    }

    /* ── TIM1 = FOC 20 kHz → Task benachrichtigen ─────────── */
    if (htim->Instance == TIM1) {
        if (foc_task_id != NULL) {
            osThreadFlagsSet(foc_task_id, FOC_FLAG_TICK);
        }
    }
}

/* ── I2C Callback ────────────────────────────────────────── */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        AS5600_ProcessRead(&sensor);
    }
}

/* ── Button EXTI ─────────────────────────────────────────── */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13)
    {
        UBaseType_t uxSaved = taskENTER_CRITICAL_FROM_ISR();

        if (!btn_pressed) {
            btn_pressed    = 1;
            btn_press_time = HAL_GetTick();
        } else {
            uint32_t duration = HAL_GetTick() - btn_press_time;
            btn_pressed = 0;

            if (duration < 800) {
                speed_idx = (speed_idx + 1) % 4;
                vel_ramp  = fabsf(vel_filtered);
                foc.target_velocity = speed_table[speed_idx]
                                    * (foc.target_velocity < 0.0f ? -1.0f : 1.0f);
            } else {
                foc.target_velocity = -foc.target_velocity;
                vel_ramp = fabsf(vel_filtered);
            }
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
