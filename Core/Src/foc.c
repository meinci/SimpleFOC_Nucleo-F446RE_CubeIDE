/* foc.c ─ FOC-Kern mit SVPWM, Park-Transformation, PI-Regler */
#include "foc.h"
#include <string.h>

/* ════════════════════════════════════════════════════════════
   PI-Regler
   ════════════════════════════════════════════════════════════ */
void PI_Init(PI_t *pi, float Kp, float Ki, float limit)
{
    pi->Kp       = Kp;
    pi->Ki       = Ki;
    pi->limit    = limit;
    pi->integral = 0.0f;
}

void PI_Reset(PI_t *pi)
{
    pi->integral = 0.0f;
}

float PI_Update(PI_t *pi, float error, float dt)
{
    float proportional = pi->Kp * error;
    float integral_new = pi->integral + pi->Ki * error * dt;
    float output       = proportional + integral_new;

    /* Anti-Windup: Integral nur wenn Ausgang nicht gesättigt */
    if (output > pi->limit) {
        output = pi->limit;
        if (error < 0.0f) pi->integral = integral_new;
    } else if (output < -pi->limit) {
        output = -pi->limit;
        if (error > 0.0f) pi->integral = integral_new;
    } else {
        pi->integral = integral_new;
    }
    return output;
}

/* ════════════════════════════════════════════════════════════
   Interne Hilfsfunktion: Tastverhältnis → PWM-Register
   ════════════════════════════════════════════════════════════ */
static inline void _set_pwm(FOC_t *foc, uint32_t ch, float duty)
{
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    __HAL_TIM_SET_COMPARE(foc->htim, ch,
        (uint32_t)(duty * (float)(foc->pwm_arr + 1)));
}

/* ════════════════════════════════════════════════════════════
   FOC_SetPhaseVoltage
   inverse Park → inverse Clarke (Sinus-Kommutierung) → PWM
   ════════════════════════════════════════════════════════════ */
void FOC_SetPhaseVoltage(FOC_t *foc, float Uq, float Ud, float angle_el)
{
    float v_lim = foc->voltage_limit;
    if (Uq >  v_lim) Uq =  v_lim;
    if (Uq < -v_lim) Uq = -v_lim;
    if (Ud >  v_lim) Ud =  v_lim;
    if (Ud < -v_lim) Ud = -v_lim;

    /* Inverse Park: dq → αβ */
    float cos_e  = cosf(angle_el);
    float sin_e  = sinf(angle_el);
    float Ualpha = Ud * cos_e - Uq * sin_e;
    float Ubeta  = Ud * sin_e + Uq * cos_e;

    /* Inverse Clarke: αβ → 3 Phasen */
    float Ua =  Ualpha;
    float Ub = -0.5f * Ualpha + FOC_SQRT3_2 * Ubeta;
    float Uc = -0.5f * Ualpha - FOC_SQRT3_2 * Ubeta;

    /* Normieren auf [0, 1] (Mittelpunkt = 0.5) */
    float norm = foc->voltage_supply;
    float Ta   = 0.5f + Ua / norm;
    float Tb   = 0.5f + Ub / norm;
    float Tc   = 0.5f + Uc / norm;

    _set_pwm(foc, foc->ch1, Ta);
    _set_pwm(foc, foc->ch2, Tb);
    _set_pwm(foc, foc->ch3, Tc);
}

/* ════════════════════════════════════════════════════════════
   FOC_VelocityLoop
   ════════════════════════════════════════════════════════════ */
void FOC_VelocityLoop(FOC_t *foc, float velocity_measured, float dt)
{
    float error = foc->target_velocity - velocity_measured;
    foc->Uq = PI_Update(&foc->pi_velocity, error, dt);
    foc->Ud = 0.0f;
}

/* ════════════════════════════════════════════════════════════
   FOC_Init
   ════════════════════════════════════════════════════════════ */
void FOC_Init(FOC_t *foc,
              TIM_HandleTypeDef *htim,
              uint32_t ch1, uint32_t ch2, uint32_t ch3,
              GPIO_TypeDef *en_port, uint16_t en_pin,
              float v_supply, int pole_pairs, uint32_t pwm_arr)
{
    memset(foc, 0, sizeof(*foc));
    foc->htim           = htim;
    foc->ch1            = ch1;
    foc->ch2            = ch2;
    foc->ch3            = ch3;
    foc->en_port        = en_port;
    foc->en_pin         = en_pin;
    foc->voltage_supply = v_supply;
    foc->voltage_limit  = v_supply * 0.45f;
    foc->pole_pairs     = pole_pairs;
    foc->pwm_arr        = pwm_arr;
    foc->direction      = 1.0f;

    PI_Init(&foc->pi_velocity, 0.1f, 1.0f, foc->voltage_limit);

    HAL_TIM_PWM_Start(htim, ch1);
    HAL_TIM_PWM_Start(htim, ch2);
    HAL_TIM_PWM_Start(htim, ch3);

    _set_pwm(foc, ch1, 0.5f);
    _set_pwm(foc, ch2, 0.5f);
    _set_pwm(foc, ch3, 0.5f);
}

void FOC_Enable(FOC_t *foc)
{
    HAL_GPIO_WritePin(foc->en_port, foc->en_pin, GPIO_PIN_SET);
}

void FOC_Disable(FOC_t *foc)
{
    HAL_GPIO_WritePin(foc->en_port, foc->en_pin, GPIO_PIN_RESET);
    _set_pwm(foc, foc->ch1, 0.5f);
    _set_pwm(foc, foc->ch2, 0.5f);
    _set_pwm(foc, foc->ch3, 0.5f);
}

/* ════════════════════════════════════════════════════════════
   FOC_Calibrate
   Motor auf d-Achse (θ_el = 0) ausrichten → Nullwinkel lesen
   FIX: sensor-Parameter hinzugefügt – Kalibrierung vollständig
   ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef FOC_Calibrate(FOC_t *foc, AS5600_t *sensor)
{
    foc->zero_electric_angle = 0.0f;
    foc->calibrated          = 0;

    float v_cal = foc->voltage_supply * 0.20f;

    for (int i = 0; i < 200; i++) {
        FOC_SetPhaseVoltage(foc, 0.0f, v_cal * (float)i / 200.0f, 0.0f);
        HAL_Delay(2);
    }
    HAL_Delay(700);

    FOC_SetPhaseVoltage(foc, 0.0f, 0.0f, 0.0f);

    /* FIX: Nullwinkel direkt hier einlesen und speichern */
    HAL_StatusTypeDef ret = AS5600_Update(sensor);
    if (ret != HAL_OK) return ret;

    foc->zero_electric_angle =
        fmodf((float)foc->pole_pairs * AS5600_GetAngle(sensor), FOC_TWO_PI);
    foc->calibrated = 1;
    return HAL_OK;
}
