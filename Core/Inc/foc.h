/* foc.h ─ Field Oriented Control für STM32 + SimpleFOC Mini (L6234) */
#ifndef FOC_H
#define FOC_H

#include "stm32f4xx_hal.h"
#include "as5600.h"   /* FIX: benötigt für FOC_Calibrate-Signatur */
#include <stdint.h>
#include <math.h>

#define FOC_SQRT3    1.7320508075688772f
#define FOC_SQRT3_2  0.8660254037844386f
#define FOC_TWO_PI   6.28318530718f
#define FOC_PI       3.14159265359f

/* ── PI-Regler ──────────────────────────────────────────────── */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float limit;
} PI_t;

void  PI_Init  (PI_t *pi, float Kp, float Ki, float limit);
float PI_Update(PI_t *pi, float error, float dt);
void  PI_Reset (PI_t *pi);

/* ── FOC-Objekt ─────────────────────────────────────────────── */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t      ch1, ch2, ch3;
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;

    float    voltage_supply;
    float    voltage_limit;
    int      pole_pairs;
    uint32_t pwm_arr;

    float    zero_electric_angle;
    uint8_t  calibrated;

    PI_t     pi_velocity;
    float    target_velocity;
    float    Ud, Uq;

    /* Drehrichtung: +1 oder -1, wird beim ersten Anlauf automatisch gesetzt */
    float    direction;
} FOC_t;

/* ── API ────────────────────────────────────────────────────── */
void FOC_Init (FOC_t *foc,
               TIM_HandleTypeDef *htim,
               uint32_t ch1, uint32_t ch2, uint32_t ch3,
               GPIO_TypeDef *en_port, uint16_t en_pin,
               float v_supply, int pole_pairs, uint32_t pwm_arr);

void FOC_Enable  (FOC_t *foc);
void FOC_Disable (FOC_t *foc);

void FOC_SetPhaseVoltage(FOC_t *foc, float Uq, float Ud, float angle_el);
void FOC_VelocityLoop  (FOC_t *foc, float velocity_measured, float dt);

/* FIX: sensor-Parameter hinzugefügt – Kalibrierung ist jetzt vollständig */
HAL_StatusTypeDef FOC_Calibrate(FOC_t *foc, AS5600_t *sensor);

/* Elektrischer Winkel – Drehrichtung über foc->direction */
static inline float FOC_ElecAngle(const FOC_t *foc, float mech_angle)
{
    float ea = foc->direction *
               ((float)foc->pole_pairs * mech_angle - foc->zero_electric_angle);
    ea = fmodf(ea, FOC_TWO_PI);
    if (ea < 0.0f) ea += FOC_TWO_PI;
    return ea;
}

#endif /* FOC_H */
