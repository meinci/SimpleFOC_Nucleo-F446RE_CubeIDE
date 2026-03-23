/* as5600.h ─ AS5600 Magnetgeber-Treiber für STM32 HAL
 * Anschluss: SDA=PB9, SCL=PB8  (I2C1, 400 kHz, CubeMX konfigurieren)
 * Non-blocking: HAL_I2C_Mem_Read_IT + HAL_I2C_MemRxCpltCallback
 */
#ifndef AS5600_H
#define AS5600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Register ──────────────────────────────────────────────── */
#define AS5600_I2C_ADDR    (0x36 << 1)   /* 7-bit Adresse, links geshiftet */
#define AS5600_REG_STATUS  0x0B
#define AS5600_REG_RAW_ANG 0x0C          /* roh, vor ZPOS/MPOS */
#define AS5600_REG_ANGLE   0x0E          /* skalierter Ausgangswinkel */
#define AS5600_STATUS_MD   (1 << 5)      /* Magnet erkannt */
#define AS5600_STATUS_ML   (1 << 4)      /* zu schwach */
#define AS5600_STATUS_MH   (1 << 3)      /* zu stark */

#define AS5600_RAW_MAX     4096.0f
#define AS5600_TWO_PI      6.28318530718f
#define AS5600_RAW_TO_RAD  (AS5600_TWO_PI / AS5600_RAW_MAX)

/* ── Geräteobjekt ───────────────────────────────────────────── */
typedef struct {
    I2C_HandleTypeDef *hi2c;

    /* Winkelverfolgung */
    float    angle_rad;          /* aktueller Winkel (unwrapped, rad) */
    float    angle_prev;         /* letzter Winkel im [0, 2π) Bereich */
    int32_t  full_rotations;     /* ganze Umdrehungen */

    /* Geschwindigkeit */
    float    velocity_rad_s;     /* rad/s, Tiefpassgefiltert */
    uint32_t ts_prev_ms;         /* letzter Zeitstempel */

    /* Tiefpassfilter */
    float    lpf_tf;             /* Zeitkonstante [s], z.B. 0.05 */

    /* Non-blocking I2C */
    uint8_t           dma_buf[2];        /* DMA/IT Empfangspuffer – muss im RAM bleiben */
    volatile uint8_t  transfer_busy;     /* 1 = I2C-Transfer läuft gerade */
    volatile uint8_t  data_ready;        /* 1 = neue Daten verfügbar (noch nicht verarbeitet) */
} AS5600_t;

/* ── API ────────────────────────────────────────────────────── */
HAL_StatusTypeDef AS5600_Init       (AS5600_t *dev, I2C_HandleTypeDef *hi2c, float lpf_tf);
uint8_t           AS5600_GetStatus  (AS5600_t *dev);

/* Blocking – nur für Init und Kalibrierung verwenden */
HAL_StatusTypeDef AS5600_Update     (AS5600_t *dev);

/* Non-blocking – im Regelzyklus verwenden */
HAL_StatusTypeDef AS5600_StartRead  (AS5600_t *dev);   /* startet IT-Transfer, kehrt sofort zurück */
void              AS5600_ProcessRead(AS5600_t *dev);   /* im HAL_I2C_MemRxCpltCallback aufrufen    */

static inline float AS5600_GetAngle   (const AS5600_t *dev) { return dev->angle_rad; }
static inline float AS5600_GetVelocity(const AS5600_t *dev) { return dev->velocity_rad_s; }

#endif /* AS5600_H */
