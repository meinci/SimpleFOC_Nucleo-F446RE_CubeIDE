/* as5600.c ─ AS5600 Magnetgeber-Implementierung*/

#include "as5600.h"
#include <math.h>
#include <string.h>

/* ════════════════════════════════════════════════════════════
   Interne Hilfsfunktion: Rohdaten verarbeiten
   Gemeinsam von AS5600_Update (blocking) und
   AS5600_ProcessRead (non-blocking) genutzt.
   ════════════════════════════════════════════════════════════ */
static void _process(AS5600_t *dev, uint8_t buf[2])
{
    /* 12-bit Rohwert → Radiant */
    uint16_t raw   = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    float    angle = (float)raw * AS5600_RAW_TO_RAD;   /* [0, 2π) */

    /* Vollständige Umdrehungen verfolgen (Überlauf erkennen) */
    float delta = angle - dev->angle_prev;
    if      (delta >  (float)M_PI)  dev->full_rotations--;
    else if (delta < -(float)M_PI)  dev->full_rotations++;

    dev->angle_prev = angle;
    dev->angle_rad  = angle + (float)dev->full_rotations * AS5600_TWO_PI;

    /* Geschwindigkeit (diskrete Ableitung + optionaler Tiefpass) */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - dev->ts_prev_ms) * 1e-3f;
    if (dt >= 1e-4f) {
        float v_raw = delta / dt;

        if (dev->lpf_tf > 0.0f) {
            float alpha = dt / (dev->lpf_tf + dt);
            dev->velocity_rad_s += alpha * (v_raw - dev->velocity_rad_s);
        } else {
            dev->velocity_rad_s = v_raw;
        }
        dev->ts_prev_ms = now;
    }
}

/* ════════════════════════════════════════════════════════════
   AS5600_Init
   ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef AS5600_Init(AS5600_t *dev, I2C_HandleTypeDef *hi2c, float lpf_tf)
{
    memset(dev, 0, sizeof(*dev));
    dev->hi2c    = hi2c;
    dev->lpf_tf  = (lpf_tf > 0.0f) ? lpf_tf : 0.0f;
    dev->ts_prev_ms = HAL_GetTick();

    /* Magnet prüfen (blocking – einmalig beim Start) */
    uint8_t status = AS5600_GetStatus(dev);
    if (!(status & AS5600_STATUS_MD))
        return HAL_ERROR;   /* kein Magnet – Kabel/Position prüfen */

    /* Erstwert blockierend einlesen (Velocity = 0 beim Start) */
    HAL_StatusTypeDef ret = AS5600_Update(dev);
    dev->velocity_rad_s = 0.0f;
    return ret;
}

/* ════════════════════════════════════════════════════════════
   AS5600_GetStatus
   ════════════════════════════════════════════════════════════ */
uint8_t AS5600_GetStatus(AS5600_t *dev)
{
    uint8_t status = 0;
    HAL_I2C_Mem_Read(dev->hi2c, AS5600_I2C_ADDR,
                     AS5600_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
                     &status, 1, 5);
    return status;
}

/* ════════════════════════════════════════════════════════════
   AS5600_Update  (BLOCKING)
   Nur für Init und FOC_Calibrate verwenden –
   im Regelzyklus stattdessen AS5600_StartRead nutzen.
   ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef AS5600_Update(AS5600_t *dev)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(
        dev->hi2c, AS5600_I2C_ADDR,
        AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT,
        buf, 2, 5);

    if (ret != HAL_OK) return ret;

    _process(dev, buf);
    return HAL_OK;
}

/* ════════════════════════════════════════════════════════════
   AS5600_StartRead  (NON-BLOCKING)
   Startet einen Interrupt-gesteuerten I2C-Transfer.
   Kehrt sofort zurück – blockiert die Regelschleife nicht.
   Ergebnis kommt in AS5600_ProcessRead().
   ════════════════════════════════════════════════════════════ */
HAL_StatusTypeDef AS5600_StartRead(AS5600_t *dev)
{
    if (dev->transfer_busy) return HAL_BUSY;   /* vorheriger Transfer noch aktiv */

    dev->transfer_busy = 1;
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(
        dev->hi2c, AS5600_I2C_ADDR,
        AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT,
        dev->dma_buf, 2);

    if (ret != HAL_OK)
        dev->transfer_busy = 0;   /* Fehler – Flag sofort freigeben */

    return ret;
}

void AS5600_ProcessRead(AS5600_t *dev)
{
    dev->transfer_busy = 0;
    _process(dev, dev->dma_buf);
    dev->data_ready = 1;
}
