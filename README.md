# SimpleFOC – STM32 Nucleo-F446RE

Field Oriented Control (FOC) für einen 3-Phasen BLDC-Motor auf Basis des STM32F446RE Mikrocontrollers und der SimpleFOC Mini Treiberplatine.

## Hardware

| Komponente | Details |
|---|---|
| Mikrocontroller | STM32F446RE (Nucleo-64) |
| Motortreiber | SimpleFOC Mini |
| Magnetsensor | AS5600 (I2C, 12-bit) |
| Versorgung | 12 V |
| Polpaare | 7 |

## Features

- **FOC-Regelung** mit inverser Park- und Clarke-Transformation (Sinus-Kommutierung)
- **PI-Geschwindigkeitsregler** mit Anti-Windup
- **AS5600-Treiber** mit Vollrotationsverfolgung, Tiefpassfilter und non-blocking I2C (Interrupt-gesteuert)
- **Sanfter Anlauf** über konfigurierbare Beschleunigungsrampe
- **Stillstandserkennung** mit automatischem Neustart nach Blockierung
- **Tastenbedienung** über Nucleo User Button:
  - Kurzer Druck → Drehzahl wechseln (50 / 100 / 200 / 300 rpm)
  - Langer Druck → Drehrichtung umkehren
- **20 kHz PWM** (TIM1, Center-Aligned), **2 kHz Sensortakt**
- LED-Statusanzeige für Boot, Kalibrierung, Stall und Neustart

## Projektstruktur

```
Core/
├── Inc/
│   ├── foc.h          – FOC-Objekt, PI-Regler, API
│   └── as5600.h       – Sensortreiber, non-blocking API
├── Src/
│   ├── main.c         – Hauptprogramm, ISR-Callbacks
│   ├── foc.c          – FOC-Kern, Kalibrierung, PWM-Ausgabe
│   └── as5600.c       – I2C-Treiber, Winkel- & Geschwindigkeitsberechnung
```

## Regelstruktur

```
TIM1-ISR (20 kHz)
├── commutate()        – elektrischer Winkel → PWM
└── alle 10 Ticks:
    ├── sensor_tick()  – Sensor lesen, Geschwindigkeit filtern, PI-Regler
    └── stall_update() – Stillstandserkennung & Neustart
```

## Konfiguration

Wichtige Parameter in `main.c`:

```c
#define MOTOR_POLE_PAIRS     7        // Polpaare des Motors
#define MOTOR_VOLTAGE_SUPPLY 12.0f    // Versorgungsspannung [V]
#define TARGET_VEL_RAD_S     20.94f   // Solldrehzahl (≈ 200 rpm)
#define RAMP_RATE            5.0f     // Anlauframpe [rad/s²]
#define STALL_VEL_THRESHOLD  2.0f     // Stillstandsschwelle [rad/s]
```

PI-Regler Parameter in `foc.c`:

```c
PI_Init(&foc->pi_velocity, 0.1f, 1.0f, foc->voltage_limit);
//                         Kp    Ki
```

## Entwicklungsumgebung

- STM32CubeIDE
- STM32CubeMX 6.17.0
- HAL-Bibliothek (STM32F4xx HAL Driver)

## Lizenz

MIT
