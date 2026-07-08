# STM32F407 + FreeRTOS + LVGL + CMSIS-DSP + ETL

## Hardware

- MCU: STM32F407ZGTx (Cortex-M4, 168MHz, 1024KB Flash, 128KB SRAM)
- LCD: ILI9341 240x320 (FSMC 16-bit 8080 interface, BANK4)
- External SRAM: IS62WV51216 1MB (FSMC BANK3, 0x68000000)
- Touch: XPT2046 (SPI)
- UART: USART1 PA9/PA10 115200 8N1
- Keys: KEY0~KEY3 (PF9/PF8/PF7/PF6, pull-up input)
- PWM: TIM1_CH1 (PA8, motor), TIM2_CH1 (PA0, 50Hz servo)

## Libraries

| Library | Path | Description |
|---------|------|-------------|
| **STM32F4 HAL** | `Drivers/` | CubeMX generated HAL drivers |
| **FreeRTOS** | `Lib/FreeRTOS/` | RTOS kernel, static library |
| **LVGL** | `Lib/lvgl/lvgl_sdk/` | GUI framework (LV_OS_NONE, bare-metal mode) |
| **CMSIS-DSP** | `Lib/DSP/cmsis_dsp_sdk/` | Digital Signal Processing, static library |
| **ETL** | `Lib/ETL/etl_sdk/` | Embedded Template Library, header-only |

## Build

```
cmake --preset Debug -S .
cmake --build build/Debug
```

Requires ARM GCC toolchain (`arm-none-eabi-`).

## Architecture

```
main.cpp ──> lv_init/lcd_init/touch_init → tasks → vTaskStartScheduler
                │
                ├── lvgl_task (prio=1): lv_timer_handler() every 5ms
                ├── show_pv_task (prio=2): create UI, page switch, periodic PID/DHT11 refresh
                ├── led_task (prio=3): LED blink every 500ms
                ├── key_task (prio=3): KEY0~3 scan; page nav + tune left PID Kp (+0.2)
                ├── servo_task (prio=3): sweep servo 0~180° via TIM2_CH1
                ├── motor_task (prio=?): speed PID loop every 10ms (未验证)

app/
├── inc/
│   ├── lvgl_task.h
│   ├── led_task.h
│   ├── key_task.h
│   ├── servo_task.h
│   ├── lv_obj.h
│   ├── motor_task.h   # extern left/right_motor_pid instances
│   ├── mutex.h         # FreeRTOS mutex (lvgl, motor)
│   └── show_pv.h
└── src/
    ├── lvgl_task.cpp
    ├── led_task.cpp
    ├── key_task.cpp    # button state machine → page nav + PID tuning
    ├── servo_task.cpp
    ├── lv_obj.cpp
    ├── motor_task.cpp  # defines left/right_motor_pid
    ├── mutex.cpp
    └── show_pv.cpp

middleware/
└── pid/
    ├── inc/pid.h       # CMSIS-DSP PID wrapper, get_instance() returns ref (未验证)
    └── src/pid.cpp

Device/input/
├── inc/
│   ├── encoder.h       # encoder base + motor_encoder (TIM) + rotary_encoder (GPIO IRQ) (未验证)
│   ├── key.h           # key: debounce/click/long-press/repeat/double-click FSM
│   └── xpt2046.h
└── src/
    ├── encoder.cpp
    ├── key.cpp
    └── xpt2046.cpp

Device/output/
├── inc/
│   ├── motor.h         # PWM motor driver (未验证)
│   ├── servo.h         # PWM servo driver (angle → pulse)
│   ├── beep.h
│   ├── dht11.h
│   ├── lcd.h
│   └── led.h
└── src/
    ├── motor.cpp
    ├── servo.cpp
    └── ...
```

- HAL tick: TIM14 (FreeRTOS uses SysTick)
- LVGL mode: LV_OS_NONE (single-thread, mutex for cross-task access)
- PID: velocity form with conditional integration anti-windup
- Encoder: TIM encoder mode (4x), hardware counting zero CPU overhead
- UI ↔ control: shared PID objects, key_task writes / show_pv reads (float 4-byte access atomic on M4)

## Memory Layout

```
0x08000000  Flash   1024KB   code, rodata, fonts
0x20000000  SRAM    128KB    bss, heap, stack
0x10000000  CCM     64KB     unused
0x68000000  Ext SRAM 1MB     LVGL pool (64KB) + display buffers
0x6C000000  LCD     FSMC     ILI9341 command/data
```

## Key Configurations

- **FreeRTOSConfig.h**: `configUSE_MUTEXES=1`, `configSUPPORT_DYNAMIC_ALLOCATION=1`
- **LVGL Memory**: external SRAM `LV_MEM_ADR = 0x68000000`
- **LVGL Buffers**: dual buffer (240x50x2 bytes), partial render, RGB565
- **stm32f4xx_it.c**: `SysTick_Handler` calls `xPortSysTickHandler()`
