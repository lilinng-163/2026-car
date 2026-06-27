# STM32F407 + FreeRTOS + LVGL + CMSIS-DSP + ETL

## Hardware

- MCU: STM32F407ZGTx (Cortex-M4, 168MHz, 1024KB Flash, 128KB SRAM)
- LCD: ILI9341 240x320 (FSMC 16-bit 8080 interface, BANK4)
- External SRAM: IS62WV51216 1MB (FSMC BANK3, 0x68000000)
- Touch: XPT2046 (SPI)
- UART: USART1 PA9/PA10 115200 8N1

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
main.c ──> lv_init/lcd_init/touch_init → tasks → vTaskStartScheduler
               │
               ├── lvgl_task (prio=1): lv_timer_handler() every 5ms
               ├── show_pv_task (prio=2): create UI widgets
               └── led_task (prio=3): LED blink every 500ms

app/
├── inc/
│   ├── lvgl_task.h      # LVGL rendering task
│   ├── led_task.h        # LED blink task
│   ├── lvgl_mutex.h      # LVGL mutual exclusion lock
│   └── show_pv.h         # Demo UI task (switches with callbacks)
└── src/
    ├── lvgl_task.cpp
    ├── led_task.cpp
    ├── lvgl_mutex.cpp
    └── show_pv.cpp
```

- HAL tick: TIM14 (FreeRTOS uses SysTick)
- LVGL mode: LV_OS_NONE (single-thread, mutex for cross-task access)
- All LVGL operations protected by `lvgl_mutex` (FreeRTOS mutex)

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
