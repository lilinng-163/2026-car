# STM32F407 + FreeRTOS + LVGL + CMSIS-DSP

## Hardware

- MCU: STM32F407ZGTx (Cortex-M4, 168MHz, 1024KB Flash, 128KB SRAM)
- LCD: ILI9341 240x320 (FSMC 16-bit 8080 interface, BANK4)
- External SRAM: IS62WV51216 1MB (FSMC BANK3, 0x68000000)
- UART: USART1 PA9/PA10 115200 8N1

## Libraries

| Library | Path | Description |
|---------|------|-------------|
| **STM32F4 HAL** | `Drivers/` | CubeMX generated HAL drivers |
| **FreeRTOS** | `Lib/FreeRTOS/` | RTOS kernel, static library |
| **LVGL** | `Lib/lvgl/lvgl_sdk/` | GUI framework, local copy |
| **CMSIS-DSP** | `Lib/DSP/cmsis_dsp_sdk/` | Digital Signal Processing, local copy |

## Build

```
cmake --preset Debug -S .
cmake --build build/Debug
```

Requires ARM GCC toolchain (`arm-none-eabi-`).

## Memory Layout

```
0x08000000  Flash   1024KB   code, rodata, fonts
0x20000000  SRAM    128KB    bss, heap, stack (~3KB used)
0x10000000  CCM     64KB     unused
0x68000000  Ext SRAM 1MB     LVGL pool (64KB) + display buffers (48KB)
0x6C000000  LCD     FSMC     ILI9341 command/data
```

## Key Configurations

- **HAL Tick**: TIM14 (not SysTick, since FreeRTOS uses SysTick)
- **LVGL Memory**: placed in external SRAM via `LV_MEM_ADR = 0x68000000`
- **LVGL Buffers**: dual buffer (240x50x2 bytes each), partial render mode
- **Display**: RGB565 color format
