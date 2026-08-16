# STM32F407 + FreeRTOS + LVGL + CMSIS-DSP + ETL

## Hardware

- MCU: STM32F407ZGTx (Cortex-M4, 168MHz, 1024KB Flash, 128KB SRAM)
- LCD: ILI9341 240x320 (FSMC 16-bit 8080 interface, BANK4)
- External SRAM: IS62WV51216 1MB (FSMC BANK3, 0x68000000)
- Touch: XPT2046 (SPI)
- UART: USART1 PA9/PA10 115200 8N1 (printf), USART2 PA2/PA3 115200 8N1 (IMU)
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
                ├── stepper_task (prio=3): sweep stepper forward/backward via TIM8_CH1
                ├── motor_task (prio=?): speed PID loop every 10ms (未验证)
                ├── imu_task (prio=3): UART RX interrupt → queue → feed_byte → parse_frame → semaphore notify
                └── oled_task (prio=?): OLED display

app/
├── CMakeLists.txt
├── common/                 # 公共头文件
│   └── inc/
│       └── debug_print.h   # 各任务调试打印总开关
├── sensors/                # 传感器数据采集任务
│   ├── inc/
│   │   ├── imu_task.h
│   │   └── tracking_task.h
│   └── src/
│       ├── imu_task.cpp
│       └── tracking_task.cpp
├── control/                # 控制算法任务
│   ├── inc/
│   │   ├── pid_task.h
│   │   ├── stepper_task.h
│   │   └── vector_pid_task.h
│   └── src/
│       ├── pid_task.cpp
│       ├── stepper_task.cpp
│       └── vector_pid_task.cpp
├── ui/                     # 显示/交互 UI
│   ├── inc/
│   │   ├── oled_task.h
│   │   └── tune_ui.h
│   └── src/
│       └── oled_task.cpp
├── comm/                   # 通信任务
│   ├── inc/
│   │   └── uart_cmd_task.h
│   └── src/
│       └── uart_cmd_task.cpp
└── system/                 # 系统任务 (按键/指示灯)
    ├── inc/
    │   ├── key_task.h
    │   └── led_task.h
    └── src/
        ├── key_task.cpp
        └── led_task.cpp

Core/Src/
└── uart_isr.cpp            # UART 中断回调 (ISR 层)

Middleware/
└── pid/
    ├── inc/pid.h       # CMSIS-DSP PID wrapper, get_instance() returns ref (未验证)
    └── src/pid.cpp

SoftDrivers/
├── inc/soft_i2c.h      # 软件 I2C (GPIO 模拟)
└── src/soft_i2c.cpp

Device/input/
├── inc/
│   ├── encoder.h       # encoder base + motor_encoder (TIM) + rotary_encoder (GPIO IRQ) (未验证)
│   ├── imu.h           # IMU 10-axis sensor, UART protocol frame parser
│   ├── key.h           # key: debounce/click/long-press/repeat/double-click FSM
│   └── xpt2046.h
└── src/
    ├── encoder.cpp
    ├── imu.cpp         # 命令发送(checksum计算) + feed_byte帧对齐 + parse_frame(0x04 raw/0x16 quat/0x26 euler)
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

## IMU (10-axis UART sensor)

- UART2 PA2/PA3, 115200 8N1
- 协议帧格式: `0x7E 0x23 [长度] [功能字] [数据...] [校验和]`
- 校验和: 从包头累加到校验位前，取最低字节
- 数据是小端字节序
- 自动上报频率默认 25Hz，可通过 `set_freq(10~100)` 调整
- `printf` 浮点打印需 linker flag `-u _printf_float`（已在 cmake toolchain 中配置）

### 协议功能字

| 功能字 | 帧长 | 方向 | 内容 |
|--------|------|------|------|
| **0x04** | 23B | IMU→MCU | 原始数据: accel 3×int16(16/32767g) + gyro 3×int16(2000/32767°/s) + mag 3×int16(800/32767mG) |
| **0x16** | 21B | IMU→MCU | 四元数: w/x/y/z 4×float32 |
| **0x26** | 17B | IMU→MCU | 欧拉角: roll/pitch/yaw 3×float32 (弧度) |
| **0x80** | 7B | MCU→IMU | 请求固件版本号 |
| **0x01** | 8B | IMU→MCU | 返回版本号 |
| **0x70** | 7B | MCU→IMU | 校准陀螺仪+加速度计 |
| **0x71** | 7B | MCU→IMU | 校准磁力计 |
| **0x73** | 8B | MCU→IMU | 校准温度 |
| **0x60** | 7B | MCU→IMU | 设置输出频率 |
| **0x61** | 7B | MCU→IMU | 设置算法类型(6/9轴) |

### 验证状态

| 功能 | 状态 |
|------|------|
| 接收 raw (0x04) | ✅ 已验证 |
| 接收 quaternion (0x16) | ✅ 已验证 |
| 接收 euler (0x26) | ✅ 已验证 |
| get_version (0x80) | ❌ 未验证 |
| calibration_6 (0x70) | ❌ 未验证 |
| calibration_3 (0x71) | ❌ 未验证 |
| set_freq (0x60) | ❌ 未验证 |
| set_calculation (0x61) | ❌ 未验证 |
