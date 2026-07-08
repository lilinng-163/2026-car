# STM32F407ZGTx 空闲引脚 / Idle Pins

## 已占用 / Occupied

| 类别 Category | 引脚 Pins | 说明 Description |
|------|------|------|
| **FSMC** | PD0,PD1,PD4,PD5,PD8~PD15, PE0,PE1,PE7~PE15, PF0~PF5,PF12~PF15, PG0~PG5,PG10,PG12 | LCD + SRAM (16-bit data + address) |
| **SWD** | PA13, PA14 | 调试接口 / Debug interface |
| **USART1** | PA9, PA10 | 串口日志 / Serial log |
| **LED** | PE3, PE4, PG9 | LED0, LED1, LED2 |
| **BEEP** | PG7 | 蜂鸣器 / Buzzer |
| **DHT11** | PD3 | 温湿度传感器 / Temperature & humidity sensor |
| **KEY** | PF9, PF8, PF7, PF6 | KEY0, KEY1, KEY2, KEY3 (上拉输入 / pull-up input) |
| **Touch** | PA5, PB0, PB1, PB2, PF11 | XPT2046 bit-bang SPI |
| **背光 Backlight** | PF10 | LCD 背光 / LCD backlight |
| **TIM1_CH1** | PA8 | 定时器1 PWM输出 / Timer1 PWM output |
| **TIM2_CH1** | PA0 | 定时器2 PWM输出 / Timer2 PWM output |
| **时钟 Clock** | PC14, PC15, PH0, PH1 | LSE + HSE 晶振 / Crystals |

> 注：PA5, PB0, PB1, PB2, PF10, PF11 由用户代码手动初始化，未在 CubeMX 中配置。  
> Note: PA5, PB0, PB1, PB2, PF10, PF11 are initialized by user code, not configured in CubeMX.

---

## 空闲引脚 / Idle Pins

| 引脚 Pin | 常用复用功能 / Common Alternate Functions |
|-----------|------------------------------------------|
| PA1  | TIM2_CH2, TIM5_CH2, USART2_RTS, ADC1 |
| PA2  | TIM2_CH3, TIM5_CH3, USART2_TX, ADC2 |
| PA3  | TIM2_CH4, TIM5_CH4, USART2_RX, ADC3 |
| PA4  | SPI1_NSS, USART2_CK, ADC4, DAC1 |
| PA6  | TIM3_CH1, TIM13_CH1, SPI1_MISO, ADC6 |
| PA7  | TIM3_CH2, TIM14_CH1, SPI1_MOSI, ADC7 |
| PA11 | TIM1_CH4, CAN1_RX, USB_DM |
| PA12 | TIM1_ETR, CAN1_TX, USB_DP |
| PA15 | TIM2_CH1, SPI1_NSS, JTDI (可用需禁JTAG / free after disabling JTAG) |
| PB3  | TIM2_CH2, SPI1_SCK, JTDO (可用需禁JTAG / free after disabling JTAG) |
| PB4  | TIM3_CH1, SPI1_MISO, NJTRST (可用需禁JTAG / free after disabling JTAG) |
| PB5  | TIM3_CH2, SPI1_MOSI, I2C1_SMBA, CAN2_RX |
| PB6  | TIM4_CH1, I2C1_SCL, CAN2_TX |
| PB7  | TIM4_CH2, I2C1_SDA, USART1_RX |
| PB8  | TIM4_CH3, TIM10_CH1, I2C1_SCL, CAN1_RX |
| PB9  | TIM4_CH4, TIM11_CH1, I2C1_SDA, CAN1_TX |
| PB10 | TIM2_CH3, I2C2_SCL, USART3_TX |
| PB11 | TIM2_CH4, I2C2_SDA, USART3_RX |
| PB12 | SPI2_NSS, I2C2_SMBA, USART3_CK |
| PB13 | TIM1_CH1N, SPI2_SCK, USART3_CTS |
| PB14 | TIM1_CH2N, TIM12_CH1, SPI2_MISO |
| PB15 | TIM1_CH3N, TIM12_CH2, SPI2_MOSI |
| PC0  | ADC10 |
| PC1  | ADC11 |
| PC2  | ADC12 |
| PC3  | ADC13 |
| PC4  | ADC14 |
| PC5  | ADC15 |
| PC6  | TIM3_CH1, TIM8_CH1, USART6_TX, I2S2_MCK |
| PC7  | TIM3_CH2, TIM8_CH2, USART6_RX, I2S3_MCK |
| PC8  | TIM3_CH3, TIM8_CH3, USART6_CK |
| PC9  | TIM3_CH4, TIM8_CH4, MCO2 |
| PC10 | SPI3_SCK, USART3_TX |
| PC11 | SPI3_MISO, USART3_RX, SDIO_D3 |
| PC12 | SPI3_MOSI, USART3_CK, SDIO_CK |
| PC13 | RTC_OUT (未用RTC时可用 / free if RTC unused) |
| PD2  | TIM3_ETR, USART5_RX, SDIO_CMD |
| PD6  | SPI3_MOSI, USART2_RX |
| PD7  | SPI3_MISO, USART2_CK |
| PE2  | TIM3_ETR, TRACECLK |
| PE5  | TRACED1 |
| PE6  | TRACED2 |
| PG6  | 普通 GPIO / General GPIO |
| PG8  | SPI6_NSS, USART6_RTS |
| PG11 | SPI6_MOSI |
| PG13 | FSMC_A24, SPI6_SCK, USART6_CTS |
| PG14 | FSMC_A25, SPI6_MISO, USART6_TX |
| PG15 | SPI6_MOSI, USART6_RTS |

---

**已占用 66 个 IO / 66 IOs occupied**（含 FSMC 16bit 数据 + 地址线，CubeMX 60 个 + 用户代码手动配置 6 个）  
**空闲约 48 个 IO / ~48 IOs free**

PB3 / PB4 / PA15 禁用 JTAG 后释放；其余直接可用。  
Free PB3/PB4/PA15 after disabling JTAG; others ready to use.
