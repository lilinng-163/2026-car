/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_3
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOE
#define KEY3_Pin GPIO_PIN_6
#define KEY3_GPIO_Port GPIOF
#define KEY2_Pin GPIO_PIN_7
#define KEY2_GPIO_Port GPIOF
#define KEY1_Pin GPIO_PIN_8
#define KEY1_GPIO_Port GPIOF
#define KEY0_Pin GPIO_PIN_9
#define KEY0_GPIO_Port GPIOF
#define LEFT_MOTOR_PWM_Pin GPIO_PIN_1
#define LEFT_MOTOR_PWM_GPIO_Port GPIOA
#define LEFT_MOTOR_EA_Pin GPIO_PIN_6
#define LEFT_MOTOR_EA_GPIO_Port GPIOA
#define LEFT_MOTOR_B_Pin GPIO_PIN_7
#define LEFT_MOTOR_B_GPIO_Port GPIOA
#define LEFT_MOTOR_IN1_Pin GPIO_PIN_0
#define LEFT_MOTOR_IN1_GPIO_Port GPIOB
#define LEFT_MOTOR_IN2_Pin GPIO_PIN_1
#define LEFT_MOTOR_IN2_GPIO_Port GPIOB
#define RIGHT_MOTOR_PWM_Pin GPIO_PIN_10
#define RIGHT_MOTOR_PWM_GPIO_Port GPIOB
#define BEEP_Pin GPIO_PIN_7
#define BEEP_GPIO_Port GPIOG
#define MPU6050_SCL_Pin GPIO_PIN_6
#define MPU6050_SCL_GPIO_Port GPIOC
#define MPU6050_SDA_Pin GPIO_PIN_7
#define MPU6050_SDA_GPIO_Port GPIOC
#define RIGHT_MOTOR_IN1_Pin GPIO_PIN_8
#define RIGHT_MOTOR_IN1_GPIO_Port GPIOC
#define RIGHT_MOTOR_IN2_Pin GPIO_PIN_9
#define RIGHT_MOTOR_IN2_GPIO_Port GPIOC
#define DHT11_Pin GPIO_PIN_3
#define DHT11_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOG
#define OLED096_SCL_Pin GPIO_PIN_3
#define OLED096_SCL_GPIO_Port GPIOB
#define OLED096_SDA_Pin GPIO_PIN_4
#define OLED096_SDA_GPIO_Port GPIOB
#define RIGHT_MOTOR_EA_Pin GPIO_PIN_6
#define RIGHT_MOTOR_EA_GPIO_Port GPIOB
#define RIGHT_MOTOR_EB_Pin GPIO_PIN_7
#define RIGHT_MOTOR_EB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
