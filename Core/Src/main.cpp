/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "lcd.h"
#include "lvgl.h"
#include "lv_port_touch.h"
#include "lvgl_task.h"
#include "led_task.h"
#include "mutex.h"
#include "servo_task.h"
#include "show_pv.h"
#include "key_task.h"
#include "oled_task.h"
#include "vector_pid_task.h"
#include "imu_task.h"
#include "tracking_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include "FreeRTOS/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* 多任务 printf 互斥：防止并发调 HAL_UART_Transmit 时返回 BUSY 丢字符 */
static SemaphoreHandle_t print_mutex = NULL;

int _write(int file, char *ptr, int len) {
    (void)file;
    bool lock = (print_mutex != NULL) &&
                (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
    if (lock) xSemaphoreTake(print_mutex, portMAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    if (lock) xSemaphoreGive(print_mutex);
    return len;
}

/* FreeRTOS 栈溢出钩子：打印溢出任务名并停住，便于定位 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    printf("!!! STACK OVERFLOW in task: %s\r\n", pcTaskName);
    __disable_irq();
    for (;;) {}
}
#ifdef __cplusplus
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // 行缓冲: 每行凑齐后由 _write 一次性发出，配合 print_mutex 保证各任务整行原子输出
  static char stdout_buf[256];
  setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));
  print_mutex = xSemaphoreCreateMutex();

    printf(
      " __        __   _  ____  \n"
      " \\ \\      / /__| |/ ___|___  _ __ ___   ___\n"
      "  \\ \\ /\\ / / _ \\ | |   / _ \\| '_ ` _ \\ / _ \\\n"
      "   \\ V  V /  __/ | |__| (_) | | | | | |  __/\n"
      "    \\_/\\_/ \\___|_|\\____\\___/|_| |_| |_|\\___|\n");
    printf(
      "____   ___ ____   __   \n"
      "|___ \\ / _ \\___ \\ / /_  \n"
      "  __) | | | |__) | '_ \\ \n"
      " / __/| |_| / __/| (_) |\n"
      "|_____|\\___/_____|\\___/ \n"
      );
  printf(
      "_____ _           _                   _   \n"
      "| ____| | ___  ___| |_ _ __ ___  _ __ (_) ___ \n"
      "|  _| | |/ _ \\/ __| __| '__/ _ \\| '_ \\| |/ __|\n"
      "| |___| |  __/ (__| |_| | | (_) | | | | | (__ \n"
      "|_____|_|\\___|\\___|\\__|_|  \\___/|_| |_|_|\\___|\n"
      "\n"
      "  ____                           _   _ _   _   \n"
      " / ___|___  _ __ ___  _ __   ___| |_(_) |_(_) ___  _ __ \n"
      "| |   / _ \\| '_ ` _ \\| '_ \\ / _ \\ __| | __| |/ _ \\| '_ \\ \n"
      "| |__| (_) | | | | | | |_) |  __/ |_| | |_| | (_) | | | |\n"
      " \\____\\___/|_| |_| |_| .__/ \\___|\\__|_|\\__|_|\\___/|_| |_|\n"
      "                     |_|   \n"
      );
  printf("author: lilinng\r\n");
  printf("email: wangyixiang051129@163.com || yi9597402@gmail.com\r\n");
  printf("__cplusplus: %ld\r\n", static_cast<long>(__cplusplus));
  printf("GCC VERSION: %d.%d.%d\r\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
  // lv_init();
  // lcd_init();
  // lv_port_touch_init();

  // lvgl_mutex_init();

  // lvgl_task_create();
  // show_pv_create();
  led_task_create();
  // servo_task_create();
  key_task_create();
  oled_task_create();
  vector_pid_task_create();
  // imu_task_create();
  tracking_task_create();

  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
