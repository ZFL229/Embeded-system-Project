/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "proto.h"
#include "flash_crc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_ADDRESS  (0x08004000U)     // 应用起始地址（与你的 App .ld 保持一致 / Must match your App .ld start)
#define BOOT_MAGIC    0xB007B007U		// Boot 请求魔术值 / Boot request magic value

extern UART_HandleTypeDef huart2;
typedef void (*pFunction)(void);		// 跳转入口函数类型（Thumb 模式）/ Jump function type (Thumb mode)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static uint8_t bl_rxbuf[4096];   // 第二周：只写 RAM，不写 Flash
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ========== 工具函数：判断 APP 标志是否有效 ========== */
/* Check if App valid-flag exists */
static inline int app_flag_is_valid(void)
{
  return (*(volatile uint32_t*)APP_VALID_ADDR) == APP_VALID_MAGIC;
}

/* ========== 工具函数：判断 APP 是否完整有效 ========== */
/* Check if App is valid by stack pointer & reset vector */
static inline int app_is_valid(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;		// 初始主栈指针值 / Initial MSP
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U);// 复位向量（入口地址|1）/ Reset vector (entry | 1)

  // 栈应落在 SRAM(0x2000xxxx)，入口应落在 Flash(0x080xxxxx)
  // Stack must be in SRAM, reset handler must be in Flash
  if ((sp & 0x2FFE0000U) != 0x20000000U) return 0;
  if ((reset & 0xFF000000U) != 0x08000000U) return 0;
  return 1;
}
/* ========== 工具函数：执行跳转到 APP ========== */
/* Perform jump to App program */
static inline void jump_to_app(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;		// App 初始栈指针 / App initial MSP
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U);// App 入口地址（含 Thumb 位）/ App reset handler address
  pFunction app  = (pFunction)reset;                // 保留 Thumb 位，直接转换为函数指针

  __disable_irq();// 关中断 / Disable interrupts

  /* 停止 SysTick 并清除 NVIC，避免残留中断影响跳转 */
  /* Stop SysTick and clear NVIC to prevent residual interrupts */
  SysTick->CTRL = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* 恢复接近复位状态 / Reset clocks & peripherals */
  (void)HAL_RCC_DeInit();	// 复位时钟树（关闭 PLL 等）
  (void)HAL_DeInit();		// HAL 层外设反初始化
  /* 设置栈、重定位向量表，并执行跳转 */
  /* Set MSP, relocate vector table, then jump */
  __set_MSP(sp);                       // 设置主栈指针为 APP 的初值/
  SCB->VTOR = APP_ADDRESS;            // 重定位向量表到 APP（确保中断向量正确）
  __DSB(); __ISB();					 // 指令/数据同步屏障，确保前述寄存器写入生效

  __enable_irq();                     // 视需要开启中断（很多场景留着也没问题）
  app();                              // 跳转到 App，不会返回 / Jump to App (never returns)
}
/* 检查是否有 APP 请求进入 Boot */
/* Check if App requested Boot via backup register */
static inline uint8_t bl_request_pending(void)
{
    // 开启 PWR 时钟、解除备份域写保护
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;// 开启 PWR 时钟 / Enable PWR clock
    PWR->CR |= PWR_CR_DBP;			  // 解除备份域写保护 / Unlock backup domain
    RCC->BDCR |= RCC_BDCR_RTCEN;	  // 启用 RTC / Enable RTC


    if (RTC->BKP0R == BOOT_MAGIC) {
        RTC->BKP0R = 0;    // 清零防止循环 / Clear to prevent infinite loop
        return 1;
    }
    return 0;
}
/* SysTick 回调：500ms 翻转一次 LED */
/* SysTick callback: toggle LED every 500ms */
void HAL_SYSTICK_Callback(void)
{
    static uint32_t cnt = 0;
    if (++cnt >= 50) {                   // 500ms 翻转一次
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        cnt = 0;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	  /* ---------- 上电时最早检查按键和 Boot 请求 ---------- */
	  /* Earliest check at power-on: button state & Boot request */
	  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	  /* PC13 上拉输入（MODER 默认为输入，配置上拉即可） */
	  GPIOC->PUPDR &= ~(3U << (13U * 2U));
	  GPIOC->PUPDR |=  (1U << (13U * 2U));      /* 01: pull-up */
	  for (volatile uint32_t d = 0; d < 50000; ++d) __NOP();

	   uint8_t key_pressed = ((GPIOC->IDR & (1U << 13)) == 0U);  // 低=按下 / Low = pressed
	   uint8_t from_app_request = bl_request_pending();
	   if (!from_app_request && !key_pressed && app_is_valid() && app_flag_is_valid()) {
	       jump_to_app();// 如果条件满足直接跳转到 APP / Jump to App if conditions met
	   }




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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // 串口提示 Boot 已就绪 / Print Boot ready message
  const char *msg = "BL_READY\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

  // LED 快闪 5 次 / Blink LED quickly 5 times
  for (int i = 0; i < 5; i++) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(100);
  }

  // 初始化协议并进入任务循环 / Init protocol and enter task
  proto_init(&huart2, bl_rxbuf, sizeof(bl_rxbuf));
  proto_task();  // 一般不会返回 / Normally never returns
  /* USER CODE END 2 */

  /* Infinite loop */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
