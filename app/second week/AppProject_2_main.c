\
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body  (Week-2 App side)
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define APP_BASE  0x08004000U  // 应用固件在 Flash 中的起始地址（仅作说明，不参与该文件逻辑）
                               // Base address of the user App in Flash (for reference; unused in this file)

#define BOOT_MAGIC 0xB007B007U // “进入Bootloader”魔术值：写入备份寄存器后复位，Bootloader 读取到此值则进入升级模式
                               // Magic value to request Bootloader mode: write to backup register then reset;
                               // the Bootloader checks this value early after reset to decide to stay in update mode
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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief  通过写入备份寄存器并触发软复位，请求在下次上电时进入 Bootloader。
 *         Write a request into a backup register and trigger a soft reset so that
 *         on next boot the Bootloader detects the request and stays in update mode.
 *
 * 实现要点 / Key steps:
 *  1) 允许访问备份域（PWR 时钟 & 允许写备份域）
 *  2) 使能 RTC 以访问备份寄存器（BKP 寄存器位于 RTC 备份域）
 *  3) 将 BOOT_MAGIC 写入某个备份寄存器（这里选用 BKP0R）
 *  4) 触发 NVIC_SystemReset() 软复位
 *
 * 设计约定 / Design contract:
 *  - Bootloader 需在上电早期（时钟/外设初始化之前或之后尽快）读取 BKP0R；
 *    若值为 BOOT_MAGIC，则清除该寄存器并留在 Bootloader；否则跳转到 App。
 *  - Bootloader 负责清除该标志（避免反复进入），并在决定跳转到 App 之前恢复默认状态。
 *
 * 注意 / Notes:
 *  - 备份域在上电或 VBAT 断电后才会清空；普通芯片复位不会清除，因此非常适合跨复位传递“意图”。
 *  - 如果你的芯片/库不同，寄存器名可能不一样，请对应修改（例如某些系列使用 RTC->BKPxx）。
 */
void JumpToBootloader(void)
{
    __disable_irq();  // 关中断，防止过程中被打断
                      // Disable global IRQs to avoid being interrupted during transition

    /* 1) 允许访问备份域 (Backup domain) */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;  // 打开 PWR 时钟 / Enable PWR clock
    PWR->CR      |= PWR_CR_DBP;         // 允许写备份域 / Enable access to backup domain

    /* 2) 使能 RTC（仅为访问备份寄存器，不涉及时基配置） */
    RCC->BDCR |= RCC_BDCR_RTCEN;        // 使能 RTC（从而可访问 BKP 寄存器）
                                        // Enable RTC to access backup registers

    /* 3) 写入魔术值到 BKP0R：供 Bootloader 在早期读取判断 */
    RTC->BKP0R = BOOT_MAGIC;            // 写入“进入Bootloader”的标志
                                        // Write the "enter Bootloader" flag

    __DSB(); __ISB();                   // 同步屏障，确保寄存器写入已生效
                                        // Barriers to ensure register writes are committed

    /* 4) 触发软复位：复位后由 Bootloader 读取 BKP0R 决策 */
    NVIC_SystemReset();                 // 软件复位（不会返回）
                                        // Soft reset (does not return)
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 在需要进入 Bootloader 的场景调用 JumpToBootloader()，例如：
   - 检测到用户按键长按 / 特定串口命令 / 主机工具请求
   - 固件自检失败或 App 无效，需要退回 Bootloader 进行修复

   Call JumpToBootloader() when you want to enter the Bootloader, e.g.:
   - Long-press on user button / special UART command / host tool request
   - Self-test failure or invalid App image that requires recovery via Bootloader */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* （此处可放置在 HAL_Init 之前需要做的用户早期判断/配置）
     Place any very-early user checks/config here if needed (before HAL_Init) */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* HAL 初始化：复位外设、初始化 Flash 接口、初始化 SysTick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 这里可以初始化与“进入Bootloader”有关的输入条件，例如按键/串口命令解析等。
     For Week-2 demo, you could check a button or parse a UART command,
     then call JumpToBootloader() to request Boot mode via backup register. */
  /* USER CODE END Init */

  /* 配置系统时钟（此处使用默认 HSI，不开 PLL；满足 LED 闪烁足够） */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* （可在此加入与系统相关的自定义初始化）
     Add any system-level custom init here if necessary */
  /* USER CODE END SysInit */

  /* 初始化 GPIO（将 PA5 配置为推挽输出） */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  /* 上电提示：快速闪烁 3 次，表示 APP 已启动
     Power-on indication: blink 3 times to show the App has started */
  for (int i = 0; i < 3; ++i) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(80);
  }

  /* Week-2 建议：
     - 如果检测到“进入Bootloader”的触发条件（例如按键被按下），可调用 JumpToBootloader();
     - 否则继续运行应用逻辑。

     For Week-2:
     - If an entry condition is met (e.g., user button pressed), call JumpToBootloader();
     - Otherwise, continue normal app flow. */
  /* 主循环：LD2 每 500ms 翻转一次 / Toggle LED every 500ms */
  while (1) {
     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
     HAL_Delay(500);

     /* 示例：当检测到某条件时进入 Bootloader（伪代码）
        Example (pseudo):
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) {
            JumpToBootloader();
        } */
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* 注：如果要用到用户按键（PC13）来触发 JumpToBootloader，可以在 CubeMX 里把 PC13 配置成输入并在循环里读取。
     Note: To use the user button (PC13) as a trigger, configure PC13 as input in CubeMX and read it in the loop. */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* （可在此添加与第二周相关的其他用户函数，例如 UART 回显、协议包解析到 RAM、CRC 计算等占位）
   Place additional Week-2 helpers here (e.g., UART echo, packet buffering in RAM, CRC stubs) */
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
