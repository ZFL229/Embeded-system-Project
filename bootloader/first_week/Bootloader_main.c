\
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (BOOT)
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
#include "stm32f4xx_hal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_ADDRESS  (0x08004000U)     // 应用起始地址；需与应用工程的链接脚本(.ld)一致
                                       // Application start address; must match the App project's linker script (.ld)

typedef void (*pFunction)(void);       // 跳转入口函数类型（Thumb 模式）
                                       // Function pointer type to App's Reset_Handler (Thumb state)
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
/* 在此声明用户辅助函数的原型（若需要）
   Declare prototypes for user helper functions here (if needed) */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==========================
 * 工具函数：判断应用是否有效
 * Validate whether an application image is present and plausible
 * --------------------------
 * 判定原则 / Heuristic:
 *  1) *(APP_ADDRESS) 作为初始主栈指针 MSP，必须位于 SRAM 区域 (0x2000_0000 开头)
 *  2) *(APP_ADDRESS + 4) 作为复位向量（入口地址|1），应指向 Flash 区域 (0x0800_0000 开头)
 *  注意：入口地址最低位为 1（Thumb 位）；下方通过掩码判断其高位区域即可。
 *  Tips: This is a lightweight plausibility check; for robust validation,
 *        consider adding CRC or a “valid flag” at a known location.
 */
static inline int app_is_valid(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;        // 读取应用的初始栈指针值 / App's initial MSP
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U); // 读取复位向量（入口地址|1）/ App Reset vector (entry addr | 1)

  // 栈地址需落在 0x2000_xxxx；入口地址需落在 0x080x_xxxx
  // MSP must point into SRAM; entry must point into Flash
  if ((sp & 0x2FFE0000U) != 0x20000000U) return 0;
  if ((reset & 0xFF000000U) != 0x08000000U) return 0;
  return 1;
}

/* ==========================
 * 工具函数：跳转至应用程序
 * Jump to user application
 * --------------------------
 * 步骤 / Steps:
 *  1) 关中断；停止 SysTick；清除 NVIC 使能/挂起，避免残留中断
 *  2) 反初始化时钟与外设（HAL_RCC_DeInit / HAL_DeInit）
 *  3) 切换到应用环境：设置 MSP 为应用初值；重定位向量表 SCB->VTOR = APP_ADDRESS
 *  4) 指令/数据同步屏障（DSB/ISB）后，调用应用的 Reset_Handler（不会返回）
 *  提示：不要清除复位向量最低位（Thumb 位），直接按函数指针调用。
 *  Tip: Keep the LSB (Thumb bit) of the reset address when casting to function pointer.
 */
static inline void jump_to_app(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;        // App MSP value
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U); // App entry (Reset_Handler | 1)
  pFunction app  = (pFunction)reset;                         // 保留 Thumb 位，直接转换 / keep LSB, cast to function pointer

  __disable_irq();            // 关全局中断 / Disable global IRQs

  // 停止 SysTick，清除所有 NVIC 使能与挂起位 / Stop SysTick and clear NVIC state
  SysTick->CTRL = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  // 反初始化：恢复到接近上电复位的状态（避免留下 Bootloader 配置）
  // De-init clocks & peripherals to avoid leaving Boot config behind
  (void)HAL_RCC_DeInit();
  (void)HAL_DeInit();

  // 切换到应用环境 / Switch context to App
  __set_MSP(sp);                  // 设置主栈指针 / Set MSP to App's initial value
  SCB->VTOR = APP_ADDRESS;        // 重定位中断向量表 / Relocate vector table
  __DSB(); __ISB();               // 同步屏障，确保寄存器写入生效 / Barriers to ensure effects

  __enable_irq();                 // 若需要可重新开中断 / Optionally re-enable IRQs
  app();                          // 跳转到 App 的 Reset_Handler（通常不返回）/ Jump (no return)
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ========= Boot 决策与指示 / Boot decision & indicator =========
   * 说明 / Notes:
   * - 若要在“非常早期”读取用户键（PC13）以决定是否进入 Bootloader，
   *   可以只打开 GPIOC 时钟并将 PC13 配置为输入，再做判断以减少干扰。
   * - 当前模板直接进入 Bootloader 初始化流程，并以 LED 快闪 5 次作为停留指示。
   *
   * If you want to read the user button (PC13) very early (before full HAL init)
   * to decide whether to stay in Bootloader, you can enable only GPIOC clock and
   * sample PC13 first. In this template we proceed with normal init and indicate
   * Bootloader by 5 fast LED blinks.
   */

  // 初始化 HAL / clocks / GPIO 仅当“决定留在 Bootloader”时再进行（示范流程）
  // Proceed with full init when we decide to stay in Bootloader (demo)
  HAL_Init();               // 复位外设、初始化 Flash 接口与 SysTick / Reset peripherals, init Flash & SysTick
  SystemClock_Config();     // 配置系统时钟（此处使用 PLL，见下文）/ Configure clocks (PLL enabled)
  MX_GPIO_Init();           // 初始化 LD2(PA5) 与 B1(PC13) / Init LED & Button

  // LED 快速闪烁 5 次：表示当前在 Bootloader 模式
  // Blink LED 5 times quickly to indicate Bootloader mode
  for (int i = 0; i < 5; ++i) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(80);
  }

  // 可选：如应用有效且未触发更新，则直接跳转到 App
  // Optional: if App image is valid and no update requested, jump to App
  if (app_is_valid()) {
    // TODO: 根据你的按键/标志位决定是否跳转（此处示例直接跳转）
    // TODO: Decide by button/flag; here we jump unconditionally if valid
    // jump_to_app();
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
  /* USER CODE BEGIN 2 */
  /* 预留区域：在此初始化 UART/CRC/协议收发等（第 2~3 周任务）
     Placeholder: initialize UART/CRC/protocol I/O here (Week 2–3) */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* 主循环：心跳指示（100ms 翻转 LED 一次），后续可在此处理命令/数据包
     Main loop: heartbeat (toggle LED every 100ms); extend to handle protocol */
  while (1)
  {
    /* USER CODE END WHILE */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(100);
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
  /* 说明：PC13 为用户按键（上拉，按下为低）；PA5 为板载 LED。
     Note: PC13 is user button (pull-up, pressed=LOW); PA5 is the onboard LED. */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 在此处添加你的 Bootloader 扩展逻辑（如 UART 接收、协议解析、写 Flash、CRC 校验等）
   Add your Bootloader extensions here: UART RX, protocol parsing, Flash write, CRC, etc. */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 可以在此加入错误指示（如快速闪烁 LED）或记录错误码
     You may add error indication here (e.g., fast LED blink) or log an error code */
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
  /* 可在此输出断言失败信息，帮助定位参数错误
     You can print assert failure info here for diagnostics */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
