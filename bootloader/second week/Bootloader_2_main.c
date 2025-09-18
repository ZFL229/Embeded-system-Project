/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Bilingual annotated version / 中英双语注释版)
  ******************************************************************************
  * @attention
  
  ******************************************************************************
  */
  /* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "proto.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* ========================== Boot / App 基本参数 ==========================
 * APP_ADDRESS:  应用程序（App）在片上 Flash 的起始地址；
 *               必须与 App 工程的链接脚本(.ld / .icf)对齐一致。
 *               这里示例为 0x08004000，通常意味着 BootLoader 占用 16KB。
 *
 * BOOT_MAGIC:   用于“从 App 请求跳回 Boot”的魔数，写入到备份寄存器后，
 *               复位后 Boot 读取到该值即可进入 Boot 模式并清除之。
 * ------------------------------------------------------------------------
 * APP_ADDRESS:  Start address of the user App in on-chip Flash.
 *               MUST match the App project's linker script.
 * BOOT_MAGIC:   A magic value used by App to request entering Boot after reset.
 *               Stored in backup register; Boot checks and clears it.
 */
#define APP_ADDRESS  (0x08004000U)     // 应用起始地址（与你的 App .ld 保持一致 / MUST match App .ld)
#define BOOT_MAGIC   0xB007B007U       // 备份域魔数 / magic value for Boot request

/* 说明（NOTE）:
 * 在使用 HAL_UART_Transmit / Receive 之前，需要完成 USART2 的初始化。
 * 这里声明 huart2 为外部句柄，以便在其他模块(如 proto.c)中使用相同句柄。
 *
 * We declare 'huart2' so it can be used by protocol code and elsewhere.
 */
extern UART_HandleTypeDef huart2;

/* pFunction 是 C 函数指针类型，签名为空参/无返回，用于跳转到 App 的入口。
 * 由于 Cortex-M 处于 Thumb 指令集模式，入口地址最低位应为 1（Thumb 位）。
 *
 * pFunction is a function pointer type for the App's Reset_Handler.
 * On Cortex-M (Thumb state), the LSB of the address must be 1.
 */
typedef void (*pFunction)(void);  // 跳转入口函数类型（Thumb 模式）

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2; // 注意：CubeMX 也会生成同名句柄，这里与其保持一致

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* 在第二周要求中，仅在 RAM 中进行缓冲，不进行 Flash 写入；
 * 因此使用一个较大的 UART 接收缓冲区（4KB）用于协议解析。
 *
 * Per assignment, week 2 stores data in RAM only (no Flash).
 * So we allocate a 4KB receive buffer for the boot protocol.
 */
static uint8_t bl_rxbuf[4096];   // 第二周：只写 RAM，不写 Flash / RAM-only RX buffer

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* --------------------------------------------------------------------------
 * 工具函数：判断 App 是否有效
 * Validate if the App image at APP_ADDRESS looks valid.
 *
 * 校验规则（Heuristics）:
 * 1) 首地址存放的是“主栈指针初值（MSP）”，应当位于 SRAM 区域 (0x2000_0000~)；
 * 2) 入口地址（复位向量）应当落在 Flash 区域 (0x0800_0000~)，并带有 Thumb 位（LSB=1）。
 * 这些检查并不保证固件完全正确，但能过滤掉大多数无效/空白区域。
 * -------------------------------------------------------------------------- */
static inline int app_is_valid(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;        // 初始主栈指针值 / initial MSP
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U); // 复位向量（入口地址|1）/ reset vector (entry|1)

  /* 栈地址应落在 SRAM（0x2000xxxx）；入口地址应落在 Flash（0x080xxxxx）
     注意：reset 最低位为 1（Thumb 位），这里用掩码判断高位区域即可
     MSP must point to SRAM; entry must be in Flash. LSB of entry is Thumb bit.
   */
  if ((sp & 0x2FFE0000U) != 0x20000000U) return 0;   // 非 0x2000_xxxx 视为非法 / not in SRAM => invalid
  if ((reset & 0xFF000000U) != 0x08000000U) return 0; // 非 0x080x_xxxx 视为非法 / not in Flash => invalid
  return 1; // 通过启发式校验 / heuristically valid
}

/* --------------------------------------------------------------------------
 * 工具函数：执行跳转到 App
 * Jump to the user App's Reset_Handler at APP_ADDRESS.
 *
 * 关键步骤（Key steps）:
 * 1) 关闭全局中断、停止 SysTick、清 NVIC 使能/挂起，避免残留中断影响；
 * 2) HAL_RCC_DeInit + HAL_DeInit，将时钟与外设尽量还原到接近复位态；
 * 3) 设置 MSP = *(APP_ADDRESS)，重定位向量表到 APP，执行数据/指令同步；
 * 4) 以函数指针形式调用入口地址（带 Thumb 位）。
 * -------------------------------------------------------------------------- */
static inline void jump_to_app(void)
{
  uint32_t sp    = *(volatile uint32_t *)APP_ADDRESS;        // 读取 App 的初始栈 / load App's MSP
  uint32_t reset = *(volatile uint32_t *)(APP_ADDRESS + 4U); // 读取入口地址（含 Thumb 位）/ App entry|1
  pFunction app  = (pFunction)reset;                         // 保留 Thumb 位，直接转换为函数指针 / keep LSB=1

  __disable_irq(); // 关全局中断 / disable global IRQs

  /* 1) 停止 SysTick 并清除所有 NVIC 使能/挂起，避免残留中断影响跳转
   *    Stop SysTick; clear NVIC enables/pending flags to avoid stray IRQs.
   */
  SysTick->CTRL = 0;
  for (uint32_t i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* 2) 反初始化：恢复到接近上电复位的时钟/外设状态
   *    De-init clocks & HAL peripherals to a near-reset state.
   *   （注意：若 App 依赖特定时钟设置，它会在 SystemInit() / HAL_Init() 自行配置。）
   */
  (void)HAL_RCC_DeInit();
  (void)HAL_DeInit();

  /* 3) 切换到 APP 的运行环境
   *    Switch to App's environment: MSP, VTOR, and barriers.
   */
  __set_MSP(sp);                 // 设置主栈指针为 APP 初值 / set MSP to App's initial value
  SCB->VTOR = APP_ADDRESS;       // 重定位向量表到 APP / relocate vector table
  __DSB(); __ISB();              // 同步屏障，确保写入生效 / barriers to flush pipeline

  __enable_irq();                // 可选：允许中断 / optionally re-enable IRQs
  app();                         // 跳转到 Reset_Handler（通常不会返回）/ jump, never returns
}

/* --------------------------------------------------------------------------
 * 工具函数：检查“从 App 请求进入 Boot”的标志
 * Check if the App has requested to enter Boot via backup register.
 *
 * 机制（Mechanism）:
 * - App 在复位前（或某命令）将 BOOT_MAGIC 写入 RTC 备份寄存器 BKP0R；
 * - Boot 上电/复位后读取该寄存器，若命中则清零并停留在 Boot 模式。
 * 注意：需开启 PWR 时钟并允许写备份域（DBP），同时确保 RTCEN 使能。
 * -------------------------------------------------------------------------- */
static inline uint8_t bl_request_pending(void)
{
    // 开启 PWR 时钟、解除备份域写保护 / enable PWR & allow backup domain write
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_DBP;
    RCC->BDCR |= RCC_BDCR_RTCEN;

    if (RTC->BKP0R == BOOT_MAGIC) {
        RTC->BKP0R = 0;   // 清零，防止循环停在 Boot / clear to avoid loop
        return 1;         // 检测到请求 / request present
    }
    return 0;             // 无请求 / no request
}

/* --------------------------------------------------------------------------
 * Systick 回调：以 500ms 周期翻转板载 LED（PA5）
 * Toggle LD2 (PA5) every 500ms via SysTick callback.
 *
 * 用途（Purpose）:
 * - 在 Boot 停留时给出可视化心跳指示；
 * - 该回调基于 HAL_IncTick 驱动，HAL_Init() 后生效。
 * -------------------------------------------------------------------------- */
void HAL_SYSTICK_Callback(void)
{
    static uint32_t cnt = 0;
    if (++cnt >= 50) {                   // 50 * 10ms = 500ms (SysTick 默认 1kHz)
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // 翻转 LD2 / toggle LD2
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
  /* ------------------------------------------------------------------------
   * 早期最小化按键检测（Before HAL_Init）
   * Minimal key scan BEFORE HAL_Init to decide Boot/App path ASAP.
   *
   * 目的（Why early?）:
   * - 在尚未初始化 HAL/时钟/外设前，尽快做出 Boot/跳 App 的决策，
   *   避免多余初始化导致的时间浪费或视觉“抖动”。
   * - PC13（用户按键）默认上拉。按下=低电平。
   * 
   * Steps:
   * 1) 仅打开 GPIOC 时钟；
   * 2) 将 PC13 配置为上拉输入；
   * 3) 简短延时后读取电平；
   * 4) 同时检查备份域魔数（从 App 的进入 Boot 请求）；
   * 5) 若“未按键 && 未请求 && App 有效”，则直接跳转到 App。
   * ------------------------------------------------------------------------ */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;               // 1) 仅开 GPIOC 时钟 / enable GPIOC
  GPIOC->PUPDR &= ~(3U << (13U * 2U));
  GPIOC->PUPDR |=  (1U << (13U * 2U));               // PC13 上拉输入 / pull-up
  for (volatile uint32_t d = 0; d < 50000; ++d) __NOP(); // 短延时 / small settle delay

  uint8_t key_pressed     = ((GPIOC->IDR & (1U << 13)) == 0U);  // 低=按下 / low means pressed
  uint8_t from_app_request = bl_request_pending();               // App 请求标志 / request flag

  // 进入 App 的条件：未按键、无请求、且 App 通过启发式校验
  // Jump to App if: no key, no request, and App looks valid.
  if (!from_app_request && !key_pressed && app_is_valid()) {
      jump_to_app(); // 不返回 / does not return
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init(); // 从此处开始，HAL 时基/中断等机制生效 / HAL tick & IRQs start working

  /* USER CODE BEGIN Init */
  /* 可在此添加与设备无关的全局初始化（若需要）。
   * Place generic init here if needed.
   */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* 若 Boot 需要特殊系统配置，可在此处扩展。
   * Extend system-level setups for Boot here if necessary.
   */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  /* ------------------------------------------------------------------------
   * Boot 提示与协议调度（Boot prompt & protocol loop）
   *
   * 1) 串口提示 "BL_READY"：用于与上位机脚本（Python 等）建立握手；
   * 2) LED 快闪 5 次：上电视觉提示；
   * 3) 初始化简易协议栈（proto_init）：传入 UART 句柄与接收缓冲区；
   * 4) 进入协议任务（proto_task）：阻塞式轮询/处理命令，正常情况下不返回。
   * 
   * Tips:
   * - 若后续需要扩展非阻塞任务，可将 proto_task 改为周期轮询，或放入 RTOS 任务中。
   * ------------------------------------------------------------------------ */
  const char *msg = "BL_READY\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY); // 上电提示 / boot banner

  // LED 快闪 5 次（100ms 周期）/ Quick blink 5x @100ms
  for (int i = 0; i < 5; i++) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      HAL_Delay(100);
  }

  // 协议初始化（传入 UART 句柄与 RAM 缓冲）/ protocol init
  proto_init(&huart2, bl_rxbuf, sizeof(bl_rxbuf));

  // 进入协议任务（阻塞，不返回）/ run protocol task (normally never returns)
  proto_task();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    // 按设计，代码不会到达这里；若到达，说明 proto_task 意外返回。
    // Normally unreachable unless proto_task() returns unexpectedly.
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
  /* 保持由 CubeMX 生成的标准初始化流程。
   * Keep the standard init flow auto-generated by CubeMX.
   */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* 如需与上位机脚本匹配其它波特率，请同步修改 huart2.Init.BaudRate。
   * If host script uses another baudrate, adjust huart2.Init.BaudRate accordingly.
   */
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
  /* 初始化完成。此后可直接使用 HAL_UART_Transmit/Receive 等 API。
   * After init, HAL UART APIs are ready for use.
   */
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
  /* 注意：这里再次初始化 PC13 上拉输入与 LD2 输出；
   * 早期在 main() 顶部已经做过一次“极简初始化”（只为决策早跳 App ），
   * 这里是 CubeMX 风格的常规初始化，不冲突。
   * We re-init PC13 and LD2 here in the standard CubeMX style.
   */
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

  /*Configure GPIO pin : LD2_Pin (PA5) */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* 若需在 Boot 中使用外部中断（例如长按触发），可在此改为 EXTI 模式并配置回调。
   * If you need EXTI for key later (e.g., long-press), switch to EXTI here.
   */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 可在此处添加与 Boot 相关的辅助函数实现（若需要）。
 * Add more Boot helper functions here if needed.
 */
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
    /* 可在此添加错误指示（如快速闪烁 LED）
     * You may add an error indicator here, e.g., fast LED blinking.
     */
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
