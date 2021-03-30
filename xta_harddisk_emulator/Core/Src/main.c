/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

HCD_HandleTypeDef hhcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_HCD_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t sample=0;
uint8_t regs[9]={0};
volatile uint8_t irq_drq_reg = 0;
volatile uint8_t irq_drq_val = 0;
uint8_t command_block[6];
uint8_t command_block_idx = 0;

#define DRIVE_TYPE 4 // see https://www.win.tue.nl/~aeb/linux/hdtypes/hdtypes-3.html section 3.2

uint8_t sense_block[0xe] = {0,0,0,0,0,0,0,0,0,0,0,0,DRIVE_TYPE,0};
uint8_t sense_block_idx = 0;

volatile uint8_t command_execute = 0;
volatile int data_idx = 0;

const uint32_t MODER_INPUTS = 0;
const uint32_t MODER_OUTPUTS = 0x5555;


static inline void irq_drq_update() {
	GPIOB->ODR = (GPIOB->ODR & ~0x300) | (irq_drq_reg & irq_drq_val)<<8;
}

static const char data[512] =
		"\xb8\x52\x0e\xbb\x00\x00\xcd\x10\xeb\xfe"
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"                                                  "
		"\x55\xaa"

		;

uint8_t access_log[256];
int access_log_idx = 0;
/* IO access interrupt
 *
 */
void EXTI1_IRQHandler(void)
{


	sample = GPIOC->IDR;
	uint8_t address = (sample >> 8) & 7;
	if (!(sample & nDACK_Pin)) address = 8;

	if (access_log_idx<256) {
		access_log[access_log_idx++] = address | ((sample & nIORD_Pin) ? 0x10:0);
	}

	if (sample & nIORD_Pin) {
		nIORDY_GPIO_Port->ODR &= ~nIORDY_Pin; // IO is ready

		// write access
		GPIOA->BSRR = (1<<5);


		if (address == 0) {
			if ((command_block_idx<6) && (!command_execute)) {
				command_block[command_block_idx++] = sample;
				if (command_block_idx==6) {
					command_execute = 1;
				}
			}
		}

		if (address == 2) { // irq/drq mask register
			if (sample & 0x80) { // reset
				// reset. Probably cannot do a processor reset since the host will start writing to us right-away
				command_block_idx = 0;
				irq_drq_val = 2; // BIOS expects an interrupt upon reset completion

			}
			irq_drq_reg = sample;
			irq_drq_update();
			// todo: set irq/drq
		} else if(address == 4) {
			if (sample & 0x80) { // start of write_command_block_to_controller
				command_block_idx = 0;
				irq_drq_val = 0;
				irq_drq_update();
			}
			if (sample & 0x20) { // start of sense operation
				sense_block_idx = 0;
				regs[0] = sense_block[sense_block_idx++];

			}


		}


		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
		GPIOA->BSRR = (0x10000<<5);


		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));

		nIORDY_GPIO_Port->ODR |=nIORDY_Pin;  // IO is not ready (next access)


	} else {
		// read access

		GPIOC->MODER = MODER_OUTPUTS;
		GPIOC->ODR=regs[address];

		nIORDY_GPIO_Port->ODR &= ~nIORDY_Pin; // IO is ready

		GPIOA->BSRR = (1<<5);

		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);

		while(!(nIOACCESS_GPIO_Port->IDR & nIOACCESS_Pin));
		GPIOC->MODER = MODER_INPUTS;

		nIORDY_GPIO_Port->ODR |=nIORDY_Pin; // IO is not ready (next access)


		if (address == 8) { // DMA data transfer
			if (data_idx==512) { // transfer completed
				irq_drq_val &=~1; // clear DRQ
				irq_drq_update();
			} else {
				regs[8] = data[data_idx++];
			}

		} else if (address == 0) { // sense register
			if (sense_block_idx<0xe) {
				regs[0] = sense_block[sense_block_idx++];
			}
		}



		GPIOA->BSRR = (0x10000<<5);

	}










}


int _write(int fd, char* ptr, int len) {
  HAL_StatusTypeDef hstatus;

  hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  if (hstatus == HAL_OK) {
    return len;
  } else {
    return 0;
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
  MX_USB_OTG_FS_HCD_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  regs[0] = 5; // data

  //322 read:
  regs[2] = 0x12; // busy/sync bit4=ready for data write to 320/read from 320. bit2=command busy   bit1=command accepted

  //322 write: dreq / int mask

  regs[4] = 0; // result; bit 7 = error

  regs[8] = 0x52; // DMA register


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  nIORDY_GPIO_Port->ODR |=nIORDY_Pin;

  printf("Starting\n");
  while (1)
  {
	  if (command_execute) {


		  printf("Command %02x size=%d\n", command_block[0], command_block[5]);
		  command_block_idx = 0;

		  if (command_block[0]==0x15) {
			  data_idx = 0;
			  regs[8] = data[data_idx++];
			  irq_drq_val |= 1; // set DRQ
			  irq_drq_update();

			  HAL_Delay(20);

			  irq_drq_val &= ~1; // clear DRQ
			  irq_drq_update();


		  }

		  irq_drq_val |= 2; // set IRQ
		  irq_drq_update();
		  command_execute = 0;
	  }


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
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
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_HCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hhcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hhcd_USB_OTG_FS.Init.Host_channels = 8;
  hhcd_USB_OTG_FS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_FS.Init.phy_itface = HCD_PHY_EMBEDDED;
  hhcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  if (HAL_HCD_Init(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nIORDY_Pin|DRQ_Pin|IRQ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D2_Pin D3_Pin
                           D4_Pin D5_Pin D6_Pin D7_Pin
                           nIORD_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D2_Pin|D3_Pin
                          |D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |nIORD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nIOACCESS_Pin */
  GPIO_InitStruct.Pin = nIOACCESS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nIOACCESS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nIORDY_Pin DRQ_Pin IRQ_Pin */
  GPIO_InitStruct.Pin = nIORDY_Pin|DRQ_Pin|IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin nDACK_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|nDACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
