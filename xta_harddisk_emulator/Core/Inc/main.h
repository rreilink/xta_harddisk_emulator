/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define A0_Pin GPIO_PIN_13
#define A0_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_14
#define A1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_15
#define A2_GPIO_Port GPIOC
#define DIN0_Pin GPIO_PIN_0
#define DIN0_GPIO_Port GPIOC
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOC
#define DIN2_Pin GPIO_PIN_2
#define DIN2_GPIO_Port GPIOC
#define DIN3_Pin GPIO_PIN_3
#define DIN3_GPIO_Port GPIOC
#define DOUT0_Pin GPIO_PIN_0
#define DOUT0_GPIO_Port GPIOA
#define DOUT1_Pin GPIO_PIN_1
#define DOUT1_GPIO_Port GPIOA
#define DOUT2_Pin GPIO_PIN_2
#define DOUT2_GPIO_Port GPIOA
#define DOUT3_Pin GPIO_PIN_3
#define DOUT3_GPIO_Port GPIOA
#define DOUT4_Pin GPIO_PIN_4
#define DOUT4_GPIO_Port GPIOA
#define DOUT5_Pin GPIO_PIN_5
#define DOUT5_GPIO_Port GPIOA
#define DOUT6_Pin GPIO_PIN_6
#define DOUT6_GPIO_Port GPIOA
#define DOUT7_Pin GPIO_PIN_7
#define DOUT7_GPIO_Port GPIOA
#define DIN4_Pin GPIO_PIN_4
#define DIN4_GPIO_Port GPIOC
#define DIN5_Pin GPIO_PIN_5
#define DIN5_GPIO_Port GPIOC
#define nWR_Pin GPIO_PIN_2
#define nWR_GPIO_Port GPIOB
#define DIN6_Pin GPIO_PIN_6
#define DIN6_GPIO_Port GPIOC
#define DIN7_Pin GPIO_PIN_7
#define DIN7_GPIO_Port GPIOC
#define nIO_RDY_Pin GPIO_PIN_8
#define nIO_RDY_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_9
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_10
#define USART_RX_GPIO_Port GPIOA
#define nCS_Pin GPIO_PIN_10
#define nCS_GPIO_Port GPIOC
#define RDDMA_Pin GPIO_PIN_4
#define RDDMA_GPIO_Port GPIOB
#define RDREG_Pin GPIO_PIN_5
#define RDREG_GPIO_Port GPIOB
#define DRQ_Pin GPIO_PIN_7
#define DRQ_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_8
#define IRQ_GPIO_Port GPIOB
#define VUSB_ENA_Pin GPIO_PIN_9
#define VUSB_ENA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
