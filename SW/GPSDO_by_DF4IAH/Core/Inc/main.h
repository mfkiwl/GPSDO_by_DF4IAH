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
#include "stm32l4xx_hal.h"

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
#define D7_Forced_Holdover_GPIO_I_Pin GPIO_PIN_14
#define D7_Forced_Holdover_GPIO_I_GPIO_Port GPIOC
#define A0_Lock_indic_GPIO_I_Pin GPIO_PIN_0
#define A0_Lock_indic_GPIO_I_GPIO_Port GPIOA
#define A1_V_OCXO_ADC1_IN6_Pin GPIO_PIN_1
#define A1_V_OCXO_ADC1_IN6_GPIO_Port GPIOA
#define A7_Onewire_USART2_TX_Pin GPIO_PIN_2
#define A7_Onewire_USART2_TX_GPIO_Port GPIOA
#define A2_Onewire_USART2_TX_Bridged_Pin GPIO_PIN_3
#define A2_Onewire_USART2_TX_Bridged_GPIO_Port GPIOA
#define A3_V_Holdover_ADC1_IN9_Pin GPIO_PIN_4
#define A3_V_Holdover_ADC1_IN9_GPIO_Port GPIOA
#define A4_I2C3_SDA_Bridged_Pin GPIO_PIN_5
#define A4_I2C3_SDA_Bridged_GPIO_Port GPIOA
#define A5_I2C3_SCL_Bridged_Pin GPIO_PIN_6
#define A5_I2C3_SCL_Bridged_GPIO_Port GPIOA
#define A6_I2C3_SCL_Pin GPIO_PIN_7
#define A6_I2C3_SCL_GPIO_Port GPIOA
#define D3_GPS_Module_TX_Bridged_Pin GPIO_PIN_0
#define D3_GPS_Module_TX_Bridged_GPIO_Port GPIOB
#define D6_HoRelay_GPIO_O_Pin GPIO_PIN_1
#define D6_HoRelay_GPIO_O_GPIO_Port GPIOB
#define D9_PLL_CLK_TIM1_CH1_Pin GPIO_PIN_8
#define D9_PLL_CLK_TIM1_CH1_GPIO_Port GPIOA
#define D1_GPS_Module_USART1_TX_Pin GPIO_PIN_9
#define D1_GPS_Module_USART1_TX_GPIO_Port GPIOA
#define D0_GPS_CLK_TIM1_CH3_Pin GPIO_PIN_10
#define D0_GPS_CLK_TIM1_CH3_GPIO_Port GPIOA
#define D12_I2C3_SDA_Pin GPIO_PIN_4
#define D12_I2C3_SDA_GPIO_Port GPIOB
#define D5_GPS_locked_GPIO_I_Pin GPIO_PIN_6
#define D5_GPS_locked_GPIO_I_GPIO_Port GPIOB
#define D4_GPS_Module_USART1_RX_Pin GPIO_PIN_7
#define D4_GPS_Module_USART1_RX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
