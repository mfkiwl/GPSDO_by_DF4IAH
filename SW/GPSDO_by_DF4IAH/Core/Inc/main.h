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
#define OCXO_RCC_CK_IN_Pin GPIO_PIN_0
#define OCXO_RCC_CK_IN_GPIO_Port GPIOA
#define TERM_GPS_USART2_TX_Pin GPIO_PIN_2
#define TERM_GPS_USART2_TX_GPIO_Port GPIOA
#define DCF77_CAR_TIM2_CH4_Pin GPIO_PIN_3
#define DCF77_CAR_TIM2_CH4_GPIO_Port GPIOA
#define V_OCXO_ADC1_IN9_Pin GPIO_PIN_4
#define V_OCXO_ADC1_IN9_GPIO_Port GPIOA
#define V_HOLD_ADC1_IN10_Pin GPIO_PIN_5
#define V_HOLD_ADC1_IN10_GPIO_Port GPIOA
#define V_DCF77_DEMOD_ADC1_IN16_Pin GPIO_PIN_1
#define V_DCF77_DEMOD_ADC1_IN16_GPIO_Port GPIOB
#define SER_GPS_EN_GPIO_O_Pin GPIO_PIN_8
#define SER_GPS_EN_GPIO_O_GPIO_Port GPIOA
#define PLL_LCKD_GPIO_I_Pin GPIO_PIN_11
#define PLL_LCKD_GPIO_I_GPIO_Port GPIOA
#define GPS_LCKD_GPIO_I_Pin GPIO_PIN_12
#define GPS_LCKD_GPIO_I_GPIO_Port GPIOA
#define TERM_USART2_RX_Pin GPIO_PIN_15
#define TERM_USART2_RX_GPIO_Port GPIOA
#define GPS_PPS_TIM2_CH2_Pin GPIO_PIN_3
#define GPS_PPS_TIM2_CH2_GPIO_Port GPIOB
#define RLY_HOLD_GPIO_O_Pin GPIO_PIN_4
#define RLY_HOLD_GPIO_O_GPIO_Port GPIOB
#define FRCD_HOLD_GPIO_I_Pin GPIO_PIN_5
#define FRCD_HOLD_GPIO_I_GPIO_Port GPIOB
#define ONEWIRE_USART1_TX_Pin GPIO_PIN_6
#define ONEWIRE_USART1_TX_GPIO_Port GPIOB
#define DCF77_DEMOD_GPIO_EXTI7_Pin GPIO_PIN_7
#define DCF77_DEMOD_GPIO_EXTI7_GPIO_Port GPIOB
#define BOOT0_GPIO_I_Pin GPIO_PIN_3
#define BOOT0_GPIO_I_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
