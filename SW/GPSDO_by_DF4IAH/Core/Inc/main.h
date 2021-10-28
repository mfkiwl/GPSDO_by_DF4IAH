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
#define NoD7_RCC_OSC32_IN_Pin GPIO_PIN_14
#define NoD7_RCC_OSC32_IN_GPIO_Port GPIOC
#define NoD8_RCC_OSC32_OUT_Pin GPIO_PIN_15
#define NoD8_RCC_OSC32_OUT_GPIO_Port GPIOC
#define A0_OCXO_RCC_CK_IN_Pin GPIO_PIN_0
#define A0_OCXO_RCC_CK_IN_GPIO_Port GPIOA
#define A1_SPI1_SCK_Pin GPIO_PIN_1
#define A1_SPI1_SCK_GPIO_Port GPIOA
#define NoA7_TERMINAL_USART2_TX_Pin GPIO_PIN_2
#define NoA7_TERMINAL_USART2_TX_GPIO_Port GPIOA
#define A2_DCF77_PHASE_TIM2_CH4_Pin GPIO_PIN_3
#define A2_DCF77_PHASE_TIM2_CH4_GPIO_Port GPIOA
#define A3_V_OCXO_ADC1_IN9_Pin GPIO_PIN_4
#define A3_V_OCXO_ADC1_IN9_GPIO_Port GPIOA
#define A4_V_HOLD_ADC1_IN10_Pin GPIO_PIN_5
#define A4_V_HOLD_ADC1_IN10_GPIO_Port GPIOA
#define A5_SPI1_MISO_Pin GPIO_PIN_6
#define A5_SPI1_MISO_GPIO_Port GPIOA
#define A6_SPI1_MOSI_Pin GPIO_PIN_7
#define A6_SPI1_MOSI_GPIO_Port GPIOA
#define D3_DCF77_DEMOD_GPIO_EXTI0_Pin GPIO_PIN_0
#define D3_DCF77_DEMOD_GPIO_EXTI0_GPIO_Port GPIOB
#define D6_V_DCF77_DEMOD_ADC1_IN16_Pin GPIO_PIN_1
#define D6_V_DCF77_DEMOD_ADC1_IN16_GPIO_Port GPIOB
#define D9_FRCD_HOLD_GPIO_I_Pin GPIO_PIN_8
#define D9_FRCD_HOLD_GPIO_I_GPIO_Port GPIOA
#define D1_UBLOX_USART1_TX_Pin GPIO_PIN_9
#define D1_UBLOX_USART1_TX_GPIO_Port GPIOA
#define D0_UBLOX_USART1_RX_Pin GPIO_PIN_10
#define D0_UBLOX_USART1_RX_GPIO_Port GPIOA
#define D10_PLL_LCKD_GPIO_I_Pin GPIO_PIN_11
#define D10_PLL_LCKD_GPIO_I_GPIO_Port GPIOA
#define D2_OCXO_LCKD_GPIO_O_Pin GPIO_PIN_12
#define D2_OCXO_LCKD_GPIO_O_GPIO_Port GPIOA
#define NoJ1J2_SYS_JTMS_SWDIO_Pin GPIO_PIN_13
#define NoJ1J2_SYS_JTMS_SWDIO_GPIO_Port GPIOA
#define NoJ1J2_SYS_JTCK_SWCLK_Pin GPIO_PIN_14
#define NoJ1J2_SYS_JTCK_SWCLK_GPIO_Port GPIOA
#define NoJ1J2_TERMINAL_USART2_RX_Pin GPIO_PIN_15
#define NoJ1J2_TERMINAL_USART2_RX_GPIO_Port GPIOA
#define D13_GPS_PPS_TIM2_CH2_Pin GPIO_PIN_3
#define D13_GPS_PPS_TIM2_CH2_GPIO_Port GPIOB
#define D12_HoRelay_GPIO_O_Pin GPIO_PIN_4
#define D12_HoRelay_GPIO_O_GPIO_Port GPIOB
#define D11_ONEWIRE_GPIO_IO_Pin GPIO_PIN_5
#define D11_ONEWIRE_GPIO_IO_GPIO_Port GPIOB
#define D5_I2C1_SCL_Pin GPIO_PIN_6
#define D5_I2C1_SCL_GPIO_Port GPIOB
#define D4_I2C1_SDA_Pin GPIO_PIN_7
#define D4_I2C1_SDA_GPIO_Port GPIOB
#define NoJ1J2_BOOT0_GPIO_I_Pin GPIO_PIN_3
#define NoJ1J2_BOOT0_GPIO_I_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* Turn off logging when not needed */
//#define LOGGING

/* Default: VCXO is displined by hardware PLL */
#define PLL_BY_SOFTWARE
#define DCF77_ENABLED

#define TIM2_IC_CH4_USE_DMA
//#define TIM2_IC_CH4_USE_INT

/* Onewire settings */
#define ONEWIRE_DEVICES_MAX							2

#define ONEWIRE_DS18B20_ADC_12B
//#define ONEWIRE_DS18B20_ADC_11B
//#define ONEWIRE_DS18B20_ADC_10B
//#define ONEWIRE_DS18B20_ADC_09B

#define ONEWIRE_DS18B20_ALARM_LO					40
#define ONEWIRE_DS18B20_ALARM_HI					50

/* ublox NEO settings */
#define UBLOX_MAX_CH								24

/* Software-PLL tuning parameters */
#define SW_PLL_TUNE_FAST							50000.0f
#define SW_PLL_TUNE_SLOW							200000.0f


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
