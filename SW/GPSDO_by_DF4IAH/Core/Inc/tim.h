/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim15;

/* USER CODE BEGIN Private defines */
#define TIM2_CH2_OUTSIDE_HW_DIVIDER						31
#define TIM2_CH2_DIVIDER								2

#define PRN_CORRELATION_SAMPLES_792MS774				512
#define PRN_CORRELATION_OVERSAMPLE						2
#define PRN_CORRELATION_SAMPLES_SECOND					((PRN_CORRELATION_BUF_SIZE / 2) / PRN_CORRELATION_OVERSAMPLE)
#define PRN_CORRELATION_SINGLE_BUF_SIZE					1250  // (77500 / (2 x 31)) = (1250 + 0.00) = (2 x 625 + 0.00) - needs a 31:1 divider in front of the input  D13_DCF77_PHASE_TIM2_CH2  pin
#define PRN_CORRELATION_DOUBLE_BUF_SIZE					(PRN_CORRELATION_SINGLE_BUF_SIZE << 1)

/* USER CODE END Private defines */

void MX_TIM2_Init(void);
void MX_TIM15_Init(void);

/* USER CODE BEGIN Prototypes */

#if 0
typedef struct tim2Ch4_TS_phase {
	uint32_t		ts_base;
	uint16_t		cnt;
	int32_t			ary[PRN_CORRELATION_BUF_SIZE];
} tim2Ch4_TS_phase_t;
#endif

typedef struct dcfTimeTelegr {
	//uint16_t											m		:1;		// b0:  Minute starts.

	/* PRN decode: sec 00 .. 09 --> 0b0 	*/
	/* PRN decode: sec 10 .. 14 --> unknown */

	uint8_t												rufBit 	:1;		// b1:  Calling staff in Braunschweig, b0: else.
	uint8_t												a1		:1;		// b1:  time change (MEZ <--> MESZ) to come in the next hour, b0: else.
	uint8_t												z		:2;		// b01: MEZ, b10: MESZ.
	uint8_t												a2		:1;		// b1:  additional second to be added within the next hour, b0: else.
	uint8_t												s		:1;		// b1:  Startbit
	uint8_t												_02		:2;		// filling the byte.

	uint8_t												mn_xM	:4;		// BCD code for the minute, right digit.
	uint8_t												mn_Mx	:3;		// BCD code for the minute, left  digit.
	uint8_t												mn_P1	:1;		// Even parity for the minute.

	uint8_t												hr_xH	:4;		// BCD code for the hour, right digit.
	uint8_t												hr_Hx	:2;		// BCD code for the hour, left  digit.
	uint8_t												hr_P2	:1;		// Even parity for the hour.
	uint8_t												_11		:1;		// filling the byte.

	uint8_t												dy_xD	:4;		// BCD code for the day of the month, right digit.
	uint8_t												dy_Dx	:2;		// BCD code for the day of the month, left  digit.
	uint8_t												_22		:2;		// filling the byte.

	uint8_t												wd_xD	:3;		// BCD code for the day of the week, 1=monday .. 7=sunday (ISO 8601).
	uint8_t												_35		:5;		// filling the byte.

	uint8_t												mo_xM	:4;		// BCD code for the month, right digit.
	uint8_t												mo_Mx	:1;		// BCD code for the month, left  digit.
	uint8_t												_43		:3;		// filling the byte.


	uint8_t												yr_xY	:4;		// BCD code for the last two digits of the year, right digit.
	uint8_t												yr_Yx	:4;		// BCD code for the last two digits of the year, left  digit.

	uint8_t												hr_P3	:1;		// Even parity for the date block.
	uint8_t												prn_59	:1;		// b0:  Minute ends.
	uint8_t												_57		:7;		// filling the byte.

} dcfTimeTelegr_t;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
