/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */
#define USART_UBLOX_RESP_BF_NAV_POSLLH			0x00000001
#define USART_UBLOX_RESP_BF_NAV_CLOCK			0x00000002
#define USART_UBLOX_RESP_BF_NAV_DOP				0x00000004
#define USART_UBLOX_RESP_BF_NAV_SVINFO			0x00000008


typedef struct UbloxNavPosllh {
	uint32_t	iTOW;
	int32_t		lon;
	int32_t		lat;
	int32_t		height;
	int32_t		hMSL;
	uint32_t	hAcc;
	uint32_t	vAcc;
} UbloxNavPosllh_t;

typedef struct UbloxNavClock {
	uint32_t	iTOW;
	int32_t		clkB;
	int32_t		clkD;
	uint32_t	tAcc;
	uint32_t	fAcc;
} UbloxNavClock_t;

typedef struct UbloxNavDop {
	uint32_t	iTOW;
	uint16_t	gDOP;
	uint16_t	pDOP;
	uint16_t	tDOP;
	uint16_t	vDOP;
	uint16_t	hDOP;
	uint16_t	nDOP;
	uint16_t	eDOP;
} UbloxNavDop_t;

typedef struct UbloxNavSvinfo {
	uint32_t	iTOW;
	uint8_t		numCh;
	uint8_t		globalFlags;
	uint16_t	reserved2;

	uint8_t		chn[UBLOX_MAX_CH];
	uint8_t		svid[UBLOX_MAX_CH];
	uint8_t		flags[UBLOX_MAX_CH];
	uint8_t		quality[UBLOX_MAX_CH];
	uint8_t		cno[UBLOX_MAX_CH];
	int8_t		elev[UBLOX_MAX_CH];
	int16_t		azim[UBLOX_MAX_CH];
	int32_t		prRes[UBLOX_MAX_CH];
} UbloxNavSvinfo_t;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
