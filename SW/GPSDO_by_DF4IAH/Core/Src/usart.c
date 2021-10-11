/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>

extern void delay(uint32_t ms);

static	uint8_t ublox_Response[256] 	= { 0 };
__IO ITStatus gUart1TxReady 			= RESET;
__IO ITStatus gUart1RxReady 			= RESET;



/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  gUart1TxReady = SET;
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  gUart1RxReady = SET;
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	/**
	 *	#define  HAL_UART_ERROR_NONE             (0x00000000U)    !< No error
	 *	#define  HAL_UART_ERROR_PE               (0x00000001U)    !< Parity error
	 *	#define  HAL_UART_ERROR_NE               (0x00000002U)    !< Noise error
	 *	#define  HAL_UART_ERROR_FE               (0x00000004U)    !< Frame error
	 *	#define  HAL_UART_ERROR_ORE              (0x00000008U)    !< Overrun error
	 *	#define  HAL_UART_ERROR_DMA              (0x00000010U)    !< DMA transfer error
	 *	#define  HAL_UART_ERROR_RTO              (0x00000020U)    !< Receiver Timeout error
	 */
	__IO uint32_t err = UartHandle->ErrorCode;

	if (UartHandle == &huart1) {
		if (err & HAL_UART_ERROR_RTO) {
			/* Stop transfer */
			gUart1RxReady = SET;
		}
		else if (
				err & HAL_UART_ERROR_PE ||
				err & HAL_UART_ERROR_NE ||
				err & HAL_UART_ERROR_FE) {
			/* Stop transfer */
			gUart1RxReady = SET;
		}
		else if (err & HAL_UART_ERROR_ORE) {
			/* Stop transfer */
			gUart1RxReady = SET;

			/* ignore */
			//gUart1RxReady = gUart1RxReady;
		}
		else {
			Error_Handler();
		}
	}
	else if (UartHandle == &huart2) {
		Error_Handler();
	}
}


/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = D1_UBLOX_USART1_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(D1_UBLOX_USART1_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = D0_UBLOX_USART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(D0_UBLOX_USART1_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = NoA7_TERMINAL_USART2_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(NoA7_TERMINAL_USART2_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NoJ1J2_TERMINAL_USART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(NoJ1J2_TERMINAL_USART2_RX_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, D1_UBLOX_USART1_TX_Pin|D0_UBLOX_USART1_RX_Pin);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, NoA7_TERMINAL_USART2_TX_Pin|NoJ1J2_TERMINAL_USART2_RX_Pin);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


/* EXTRA INITS */

void MX_USART1_UART_Init_38400baud(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}


/* UBLOX COMMUNICATION */

void calcChecksumRFC1145(uint8_t* ubxMsg, uint8_t ubxSize)
{
	uint8_t ck_a = 0U, ck_b = 0U;

	/* Forward to checking region */
	ubxMsg += 2;

	/* Calc checksums */
	for (int i = ubxSize - 4; i; --i) {
		ck_a = 0xffU & (ck_a + *(ubxMsg++));
		ck_b = 0xffU & (ck_b + ck_a);
	}

	/* Fill in checksums */
	*(ubxMsg++) = ck_a;
	*ubxMsg 	= ck_b;
}


void ubloxUartSpeedFast(void)
{
	const uint32_t baudrate = 38400UL;

	uint8_t cfg_Port1_Req[]		= {
			0xb5,	0x62,
			0x06,	0x00,
			0x01,	0x00,
			0x01,
			0xff,	0xff
	};
	calcChecksumRFC1145(cfg_Port1_Req, sizeof(cfg_Port1_Req));

	uint8_t cfg_Port1_Set[28] 	= { 0 };

	/* Preparation for little endian */
	uint8_t buf[4];
	buf[0] = (baudrate & 0x000000ffUL)      ;
	buf[1] = (baudrate & 0x0000ff00UL) >>  8;
	buf[2] = (baudrate & 0x00ff0000UL) >> 16;
	buf[3] = (baudrate & 0xff000000UL) >> 24;

	int cnt = 3;
	while (cnt) {
#if defined(LOGGING)
		{
			uint8_t msg[] = "\r\n*** CFG-PORT: TX --> RX --> ";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif

		/* Send CFG-PORT request */
		gUart1TxReady = RESET;
		HAL_UART_Transmit_IT(&huart1, cfg_Port1_Req, sizeof(cfg_Port1_Req));
		while (gUart1TxReady != SET) {
		}

		gUart1RxReady = RESET;
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_EnableReceiverTimeout(&huart1);
		HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
		int i = 11;
		while (i && (gUart1RxReady != SET)) {
			HAL_Delay(100);
			--i;
		}

		if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
				(ublox_Response[2] == 0x06) && (ublox_Response[3] == 0x00) &&
				(ublox_Response[4] == 0x14) && (ublox_Response[5] == 0x00)) {

			for (int i = 0; i < sizeof(cfg_Port1_Set); ++i) {
				cfg_Port1_Set[i] = ublox_Response[i];
			}

			/* Set new baudrate */
			cfg_Port1_Set[6 +  8] = buf[0];
			cfg_Port1_Set[6 +  9] = buf[1];
			cfg_Port1_Set[6 + 10] = buf[2];
			cfg_Port1_Set[6 + 11] = buf[3];

			/* Recalculate checksum */
			calcChecksumRFC1145(cfg_Port1_Set, sizeof(cfg_Port1_Set));

			/* Send CFG-PORT for COM1 */
			gUart1TxReady = RESET;
			HAL_UART_Transmit_IT(&huart1, cfg_Port1_Set, sizeof(cfg_Port1_Set));
			while (gUart1TxReady != SET) {
			}
			HAL_UART_AbortTransmit_IT(&huart1);

			/* Change baudrate */
			HAL_UART_DeInit(&huart1);
			MX_USART1_UART_Init_38400baud();

			/* Receive CFG-PORT status */
			gUart1RxReady = RESET;
			HAL_UART_AbortReceive_IT(&huart1);
			HAL_UART_EnableReceiverTimeout(&huart1);
			HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
			while (gUart1RxReady != SET) {
			}

			/* Check for CFG-TP5 ACK-ACK */
			if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
					(ublox_Response[2] == 0x05) && (ublox_Response[3] == 0x01) &&
					(ublox_Response[4] == 0x02) && (ublox_Response[5] == 0x00) &&
					(ublox_Response[6] == 0x06) && (ublox_Response[7] == 0x00)) {
				/* ACK-ACK for CFG-PORT received */
#if defined(LOGGING)
				{
					uint8_t msg[] = "ACK-ACK received --> done.\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
					HAL_Delay(100);
				}
#endif
			}
			else {
#if defined(LOGGING)
				{
					uint8_t msg[] = "no ACK-ACK received --> silently drop and accept.\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
					HAL_Delay(100);
				}
#endif
			}
			return;
		}
		else {
			/* Failure in transmissions */
			HAL_Delay(200);
			--cnt;
		}
	}  // while (cnt)

	/* Change baudrate */
	HAL_UART_DeInit(&huart1);
	MX_USART1_UART_Init_38400baud();

#if defined(LOGGING)
	{
		uint8_t msg[] = "no result, already fast? Turning local bitrate up.\r\n";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		HAL_Delay(100);
	}
#endif
}

void ubloxFlush(void)
{
	/* Flush the queue first */
	HAL_UART_EnableReceiverTimeout(&huart1);

	gUart1RxReady = RESET;
	HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));

	int cnt = 12;
	while (cnt && (gUart1RxReady != SET)) {
		HAL_Delay(100);
		--cnt;
	}

	HAL_UART_AbortReceive_IT(&huart1);
}

uint8_t ubloxSetFrequency(uint16_t frequency)
{
	uint8_t cfg_tp5_Set[40] 	= { 0 };
	uint8_t buf[4];

	/* Preparation for little endian */
	buf[0] = (frequency & 0x000000ffUL)      ;
	buf[1] = (frequency & 0x0000ff00UL) >>  8;
	buf[2] = (frequency & 0x00ff0000UL) >> 16;
	buf[3] = (frequency & 0xff000000UL) >> 24;

	/* Generate the configuration string for the TimePulse with given frequency */
	uint8_t cfg_tp5_Req[] 		= {
			0xb5,	0x62,
			0x06,	0x31,
			0x00,	0x00,
			0xff,	0xff
	};
	calcChecksumRFC1145(cfg_tp5_Req, sizeof(cfg_tp5_Req));

	/* First get current CFG-TP5 settings for channel TIMEPULSE */
	uint8_t tryCtr = 3;
	while (tryCtr) {
#if defined(LOGGING)
		{
			uint8_t msg[] = "\r\n*** ubloxSetFrequency() --> requesting TimePulse Parameters --> ";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
			HAL_Delay(100);
		}
#endif

		/* Prepare for answer */
		gUart1RxReady = RESET;
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_EnableReceiverTimeout(&huart1);
		HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));

		/* Send CFG-TP5 request */
		gUart1TxReady = RESET;
		//HAL_UART_AbortTransmit_IT(&huart1);
		HAL_UART_Transmit_IT(&huart1, cfg_tp5_Req, sizeof(cfg_tp5_Req));
		while (gUart1TxReady != SET) {
		}

		/* Wait for the response */
		int i = 11;
		while (i && (gUart1RxReady != SET)) {
			HAL_Delay(100);
			--i;
		}

#if defined(LOGGING)
		{
			uint8_t msg[] = "TX --> RX --> check ReqAnswer --> ";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
			HAL_Delay(100);
		}
#endif

		/* Response to our request? */
		if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
				(ublox_Response[2] == 0x06) && (ublox_Response[3] == 0x31)) {
			/* Copy template */
			for (int i = 0; i < sizeof(cfg_tp5_Set); ++i) {
				cfg_tp5_Set[i] = ublox_Response[i];
			}

			/* Fill in Period Time for when not Locked */
			cfg_tp5_Set[6 +  8] = buf[0];
			cfg_tp5_Set[6 +  9] = buf[1];
			cfg_tp5_Set[6 + 10] = buf[2];
			cfg_tp5_Set[6 + 11] = buf[3];

			/* Fill in Period Time for when Locked */
			cfg_tp5_Set[6 + 12] = buf[0];
			cfg_tp5_Set[6 + 13] = buf[1];
			cfg_tp5_Set[6 + 14] = buf[2];
			cfg_tp5_Set[6 + 15] = buf[3];

			/* Fill in 50% ratio when not Locked */
			cfg_tp5_Set[6 + 16] = 0x00;
			cfg_tp5_Set[6 + 17] = 0x00;
			cfg_tp5_Set[6 + 18] = 0x00;
			cfg_tp5_Set[6 + 19] = 0x80;

			/* Fill in 50% ratio when Locked */
			cfg_tp5_Set[6 + 20] = 0x00;
			cfg_tp5_Set[6 + 21] = 0x00;
			cfg_tp5_Set[6 + 22] = 0x00;
			cfg_tp5_Set[6 + 23] = 0x80;

			/* bit 0: 1 = output active */
			/* bit 1: 1 = TimePulse sync to GPS */
			/* bit 2: 1 = two different sets for GPS locked and non-locked TimePulses are being used */
			/* bit 3: 1 = use fields as frequencies and not period times */
			/* bit 4: 0 = use pulse ratios instead of duration in microseconds */
			/* bit 5: 0 = frequencies not multiple of 1 sec so bit 'alignToTow' has to be cleared */
			/* bit 6: 1 = positive polarity */
			/* bit 7: 1 = timegrid is GPS (not UTC) */
			cfg_tp5_Set[6 + 28] = 0b11001111;

			/* Recalculate checksum */
			calcChecksumRFC1145(cfg_tp5_Set, sizeof(cfg_tp5_Set));

			/* Send TimePule Parameters for new frequency */
			gUart1TxReady = RESET;
			HAL_UART_AbortTransmit_IT(&huart1);
			HAL_UART_Transmit_IT(&huart1, cfg_tp5_Set, sizeof(cfg_tp5_Set));
			while (gUart1TxReady != SET) {
			}
			HAL_UART_AbortTransmit_IT(&huart1);

			/* Receive CFG-TP5 status */
			gUart1RxReady = RESET;
			HAL_UART_EnableReceiverTimeout(&huart1);
			HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
			while (gUart1RxReady != SET) {
			}
			HAL_UART_AbortReceive_IT(&huart1);

			/* Check for CFG-TP5 ACK-ACK */
			if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
					(ublox_Response[2] == 0x05) && (ublox_Response[3] == 0x01) &&
					(ublox_Response[4] == 0x02) && (ublox_Response[5] == 0x00) &&
					(ublox_Response[6] == 0x06) && (ublox_Response[7] == 0x31)) {
				/* ACK-ACK for CFG-TP5 received */
#if defined(LOGGING)
				{
					uint8_t msg[] = "ACK-ACK received --> done.\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
					HAL_Delay(100);
				}
#endif
				return 0;
			}
		}

#if defined(LOGGING)
		{
			uint8_t msg[] = "not relating ACK-ACK received, try again ...\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
			HAL_Delay(100);
		}
#endif

		/* Next round to come ... */
		--tryCtr;
		HAL_Delay(1500);
	}

	return 1;
}

void ubloxMsgsTurnOff(void)
{
	uint8_t msg[] = "$PUBX,40,RMC,0,0,0,0,0,0*47\r\n" \
					"$PUBX,40,VTG,0,0,0,0,0,0*5E\r\n" \
					"$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n" \
					"$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n" \
					"$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n" \
					"$PUBX,40,GSV,0,0,0,0,0,0*59\r\n";

	/* Turn off these messages */
	gUart1TxReady = RESET;
	HAL_UART_Transmit_IT(&huart1, msg, sizeof(msg));
	while (gUart1TxReady != SET) {
	}
	HAL_UART_AbortTransmit_IT(&huart1);
}

void ublox_NavDop_get(UbloxNavDop_t* dop)
{
	uint8_t nav_Dop_Req[] 		= {
			0xb5,	0x62,
			0x01,	0x04,
			0x00,	0x00,
			0xff,	0xff
	};
	calcChecksumRFC1145(nav_Dop_Req, sizeof(nav_Dop_Req));

#if defined(LOGGING)
	{
		uint8_t msg[] = "\r\n\t*** NAV-DOP: TX --> RX --> ";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	}
#endif

	/* Re-init the device */
	HAL_UART_DeInit(&huart1);
	MX_USART1_UART_Init_38400baud();

	/* Send NAV-DOP request */
	gUart1TxReady = RESET;
	HAL_UART_Transmit_IT(&huart1, nav_Dop_Req, sizeof(nav_Dop_Req));
	while (gUart1TxReady != SET) {
	}
	//HAL_UART_AbortTransmit_IT(&huart1);

	gUart1RxReady = RESET;
	HAL_UART_EnableReceiverTimeout(&huart1);
	HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
	while (gUart1RxReady != SET) {
	}
	//HAL_UART_AbortReceive_IT(&huart1);

	if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
			(ublox_Response[2] == 0x01) && (ublox_Response[3] == 0x04) &&
			(ublox_Response[4] == 0x12) && (ublox_Response[5] == 0x00)) {
		dop->iTOW		= ublox_Response[6 +  0] | (ublox_Response[6 +  1] << 8) | (ublox_Response[6 +  2] << 16) | (ublox_Response[6 +  3] << 24);
		dop->gDOP		= ublox_Response[6 +  4] | (ublox_Response[6 +  5] << 8);
		dop->pDOP		= ublox_Response[6 +  6] | (ublox_Response[6 +  7] << 8);
		dop->tDOP		= ublox_Response[6 +  8] | (ublox_Response[6 +  9] << 8);
		dop->vDOP		= ublox_Response[6 + 10] | (ublox_Response[6 + 11] << 8);
		dop->hDOP		= ublox_Response[6 + 12] | (ublox_Response[6 + 13] << 8);
		dop->nDOP		= ublox_Response[6 + 14] | (ublox_Response[6 + 15] << 8);
		dop->eDOP		= ublox_Response[6 + 16] | (ublox_Response[6 + 17] << 8);

#if defined(LOGGING)
		{
			uint8_t msg[] = "data OK:\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}

		{
			uint8_t msg[64];
			int len;

			len = snprintf(((char*) msg), sizeof(msg), "\t  * GPS Millisec Time of Week: %ld\r\n", dop->iTOW);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Geometric  DOP: %d.%02d\r\n", (dop->gDOP / 100), (dop->gDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Position   DOP: %d.%02d\r\n", (dop->pDOP / 100), (dop->pDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Time       DOP: %d.%02d\r\n", (dop->tDOP / 100), (dop->tDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Vertical   DOP: %d.%02d\r\n", (dop->vDOP / 100), (dop->vDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Horizontal DOP: %d.%02d\r\n", (dop->hDOP / 100), (dop->hDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Northing   DOP: %d.%02d\r\n", (dop->nDOP / 100), (dop->nDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t  * Easting    DOP: %d.%02d\r\n", (dop->eDOP / 100), (dop->eDOP % 100));
			HAL_UART_Transmit(&huart2, msg, len, 25);
		}

		{
			uint8_t msg[] = "\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
	else {
		dop->iTOW		= 0UL;
		dop->gDOP		= 0U;
		dop->pDOP		= 0U;
		dop->tDOP		= 0U;
		dop->vDOP		= 0U;
		dop->hDOP		= 0U;
		dop->nDOP		= 0U;
		dop->eDOP		= 0U;

#if defined(LOGGING)
		{
			uint8_t msg[] = "data FAILED!\r\n\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
}

void ublox_NavClock_get(UbloxNavClock_t* ubloxNavClock)
{
	uint8_t nav_Clock_Req[] 		= {
			0xb5,	0x62,
			0x01,	0x22,
			0x00,	0x00,
			0xff,	0xff
	};
	calcChecksumRFC1145(nav_Clock_Req, sizeof(nav_Clock_Req));

#if defined(LOGGING)
	{
		uint8_t msg[] = "\r\n\t\t*** NAV-CLOCK: TX --> RX --> ";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	}
#endif

	/* Re-init the device */
	HAL_UART_DeInit(&huart1);
	MX_USART1_UART_Init_38400baud();

	/* Send NAV-CLOCK request */
	gUart1TxReady = RESET;
	HAL_UART_Transmit_IT(&huart1, nav_Clock_Req, sizeof(nav_Clock_Req));
	while (gUart1TxReady != SET) {
	}
	//HAL_UART_AbortTransmit_IT(&huart1);

	gUart1RxReady = RESET;
	HAL_UART_EnableReceiverTimeout(&huart1);
	HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
	while (gUart1RxReady != SET) {
	}
	//HAL_UART_AbortReceive_IT(&huart1);

	if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
			(ublox_Response[2] == 0x01) && (ublox_Response[3] == 0x22) &&
			(ublox_Response[4] == 0x14) && (ublox_Response[5] == 0x00)) {
		ubloxNavClock->iTOW	=            ublox_Response[6 +  0] | (ublox_Response[6 +  1] << 8) | (ublox_Response[6 +  2] << 16) | (ublox_Response[6 +  3] << 24);
		ubloxNavClock->clkB	= (int32_t) (ublox_Response[6 +  4] | (ublox_Response[6 +  5] << 8) | (ublox_Response[6 +  6] << 16) | (ublox_Response[6 +  7] << 24));
		ubloxNavClock->clkD	= (int32_t) (ublox_Response[6 +  8] | (ublox_Response[6 +  9] << 8) | (ublox_Response[6 + 10] << 16) | (ublox_Response[6 + 11] << 24));
		ubloxNavClock->tAcc	=            ublox_Response[6 + 12] | (ublox_Response[6 + 13] << 8) | (ublox_Response[6 + 14] << 16) | (ublox_Response[6 + 15] << 24);
		ubloxNavClock->fAcc	=            ublox_Response[6 + 16] | (ublox_Response[6 + 17] << 8) | (ublox_Response[6 + 18] << 16) | (ublox_Response[6 + 19] << 24);

#if defined(LOGGING)
		{
			uint8_t msg[] = "data OK:\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}

		{
			uint8_t msg[64];
			int len;

			len = snprintf(((char*) msg), sizeof(msg), "\t\t*** GPS Millisec Time of Week: %ld\r\n", 	ubloxNavClock->iTOW);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t  * Clock bias    : %+ld ns\r\n",   		ubloxNavClock->clkB);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t  * Clock drift   : %+ld ns/s\r\n", 		ubloxNavClock->clkD);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t  * Time Acc Est. : %lu ns\r\n", 			ubloxNavClock->tAcc);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t  * Freq Acc Est. : %lu ps/s\r\n", 			ubloxNavClock->fAcc);
			HAL_UART_Transmit(&huart2, msg, len, 25);
		}

		{
			uint8_t msg[] = "\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
	else {
		ubloxNavClock->iTOW	= 0UL;
		ubloxNavClock->clkB	= 0UL;
		ubloxNavClock->clkD	= 0UL;
		ubloxNavClock->tAcc	= 0UL;
		ubloxNavClock->fAcc	= 0UL;

#if defined(LOGGING)
		{
			uint8_t msg[] = "data FAILED!\r\n\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
}

void ublox_NavSvinfo_get(UbloxNavSvinfo_t* ubloxNavSvinfo)
{
	uint8_t nav_Svinfo_Req[] 		= {
			0xb5,	0x62,
			0x01,	0x30,
			0x00,	0x00,
			0xff,	0xff
	};
	calcChecksumRFC1145(nav_Svinfo_Req, sizeof(nav_Svinfo_Req));

#if defined(LOGGING)
	{
		uint8_t msg[] = "\r\n\t\t\t*** NAV-SVINFO: TX --> RX --> ";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	}
#endif

	/* Re-init the device */
	HAL_UART_DeInit(&huart1);
	MX_USART1_UART_Init_38400baud();

	/* Send NAV-SVINFO request */
	gUart1TxReady = RESET;
	HAL_UART_AbortTransmit_IT(&huart1);
	HAL_UART_Transmit_IT(&huart1, nav_Svinfo_Req, sizeof(nav_Svinfo_Req));
	while (gUart1TxReady != SET) {
	}

	/* Wait for the response */
	gUart1RxReady = RESET;
	HAL_UART_AbortReceive_IT(&huart1);
	HAL_UART_EnableReceiverTimeout(&huart1);
	HAL_UART_Receive_IT(&huart1, ublox_Response, sizeof(ublox_Response));
	while (gUart1RxReady != SET) {
	}

	/* Clear fields */
	{
		uint8_t* ptr = (uint8_t*) ubloxNavSvinfo;
		for (int cnt = sizeof(*ubloxNavSvinfo); cnt; --cnt) {
			*(ptr++) = 0U;
		}
	}

	if (	(ublox_Response[0] == 0xb5) && (ublox_Response[1] == 0x62) &&
			(ublox_Response[2] == 0x01) && (ublox_Response[3] == 0x30)) {
		ubloxNavSvinfo->iTOW		= ublox_Response[6 +  0] | (ublox_Response[6 +  1] << 8) | (ublox_Response[6 +  2] << 16) | (ublox_Response[6 +  3] << 24);
		ubloxNavSvinfo->numCh		= ublox_Response[6 +  4];
		ubloxNavSvinfo->globalFlags	= ublox_Response[6 +  5];
		ubloxNavSvinfo->reserved2	= ublox_Response[6 +  6] | (ublox_Response[6 +  7] << 8);

		if (ubloxNavSvinfo->numCh > UBLOX_MAX_CH) {
			ubloxNavSvinfo->numCh = (uint8_t) UBLOX_MAX_CH;
		}

		/* Read in each space vehicle */
		for (int iChn = 0; iChn < ubloxNavSvinfo->numCh; iChn++) {
			ubloxNavSvinfo->chn[iChn]		= ublox_Response[6 +  8 + 12 * iChn];
			ubloxNavSvinfo->svid[iChn]		= ublox_Response[6 +  9 + 12 * iChn];
			ubloxNavSvinfo->flags[iChn]		= ublox_Response[6 + 10 + 12 * iChn];
			ubloxNavSvinfo->quality[iChn]	= ublox_Response[6 + 11 + 12 * iChn];
			ubloxNavSvinfo->cno[iChn]		= ublox_Response[6 + 12 + 12 * iChn];
			ubloxNavSvinfo->elev[iChn]		= (int8_t)  (ublox_Response[6 + 13 + 12 * iChn]);
			ubloxNavSvinfo->azim[iChn]		= (int16_t) ((uint16_t)ublox_Response[6 + 14 + 12 * iChn] | ((uint16_t)ublox_Response[6 + 15 + 12 * iChn] << 8));
			ubloxNavSvinfo->prRes[iChn]		= (int16_t) ((uint32_t)ublox_Response[6 + 16 + 12 * iChn] | ((uint32_t)ublox_Response[6 + 17 + 12 * iChn] << 8)  | ((uint32_t)ublox_Response[6 + 18 + 12 * iChn] << 16)  | ((uint32_t)ublox_Response[6 + 19 + 12 * iChn] << 24));
		}

#if defined(LOGGING)
		{
			uint8_t msg[] = "data OK:\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}

		{
			uint8_t msg[64];
			int len;

			len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * GPS Millisec Time of Week: %ld\r\n", 	ubloxNavSvinfo->iTOW);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Number of Chn : %u\r\n",   				ubloxNavSvinfo->numCh);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Global flags  : 0x%02x\r\n", 			ubloxNavSvinfo->globalFlags);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * reserved2     : %u\r\n",				ubloxNavSvinfo->reserved2);
			HAL_UART_Transmit(&huart2, msg, len, 25);

			for (int iChn = 0; iChn < ubloxNavSvinfo->numCh; iChn++) {
				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  *\r\n");
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d chn    : %u\r\n", iChn, 		ubloxNavSvinfo->chn[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d svid   : %u\r\n", iChn, 		ubloxNavSvinfo->svid[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d flags  : 0x%02x\r\n", iChn,	ubloxNavSvinfo->flags[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d quality: 0x%02x\r\n", iChn,	ubloxNavSvinfo->quality[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d Car/Nse: %u dbHz\r\n", iChn,	ubloxNavSvinfo->cno[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d Elev.  : %d deg\r\n", iChn, 	ubloxNavSvinfo->elev[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d Azimuth: %d deg\r\n", iChn, 	ubloxNavSvinfo->azim[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);

				len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * Ch%02d prRes  : %ld cm\r\n", iChn, 	ubloxNavSvinfo->prRes[iChn]);
				HAL_UART_Transmit(&huart2, msg, len, 25);
			}
		}

		{
			uint8_t msg[] = "\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
	else {
#if defined(LOGGING)
		{
			uint8_t msg[] = "data FAILED!\r\n\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		}
#endif
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
