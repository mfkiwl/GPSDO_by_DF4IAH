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
__IO ITStatus Uart1Ready = RESET;


extern void delay(uint32_t ms);


/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  Uart1Ready = SET;
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
  Uart1Ready = SET;
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

	switch (err)
	{
	default:
		if (UartHandle == &huart1) {
			Uart1Ready = SET;
		}
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
    GPIO_InitStruct.Pin = D1_UBLOX_USART1_TX_Pin|D4_UBLOX_USART1_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(NoA7_TERMINAL_USART2_TX_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = NoJ1J2_TERMINAL_USART2_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
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
    HAL_GPIO_DeInit(GPIOA, D1_UBLOX_USART1_TX_Pin|D4_UBLOX_USART1_RX_Pin);

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

#define LOGGING
uint8_t ubloxSetFrequency(uint16_t frequency)
{
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

	uint8_t cfg_tp5_Set[40] 	= { 0 };
	uint8_t cfg_Response[256] 	= { 0 };

	/* First get current CFG-TP5 settings for channel TIMEPULSE */
	uint8_t tryCtr = 3;
	while (tryCtr) {
#if defined(LOGGING)
		{
			uint8_t msg[] = "\r\n*** ubloxSetFrequency() --> requesting TimePulse Parameters --> ";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg), 25);
			HAL_Delay(100);
		}
#endif

		/* Flush the queue first */
		Uart1Ready = RESET;
		HAL_UART_EnableReceiverTimeout(&huart1);
		HAL_UART_Receive_IT(&huart1, cfg_Response, sizeof(cfg_Response));
		while (Uart1Ready != SET) {
		}
		HAL_UART_AbortReceive_IT(&huart1);

		/* Send CFG-TP5 request */
		Uart1Ready = RESET;
		HAL_UART_Transmit_IT(&huart1, cfg_tp5_Req, sizeof(cfg_tp5_Req));
		while (Uart1Ready != SET) {
		}
		HAL_UART_AbortTransmit_IT(&huart1);

		Uart1Ready = RESET;
		HAL_UART_EnableReceiverTimeout(&huart1);
		HAL_UART_Receive_IT(&huart1, cfg_Response, sizeof(cfg_Response));
		while (Uart1Ready != SET) {
		}
		HAL_UART_AbortReceive_IT(&huart1);

#if defined(LOGGING)
		{
			uint8_t msg[] = "TX --> RX --> check ReqAnswer --> ";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg), 25);
			HAL_Delay(100);
		}
#endif

		/* Response to our request? */
		if ((cfg_Response[0] == 0xb5) && (cfg_Response[1] == 0x62) && (cfg_Response[2] == 0x06) && (cfg_Response[3] == 0x31)) {
			/* Copy template */
			for (int i = 0; i < sizeof(cfg_tp5_Set); ++i) {
				cfg_tp5_Set[i] = cfg_Response[i];
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
			Uart1Ready = RESET;
			HAL_UART_AbortTransmit_IT(&huart1);
			HAL_UART_Transmit_IT(&huart1, cfg_tp5_Set, sizeof(cfg_tp5_Set));
			while (Uart1Ready != SET) {
			}
			HAL_UART_AbortTransmit_IT(&huart1);

			/* Receive CFG-TP5 status */
			Uart1Ready = RESET;
			HAL_UART_EnableReceiverTimeout(&huart1);
			HAL_UART_Receive_IT(&huart1, cfg_Response, sizeof(cfg_Response));
			while (Uart1Ready != SET) {
			}
			HAL_UART_AbortReceive_IT(&huart1);

			/* Check for CFG-TP5 ACK-ACK */
			if (	(cfg_Response[0] == 0xb5) && (cfg_Response[1] == 0x62) &&
					(cfg_Response[2] == 0x05) && (cfg_Response[3] == 0x01) &&
					(cfg_Response[4] == 0x02) && (cfg_Response[5] == 0x00) &&
					(cfg_Response[6] == 0x06) && (cfg_Response[7] == 0x31)) {
				/* ACK-ACK for CFG-TP5 received */
#if defined(LOGGING)
				{
					uint8_t msg[] = "ACK-ACK received --> done.\r\n";
					HAL_UART_Transmit(&huart2, msg, sizeof(msg), 25);
					HAL_Delay(100);
				}
#endif
				return 0;
			}
		}

#if defined(LOGGING)
		{
			uint8_t msg[] = "not relating ACK-ACK received, try again ...\r\n";
			HAL_UART_Transmit(&huart2, msg, sizeof(msg), 25);
			HAL_Delay(100);
		}
#endif

		/* Next round to come ... */
		--tryCtr;
		HAL_Delay(1500);
	}

	return 1;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
