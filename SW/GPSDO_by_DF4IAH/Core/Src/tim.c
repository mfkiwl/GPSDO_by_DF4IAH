/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

uint8_t  tim2Ch2_idx				= 0U;
uint32_t tim2Ch2_ts[10]				= { 0 };

uint32_t timTicksEvt				= 0UL;
int32_t timTicksDiff				= 0L;

float tim2Ch2_ppm					= 0.0f;

int32_t timTicksSumDev				= 0L;


#if 0
uint32_t tim_dma_ch2_buf[2] 		= { 0 };
const uint32_t TIM_DMA_CH2_Buf_Len 	= sizeof(tim_dma_ch2_buf) / sizeof(uint32_t);
#endif

#if 0
uint32_t tim2Ch4_ts				= 0UL;
uint32_t tim_dma_ch4_buf[2] 		= { 0 };
const uint32_t TIM_DMA_CH4_Buf_Len 	= sizeof(tim_dma_ch4_buf) / sizeof(uint32_t);
#endif

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA3     ------> TIM2_CH4
    PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = A2_DCF77_CAR_TIM2_CH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(A2_DCF77_CAR_TIM2_CH4_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = D13_GPS_PPS_TIM2_CH2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(D13_GPS_PPS_TIM2_CH2_GPIO_Port, &GPIO_InitStruct);

    /* TIM2 DMA Init */
    /* TIM2_CH2_CH4 Init */
    hdma_tim2_ch2_ch4.Instance = DMA1_Channel7;
    hdma_tim2_ch2_ch4.Init.Request = DMA_REQUEST_4;
    hdma_tim2_ch2_ch4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_tim2_ch2_ch4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim2_ch2_ch4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim2_ch2_ch4.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim2_ch2_ch4.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim2_ch2_ch4.Init.Mode = DMA_CIRCULAR;
    hdma_tim2_ch2_ch4.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_tim2_ch2_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2_ch4);
    __HAL_LINKDMA(tim_icHandle,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch2_ch4);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PA3     ------> TIM2_CH4
    PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(A2_DCF77_CAR_TIM2_CH4_GPIO_Port, A2_DCF77_CAR_TIM2_CH4_Pin);

    HAL_GPIO_DeInit(D13_GPS_PPS_TIM2_CH2_GPIO_Port, D13_GPS_PPS_TIM2_CH2_Pin);

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_icHandle->hdma[TIM_DMA_ID_CC4]);

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		/* GPS 1PPS (1 kHz) pulse entered */
		uint32_t tim2_ch2_ts_now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

#if !defined(DISCIPLINED_BY_SOFTWARE)
		/* First pulse of a second, only */
		if (tim2_ch2_ts_now < 60000UL) {
#else
		/* 1 PPS mode */
		{
#endif
			int32_t diff = tim2_ch2_ts_now - tim2Ch2_ts[tim2Ch2_idx];

			++timTicksEvt;

			/* Clamp below +/-5 ppm */
			if ((-3000 < diff) && (diff < +3000)) {
				/* Store accumulated difference */
				if (timTicksEvt > 12) {
					timTicksDiff += diff;
				}
			} else {
				diff = 0;
			}

			/* Calculate PPMs */
			tim2Ch2_ppm = diff / 600.0f;

			/* Write back TimeStamp to 10 sec circle-buffer */
			tim2Ch2_ts[tim2Ch2_idx++] = tim2_ch2_ts_now;
			tim2Ch2_idx %= 10;
		}
	}
}


void tim_start(void)
{
	if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
}

void tim_capture_ch2(void)
{
#if 0
  if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2)) {
	  Error_Handler();
  }
#endif
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
