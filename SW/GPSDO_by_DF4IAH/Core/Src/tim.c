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

/* Timestamps from Callback functions */

/* GPS 1PPS monitoring */

__IO uint32_t			giTim2Ch2_TS					= 0UL;
__IO uint8_t			giTim2Ch2_TS_ary_idx			= 0U;
__IO uint32_t			giTim2Ch2_TS_ary[10]			= { 0 };

__IO uint32_t			giTim2Ch2_TicksEvt				= 0UL;
__IO int32_t			giTim2Ch2_TicksDiff				= 0L;
__IO int32_t			giTim2Ch2_TicksSumDev			= 0L;
__IO float 				giTim2Ch2_ppm					= 0.0f;


/* DCF77 RF signal monitoring */

__IO uint32_t			giTim2Ch4_TS					= 0UL;

/* DCF77 amplitude (CW) monitoring */
__IO uint32_t			giTim2Ch4_TS_ary[10]			= { 0 };

/* DCF77 phase modulation monitoring */
__IO uint16_t			giTim2Ch4_Phase_ary_idx							= 0U;
__IO uint32_t			giTim2Ch4_Phase_TS_idx_0						= 0UL;
__IO int32_t			giTim2Ch4_Phase_ary[PRN_CORRELATION_BUF_SIZE]	= { 0 };

/* DCF77 decoded time & date telegram data */
dcfTimeTelegr_t 		gDcfNxtMinuteTime				= { 0 };

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
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV4;
  sConfigIC.ICFilter = 3;
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
    GPIO_InitStruct.Pin = A2_DCF77_PHASE_TIM2_CH4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(A2_DCF77_PHASE_TIM2_CH4_GPIO_Port, &GPIO_InitStruct);

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
    HAL_GPIO_DeInit(A2_DCF77_PHASE_TIM2_CH4_GPIO_Port, A2_DCF77_PHASE_TIM2_CH4_Pin);

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
	if (htim == &htim2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			/* GPS 1PPS pulse captured */
			giTim2Ch2_TS = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

#if !defined(PLL_BY_SOFTWARE)
			/* First pulse of a second, only */
			if (giTim2Ch2_TS < 60000UL) {
#else
			/* 1 PPS mode */
			{
#endif
				int32_t diff = giTim2Ch2_TS - giTim2Ch2_TS_ary[giTim2Ch2_TS_ary_idx];

#if !defined(PLL_BY_SOFTWARE)
				if ((-100000 < diff) && (diff < +100000)) {
#else
				/* Clamp below +/-5 ppm */
				if ((-3000 < diff) && (diff < +3000)) {
#endif
					/* Store accumulated difference */
					++giTim2Ch2_TicksEvt;
					giTim2Ch2_TicksDiff += diff;

					/* Calculate PPMs */
					giTim2Ch2_ppm = diff / 600.0f;
				}

				if (giTim2Ch2_TicksEvt > 1UL) {
					/* Write back TimeStamp to 10 sec circle-buffer */
					giTim2Ch2_TS_ary[giTim2Ch2_TS_ary_idx++] = giTim2Ch2_TS;
					giTim2Ch2_TS_ary_idx %= 10;
				}
				else {
					/* Fast fill of the timestamp buffer */
					for (uint8_t idx = 0U; idx < 10U; ++idx) {
						giTim2Ch2_TS_ary[idx] = giTim2Ch2_TS;
					}
				}
			}
		}  // if (CHANNEL_2)

		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
			/* DCF77 77500 Hz / 4 = 19375 Hz pulse captured */
			static uint16_t cntCwClk	= 0UL;
			static uint8_t cntPhaseClk	= 0U;

			giTim2Ch4_TS				= HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);

			/* One per second */
			if (++cntCwClk >= 19375U) {
				cntCwClk = 0U;

				/* Timestamp @ 60 MHz */
				giTim2Ch4_TS_ary[9] = giTim2Ch4_TS_ary[8];
				giTim2Ch4_TS_ary[8] = giTim2Ch4_TS_ary[7];
				giTim2Ch4_TS_ary[7] = giTim2Ch4_TS_ary[6];
				giTim2Ch4_TS_ary[6] = giTim2Ch4_TS_ary[5];
				giTim2Ch4_TS_ary[5] = giTim2Ch4_TS_ary[4];
				giTim2Ch4_TS_ary[4] = giTim2Ch4_TS_ary[3];
				giTim2Ch4_TS_ary[3] = giTim2Ch4_TS_ary[2];
				giTim2Ch4_TS_ary[2] = giTim2Ch4_TS_ary[1];
				giTim2Ch4_TS_ary[1] = giTim2Ch4_TS_ary[0];
				giTim2Ch4_TS_ary[0] = giTim2Ch4_TS;
			}

			/* 120 (DCF77) / 4 (TIM2_IC4 div 4) 	= 30 -->  0 .. 29 */
			/* and 6x oversampling 					=  5 -->  0 ..  4 */
			if (cntPhaseClk++ >= 4U) {
				/* Phase data coded:		  [ 0.2 sec .. 0.9927742 sec ]
				 * One phase data bit clock	= 1.54839 ms = 92903.2258065 ticks @ 60 MHz
				 * One frame				= 512 x phase data bit clock = 792.7742 ms
				 * One second				= 645.8333 bit clocks, 6x oversampling = 3875 time stamps / sec ==>  480000 / 31 = 15483.87097...
				 * DCF77 PRN phase mod: +/-13 deg = +/-466 ns = +/- 28 ticks @ 60 MHz
				 */
				cntPhaseClk = 0U;

				if (giTim2Ch4_Phase_ary_idx == 0U) {
					giTim2Ch4_Phase_TS_idx_0 = giTim2Ch4_TS;
					giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx++] = giTim2Ch4_TS - giTim2Ch4_Phase_TS_idx_0;
				}
				else if (giTim2Ch4_Phase_ary_idx < PRN_CORRELATION_BUF_SIZE) {
					/* Record time stamp */
					giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx] = giTim2Ch4_TS - (giTim2Ch4_Phase_TS_idx_0 + ((giTim2Ch4_Phase_ary_idx * 480000UL + 15UL) / 31UL));

					/* Underflow / Overflow */
					if (giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx] < -30000000L) {
						giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx] += 60000000L;
					}
					else if (giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx] > 30000000L) {
						giTim2Ch4_Phase_ary[giTim2Ch4_Phase_ary_idx] -= 60000000L;
					}
					giTim2Ch4_Phase_ary_idx++;
				}
				else if (giTim2Ch4_Phase_ary_idx == PRN_CORRELATION_BUF_SIZE) {
					/* Recording ends */
					giTim2Ch4_Phase_ary_idx = 0xffffU;
				}
			}
		}  // if (CHANNEL_4)

	}  // if (htim == &htim2)
}


void tim_start(void)
{
	/* TIM2 IC CH2 NEO-x */
	{
		if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2) != HAL_OK) {
			/* Starting Error */
			Error_Handler();
		}
	}

	/* TIM2 IC CH4 DCF77*/
	{
		if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4) != HAL_OK) {
			/* Starting Error */
			Error_Handler();
		}
	}
}

uint32_t tim_get_timeStamp(TIM_HandleTypeDef *htim)
{
	return htim->Instance->CNT;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
