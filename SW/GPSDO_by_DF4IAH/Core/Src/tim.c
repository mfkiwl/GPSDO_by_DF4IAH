/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
#include <string.h>

/* Timestamps from Callback functions */

__IO uint8_t			giTIM2_INT_DISABLE				= 0;
__IO uint8_t 			giTIM2_INT_CH4_CNT				= 0U;
__IO uint32_t			giTIM2_INT_CH4_BATCH[32] 		= { 0 };



/* GPS 1PPS monitoring */

__IO uint32_t			giTim15Ch2_TS					= 0UL;
__IO uint8_t			giTim15Ch2_TS_ary_idx			= 0U;
__IO uint32_t			giTim15Ch2_TS_ary[10]			= { 0 };

__IO uint32_t			giTim15Ch2_TicksEvt				= 0UL;
__IO int32_t			giTim15Ch2_TicksDiff			= 0L;
__IO int32_t			giTim2Ch2_TicksSumDev			= 0L;
__IO float 				giTim15Ch2_ppm					= 0.0f;


/* DCF77 RF signal monitoring */
__IO uint32_t			giTim2Ch2_TS_ary[10]			= { 0 };

/* DCF77 phase modulation monitoring */
__IO uint32_t			giTim2Ch2_TS_Phase_ary[PRN_CORRELATION_DOUBLE_BUF_SIZE]		= { 0 };

__IO uint8_t			giTim2Ch2_TS_PhaseDiff_ary_page	= 0U;
__IO int8_t			    giTim2Ch2_TS_PhaseDiff_ary[PRN_CORRELATION_SINGLE_BUF_SIZE]	= { 0 };


/* DCF77 decoded time & date telegram data */
dcfTimeTelegr_t 		gDcfNxtMinuteTime				= { 0 };

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim15;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV2;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM15 init function */
void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 59999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
    */
    GPIO_InitStruct.Pin = D13_DCF77_PHASE_TIM2_CH2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(D13_DCF77_PHASE_TIM2_CH2_GPIO_Port, &GPIO_InitStruct);

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
    hdma_tim2_ch2_ch4.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_tim2_ch2_ch4) != HAL_OK)
    {
      Error_Handler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC2],hdma_tim2_ch2_ch4);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC4],hdma_tim2_ch2_ch4);

  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */
    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM15 GPIO Configuration
    PA3     ------> TIM15_CH2
    */
    GPIO_InitStruct.Pin = A2_GPS_PPS_TIM15_CH2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
    HAL_GPIO_Init(A2_GPS_PPS_TIM15_CH2_GPIO_Port, &GPIO_InitStruct);

    /* TIM15 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /**TIM2 GPIO Configuration
    PB3 (JTDO-TRACESWO)     ------> TIM2_CH2
    */
    HAL_GPIO_DeInit(D13_DCF77_PHASE_TIM2_CH2_GPIO_Port, D13_DCF77_PHASE_TIM2_CH2_Pin);

    /* TIM2 DMA DeInit */
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC2]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC4]);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspDeInit 0 */

  /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM15_CLK_DISABLE();

    /**TIM15 GPIO Configuration
    PA3     ------> TIM15_CH2
    */
    HAL_GPIO_DeInit(A2_GPS_PPS_TIM15_CH2_GPIO_Port, A2_GPS_PPS_TIM15_CH2_Pin);

    /* TIM15 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
  /* USER CODE BEGIN TIM15_MspDeInit 1 */

  /* USER CODE END TIM15_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static void dcf_extract_seconds(void) {
	const uint32_t phaseTicksPerSec = 60000000UL;

	/* Timestamp @ 60 MHz */
	for (uint8_t mvIdx = 9U; mvIdx; mvIdx--) {
		giTim2Ch2_TS_ary[mvIdx] = giTim2Ch2_TS_ary[mvIdx - 1U];
	}

	giTim2Ch2_TS_ary[0U] = giTim2Ch2_TS_Phase_ary[PRN_CORRELATION_SINGLE_BUF_SIZE] % phaseTicksPerSec;
}

/* Every second half of the buffer gets ready */
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	/* TIM2: DCF77 timer */
	if (htim == &htim2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			/* First half is complete */
			for (uint16_t cnt = PRN_CORRELATION_SINGLE_BUF_SIZE, idx = 0; cnt; idx++, cnt--) {
				giTim2Ch2_TS_PhaseDiff_ary[idx] = (int8_t) (giTim2Ch2_TS_Phase_ary[idx] - giTim2Ch2_TS_Phase_ary[0] - ((idx * 2ULL * 31ULL * 60000000ULL) / 77500ULL));
			}

			/* Timestamp @ 60 MHz */
			dcf_extract_seconds();

			/* Page has changed */
			giTim2Ch2_TS_PhaseDiff_ary_page++;

			//HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_SET);
		}  // if (CHANNEL_2)
	}  // if (&htim2)
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* TIM15: GPS timer */
	if (htim == &htim15) {
		if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)) {
			/* GPS 1PPS pulse captured */
			giTim15Ch2_TS = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

#if !defined(PLL_BY_SOFTWARE)
			/* First pulse of a second, only */
			if (giTim15Ch2_TS < 60000UL) {
#else
			/* 1 PPS mode */
			{
#endif
				int32_t diff = giTim15Ch2_TS - giTim15Ch2_TS_ary[giTim15Ch2_TS_ary_idx];

#if !defined(PLL_BY_SOFTWARE)
				if ((-100000 < diff) && (diff < +100000)) {
#else
				/* Clamp below +/-5 ppm */
				if ((-3000 < diff) && (diff < +3000)) {
#endif
					/* Store accumulated difference */
					++giTim15Ch2_TicksEvt;
					giTim15Ch2_TicksDiff += diff;

					/* Calculate PPMs */
					giTim15Ch2_ppm = diff / 600.0f;
				}

				if (giTim15Ch2_TicksEvt > 1UL) {
					/* Write back TimeStamp to 10 sec circle-buffer */
					giTim15Ch2_TS_ary[giTim15Ch2_TS_ary_idx++] = giTim15Ch2_TS;
					giTim15Ch2_TS_ary_idx %= 10;
				}
				else {
					/* Fast fill of the timestamp buffer */
					for (uint8_t idx = 0U; idx < 10U; ++idx) {
						giTim15Ch2_TS_ary[idx] = giTim15Ch2_TS;
					}
				}
			}
		}  // if (CHANNEL_2)
	}  // if (htim == &htim15)

	/* TIM2: DCF77 timer */
	if (htim == &htim2) {
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			/* Second half is complete */
			for (uint16_t cnt = PRN_CORRELATION_SINGLE_BUF_SIZE, idxA = 0, idxB = PRN_CORRELATION_SINGLE_BUF_SIZE; cnt; idxA++, idxB++, cnt--) {
				giTim2Ch2_TS_PhaseDiff_ary[idxA] = (int8_t) (giTim2Ch2_TS_Phase_ary[idxB] - giTim2Ch2_TS_Phase_ary[PRN_CORRELATION_SINGLE_BUF_SIZE]  - ((idxA * 2ULL * 31ULL * 60000000ULL) / 77500ULL));
			}

			/* Timestamp @ 60 MHz */
			dcf_extract_seconds();

			/* Page has changed */
			giTim2Ch2_TS_PhaseDiff_ary_page++;

			//HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_RESET);
		}  // if (CHANNEL_2)
	}  // if (htim == &htim2)
}


void tim_TIM2_IC2_DMA_restart(void)
{
	if (HAL_TIM_IC_Stop_DMA( &htim2, TIM_CHANNEL_2) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}

	if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)giTim2Ch2_TS_Phase_ary, PRN_CORRELATION_DOUBLE_BUF_SIZE) != HAL_OK) {
		/* Starting Error */
		Error_Handler();
	}
}

void tim_start(void)
{
	/* TIM15 IC CH2 NEO-x */
	{
		if (HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2) != HAL_OK) {
			/* Starting Error */
			Error_Handler();
		}
	}

	/* TIM2 IC CH2 DCF77 PHASE */
	{
		if (HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)giTim2Ch2_TS_Phase_ary, PRN_CORRELATION_DOUBLE_BUF_SIZE) != HAL_OK) {
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
