/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */

ADC_ChannelConfTypeDef adcChConfig = { 0 };

uint8_t  adcCh9_lck			= 0U;
uint32_t adcCh9_val 		= 0UL;

uint8_t  adcCh10_lck		= 0U;
uint32_t adcCh10_val 		= 0UL;

uint8_t  adcCh16_lck		= 0U;
uint32_t adcCh16_val 		= 0UL;


#if 0
uint16_t adc_dma_buf[16] 		= { 0 };
const uint32_t ADC_DMA_Buf_Len 	= sizeof(adc_dma_buf) / sizeof(uint16_t);
#endif


/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  adcChConfig.Channel		= sConfig.Channel;
  adcChConfig.Rank			= sConfig.Rank;
  adcChConfig.SamplingTime	= sConfig.SamplingTime;
  adcChConfig.SingleDiff	= sConfig.SingleDiff;
  adcChConfig.OffsetNumber	= sConfig.OffsetNumber;
  adcChConfig.Offset		= sConfig.Offset;

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* ADC1 clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN9
    PA5     ------> ADC1_IN10
    PB1     ------> ADC1_IN16
    */
    GPIO_InitStruct.Pin = A3_V_OCXO_ADC1_IN9_Pin|A4_V_HOLD_ADC1_IN10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = D6_V_DCF77_DEMOD_ADC1_IN16_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(D6_V_DCF77_DEMOD_ADC1_IN16_GPIO_Port, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA4     ------> ADC1_IN9
    PA5     ------> ADC1_IN10
    PB1     ------> ADC1_IN16
    */
    HAL_GPIO_DeInit(GPIOA, A3_V_OCXO_ADC1_IN9_Pin|A4_V_HOLD_ADC1_IN10_Pin);

    HAL_GPIO_DeInit(D6_V_DCF77_DEMOD_ADC1_IN16_GPIO_Port, D6_V_DCF77_DEMOD_ADC1_IN16_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint8_t chSel = 0U;

	if (HAL_ADC_GetState(hadc) & HAL_ADC_STATE_REG_EOC) {
		/* One Convertion has finished */
		switch (chSel)
		{
		default:
		case 0:
	    	if (!adcCh9_lck) {
				/* Get the converted value of regular channel */
				adcCh9_val = HAL_ADC_GetValue(hadc);
	    	}
			break;

		case 1:
	    	if (!adcCh10_lck) {
				/* Get the converted value of regular channel */
	    		adcCh10_val = HAL_ADC_GetValue(hadc);
	    	}
			break;

		case 2:
	    	if (!adcCh16_lck) {
				/* Get the converted value of regular channel */
	    		adcCh16_val = HAL_ADC_GetValue(hadc);
	    	}
			break;
		}

    	/* Next time use next channel */
    	++chSel;
    	chSel %= 3;
	}

	uint32_t status = READ_REG(hadc->Instance->ISR);
	if (status & ADC_FLAG_EOS) {
	//if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOS)) {
		/* Sequence Convertion has finished */
		__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOS);
		chSel = 0;
	}

	if (status & ADC_FLAG_OVR) {
		__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
	}
}

#if 0
/* DMA 1/2 completion */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	static uint32_t ctr = 0UL;
	++ctr;
}
#endif

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
	static uint32_t ctr = 0UL;
	++ctr;
}


void ADC_init(void)
{
	if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK)
	{
		Error_Handler();
	}
}
void ADC_start(void)
{
	if (HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_READY) {
#if 1
		if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
			Error_Handler();
		}
#else
		if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_dma_buf, ADC_DMA_Buf_Len) != HAL_OK) {
			Error_Handler();
		}
#endif
	}
}

void ADC_stop(void)
{
	if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_READY) == 0) {
#if 1
		HAL_ADC_Stop_IT(&hadc1);
#else
		HAL_ADC_Stop_DMA(&hadc1);
#endif
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
