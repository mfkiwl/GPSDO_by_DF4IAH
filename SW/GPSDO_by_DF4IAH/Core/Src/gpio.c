/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
uint8_t onewireDevices[ONEWIRE_DEVICES_MAX][8];
uint8_t onewireDeviceCount;

extern void uDelay(uint16_t uDelay);

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D3_DCF77_DEMOD_GPIO_EXTI0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(D3_DCF77_DEMOD_GPIO_EXTI0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = D9_FRCD_HOLD_GPIO_I_Pin|D10_PLL_LCKD_GPIO_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D2_OCXO_LCKD_GPIO_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D2_OCXO_LCKD_GPIO_O_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D12_HoRelay_GPIO_O_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D12_HoRelay_GPIO_O_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = D11_ONEWIRE_GPIO_IO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(D11_ONEWIRE_GPIO_IO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = NoJ1J2_BOOT0_GPIO_I_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NoJ1J2_BOOT0_GPIO_I_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* ONEWIRE */

uint8_t onewire_CRC8_calc(uint8_t* fields, uint8_t len)
{
	uint8_t crc = 0U;

	for (; len; --len) {
		uint8_t pad = *(fields++);

		for (uint8_t bits = 8; bits; --bits) {
			uint8_t inBit = pad & 0x01U;
			uint8_t fbBit = inBit ^ (crc & 0x01U);

			/* New state */
			crc >>= 1;
			if (fbBit) {
				crc |= 0x80U;
				crc ^= 0x0cU;
			}

			pad >>= 1;
		}
	}
	return crc;
}


static void onewireMasterWr_bit(uint8_t bit)
{
	/* Ensure relaxation */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
	uDelay(2);

	/* TimeSlot starts */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_RESET);

	if (bit) {
		/* Writing a One */
		uDelay(2);
		HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
		uDelay(88);
	}
	else {
		/* Writing a Zero */
		uDelay(90);
		HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
	}

	/* Enter relaxation state */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
}

static void onewireMasterWr_byte(uint8_t byte)
{
	for (uint8_t idx = 0; idx < 8; ++idx) {
		onewireMasterWr_bit((byte >> idx) & 0x01U);
	}
}

static void onewireMasterWr_romCode(uint8_t* romCode)
{
	if (!romCode[0] && !romCode[1] && !romCode[2] && !romCode[3] && !romCode[4] && !romCode[5] && !romCode[6] && !romCode[7]) {
		romCode = 0;
	}
	if (!romCode) {
		return;
	}

	for (uint8_t len = 8; len; --len) {
		onewireMasterWr_byte(*(romCode++));
	}
}

static uint8_t onewireMasterRd_bit(void)
{
	/* Ensure relaxation */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
	uDelay(2);

	/* TimeSlot starts */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_RESET);
	uDelay(2);
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);

	/* Get read bit of slave */
	uDelay(13);
	GPIO_PinState pinstate = HAL_GPIO_ReadPin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin);
	uDelay(75);

	/* Enter relaxation state */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);

	return (pinstate == GPIO_PIN_SET);
}

static uint32_t onewireMasterRd_field(uint8_t bitCnt)
{
	uint32_t rdVal = 0UL;

	/* Paramter check */
	if (bitCnt > 32) {
		return 0xffffffffUL;
	}

	for (uint8_t idx = 0U; idx < bitCnt; ++idx) {
		if (onewireMasterRd_bit()) {
			rdVal |= (1UL << idx);
		}
	}

	return rdVal;
}

GPIO_PinState onewireMasterCheck_presence(void)
{
	/* Ensure the bus is inactive */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);
	uDelay(2000);

	/* 1w: Reset */
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_RESET);
	uDelay(550);
	HAL_GPIO_WritePin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin, GPIO_PIN_SET);

	/* Read back Presence */
	uDelay(90);
	GPIO_PinState presence = HAL_GPIO_ReadPin(D11_ONEWIRE_GPIO_IO_GPIO_Port, D11_ONEWIRE_GPIO_IO_Pin);
	uDelay(550);

	return presence;
}

uint8_t onewireMasterTree_search(uint8_t searchAlarms, uint8_t devicesMax, uint8_t onewireDevices[][8])
{
	uint8_t devicesCnt			= 0U;
	uint8_t bitIdxNow			= 0U;
	uint8_t direction			= 0U;
	int8_t bitIdxLastZero		= -1;
	int8_t discrepancyLast		= -1;
	uint8_t lastDeviceFlag		= 0U;
	uint8_t masterMind[64 / 8]	= { 0 };		// Keeps track of common path entries

	/* For any device, restart the whole path to find each of them on the bus */
	while (devicesCnt < devicesMax) {
		/* Any devices present? */
		if (GPIO_PIN_SET == onewireMasterCheck_presence()) {
			/* No devices */
			return 0;
		}

		/* End of tree */
		if (lastDeviceFlag) {
			break;
		}

		if (searchAlarms) {
			/* ALARM Search cmd */
			onewireMasterWr_byte(0xecU);
		}
		else {
			/* Search ROM cmd */
			onewireMasterWr_byte(0xf0U);
		}

		/* Step over each bit of the IDs */
		bitIdxNow 		= 0U;
		while (bitIdxNow < 64) {
			/* Get last */
			uint8_t bitNow = 0x01U & (masterMind[bitIdxNow >> 3] >> (bitIdxNow & 0x07U));

			uint8_t b_pos = onewireMasterRd_bit();
			uint8_t b_neg = onewireMasterRd_bit();

			if (!b_pos && b_neg) {
				/* Only (common or single) '0' */
				direction = 0U;
			}
			else if (b_pos && !b_neg) {
				/* Only (common or single) '1' */
				direction = 1U;
			}
			else if (!b_pos && !b_neg) {
				/* Discrepancy at this point of the path */

				if ((int8_t)bitIdxNow < bitIdxLastZero) {
					/* Follow last trace */
					direction = bitNow;
				}
				else if ((int8_t)bitIdxNow == bitIdxLastZero) {
					/* Select now the '1' branch */
					direction = 1U;
					bitIdxLastZero = -1;  // DF4IAH
				}
				else {
					/* Select the '0' branch */
					direction = 0U;
				}

				if (!direction) {
					bitIdxLastZero = bitIdxNow;
				}
			}
			else if (b_pos && b_neg) {
				/* No devices anymore */
				return 0;
			}

			/* Write direction to the path */
			if (direction > 0U) {
				masterMind[bitIdxNow >> 3] |=  (1U << (bitIdxNow & 0x07U));
			} else {
				masterMind[bitIdxNow >> 3] &= ~(1U << (bitIdxNow & 0x07U));
			}

			/* Write direction to the bus */
			onewireMasterWr_bit(direction);

			++bitIdxNow;
		}  // while (bitIdxNow < 64)

		discrepancyLast = bitIdxLastZero;
		if (discrepancyLast == -1) {
			lastDeviceFlag = 1U;
		}

		/* Copy over one valid device */
		for (int idx = 0; idx < (64 / 8); ++idx) {
			onewireDevices[devicesCnt][idx] = masterMind[idx];
		}
		++devicesCnt;
	}

	/* Issue a reset */
	onewireMasterCheck_presence();

	return devicesCnt;
}

void onewireDS18B20_readROMcode(uint8_t* romCode)
{
	if (!romCode[0] && !romCode[1] && !romCode[2] && !romCode[3] && !romCode[4] && !romCode[5] && !romCode[6] && !romCode[7]) {
		romCode = 0;
	}

	/* At least one device is present */
	if (GPIO_PIN_RESET == onewireMasterCheck_presence()) {
		/* Read ROM cmd */
		onewireMasterWr_byte(0x33U);

		/* Read all entries of the Scratchpad */
		for (int i = 0; i < 8; ++i) {
			*(romCode++) = (uint8_t) (onewireMasterRd_field(8) & 0xffUL);
		}
	}

	/* Issue a reset */
	onewireMasterCheck_presence();
}

void onewireDS18B20_setAdcWidth(uint8_t width, int8_t tempAlarmHi, int8_t tempAlarmLo, uint8_t* romCode)
{
	if (!romCode[0] && !romCode[1] && !romCode[2] && !romCode[3] && !romCode[4] && !romCode[5] && !romCode[6] && !romCode[7]) {
		romCode = 0;
	}

	uint8_t reg_Ctrl = 0b00011111;

	switch (width) {
	case 9:
		break;

	case 10:
		reg_Ctrl |= (0b01 << 5);
		break;

	case 11:
		reg_Ctrl |= (0b10 << 5);
		break;

	case 12:
	default:
		reg_Ctrl |= (0b11 << 5);
		break;
	}

	/* At least one device is present */
	if (GPIO_PIN_RESET == onewireMasterCheck_presence()) {
		if (!romCode) {
			/* Skip ROM cmd */
			onewireMasterWr_byte(0xccU);
		}
		else {
			/* Match ROM cmd */
			onewireMasterWr_byte(0x55U);
			onewireMasterWr_romCode(romCode);
		}

		/* Write Scratchpad */
		onewireMasterWr_byte(0x4eU);

		/* Alarm temperature high */
		onewireMasterWr_byte((uint8_t)tempAlarmHi);

		/* Alarm temperature low */
		onewireMasterWr_byte((uint8_t)tempAlarmLo);

		/* Configuration byte */
		onewireMasterWr_byte(reg_Ctrl);
	}

	/* Issue a reset */
	onewireMasterCheck_presence();
}

uint32_t onewireDS18B20_tempReq(uint8_t* romCode)
{
	if (!romCode[0] && !romCode[1] && !romCode[2] && !romCode[3] && !romCode[4] && !romCode[5] && !romCode[6] && !romCode[7]) {
		romCode = 0;
	}

	/* At least one device is present */
	if (GPIO_PIN_RESET == onewireMasterCheck_presence()) {
		if (!romCode) {
			/* Skip ROM cmd */
			onewireMasterWr_byte(0xccU);
		}
		else {
			/* Match ROM cmd */
			onewireMasterWr_byte(0x55U);
			onewireMasterWr_romCode(romCode);
		}

		/* Convert-T cmd */
		onewireMasterWr_byte(0x44U);

#if 0
		/* Change to Push-Pull mode */
		uint32_t bfOpenDrain = D11_ONEWIRE_GPIO_IO_GPIO_Port->OTYPER;
		uint32_t bfPushPull  = bfOpenDrain & (~D11_ONEWIRE_GPIO_IO_Pin);
		D11_ONEWIRE_GPIO_IO_GPIO_Port->OTYPER = bfPushPull;
#endif

		/* End time */
		uint32_t waitTime_ms = 760UL;
#if   defined(ONEWIRE_DS18B20_ADC_12B)
		waitTime_ms = 760UL;
#elif defined(ONEWIRE_DS18B20_ADC_11B)
		waitTime_ms = 375UL;
#elif defined(ONEWIRE_DS18B20_ADC_10B)
		waitTime_ms = 188UL;
#elif defined(ONEWIRE_DS18B20_ADC_09B)
		waitTime_ms =  94UL;
#endif
		return HAL_GetTick() + waitTime_ms;
	}

	/* No device present */
	return 0UL;
}

int16_t onewireDS18B20_tempRead(uint32_t waitUntil, uint8_t* romCode)
{
	if (!romCode[0] && !romCode[1] && !romCode[2] && !romCode[3] && !romCode[4] && !romCode[5] && !romCode[6] && !romCode[7]) {
		romCode = 0;
	}

	/* wait until ADC is ready */
	uint32_t t_now = HAL_GetTick();
	if (t_now < waitUntil) {
		HAL_Delay(waitUntil - t_now);
	}

	/* Revert to Open-Drain mode */
	uint32_t bfPushPull		= D11_ONEWIRE_GPIO_IO_GPIO_Port->OTYPER;
	uint32_t bfOpenDrain  	= bfPushPull | D11_ONEWIRE_GPIO_IO_Pin;
	D11_ONEWIRE_GPIO_IO_GPIO_Port->OTYPER = bfOpenDrain;

	/* 1w: Reset */
	onewireMasterCheck_presence();

	if (!romCode) {
		/* Skip ROM cmd */
		onewireMasterWr_byte(0xccU);
	}
	else {
		/* Match ROM cmd */
		onewireMasterWr_byte(0x55U);
		onewireMasterWr_romCode(romCode);
	}

	/* Read scratchpad */
	onewireMasterWr_byte(0xbeU);
	return (int16_t) onewireMasterRd_field(16);
}



/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
