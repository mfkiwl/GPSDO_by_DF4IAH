/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"
#include "stdio.h"

/* USER CODE BEGIN 0 */

const uint8_t I2c_Lcd16x2_Welcome_L0_str[] = " (s)GPSDO a la  ";
const uint8_t I2c_Lcd16x2_Welcome_L1_str[] = " DF4IAH   V0.62 ";

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = D5_I2C1_SCL_Pin|D4_I2C1_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(D5_I2C1_SCL_GPIO_Port, D5_I2C1_SCL_Pin);

    HAL_GPIO_DeInit(D4_I2C1_SDA_GPIO_Port, D4_I2C1_SDA_Pin);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t i2cBusGetDeviceList(uint32_t* i2cDevicesBF)
{
	uint8_t i2cBusDeviceCnt = 0U;

	*i2cDevicesBF = 0UL;

	for (uint8_t i2cDevAddr = 1; i2cDevAddr < 128U; ++i2cDevAddr) {
		HAL_StatusTypeDef stat = HAL_I2C_IsDeviceReady(&hi2c1, (i2cDevAddr << 1), 1, 100);
		if (stat == HAL_OK) {
			/* I2C device on the bus */
			++i2cBusDeviceCnt;

			switch (i2cDevAddr) {
			case I2C_CHIP_ADDR_LCD_0:
				/* LCD 16x2 via Port-Expander MCP23017  */
				*i2cDevicesBF |= I2C_DEVICE_LCD_0;
				break;

			case I2C_CHIP_ADDR_LCD_1:
				/* LCD 16x2 via Port-Expander MCP23017  */
				*i2cDevicesBF |= I2C_DEVICE_LCD_1;
				break;

			case I2C_CHIP_ADDR_DAC_MCP4725_0:
				/* DAC 0 */
				*i2cDevicesBF |= I2C_DEVICE_DAC_MCP4725_0;
				break;

			case I2C_CHIP_ADDR_DAC_MCP4725_1:
				/* DAC 1 */
				*i2cDevicesBF |= I2C_DEVICE_DAC_MCP4725_1;
				break;
			}
		}
	}

	return i2cBusDeviceCnt;
}

uint8_t i2cDeviceDacMcp4725_set(uint8_t chipAddr, uint8_t pdMode, uint16_t dac_12b)
{
	uint8_t i2cTxBuf[2] = { 0 };

	/* A0 address bit and base address */
	chipAddr &= 0x01U;
	chipAddr |= 0x60U;
	chipAddr <<= 1;

	/* Power-Down mode */
	uint16_t dacFastWord = ((uint16_t)pdMode & 0x0003U) << 12;

	/* unsigned 12 bit DAC value */
	dacFastWord |= dac_12b & 0x0fffU;

	/* Fill in data */
	i2cTxBuf[0] = (uint8_t) ((dacFastWord >> 8) & 0xffU);
	i2cTxBuf[1] = (uint8_t) ( dacFastWord       & 0xffU);

    /* Write data to the DAC chip */
	HAL_StatusTypeDef stat = HAL_I2C_Master_Transmit_IT(&hi2c1, chipAddr, i2cTxBuf, sizeof(i2cTxBuf));
	if (stat != HAL_OK) {
		return 1;
	}

	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
		return 2;
	}

	return 0;
}


static uint8_t i2cMCP23017_Lcd16x2_Write(uint8_t cmd, uint8_t rs)
{
	uint8_t i2cTxBuf[3];
	HAL_StatusTypeDef stat;

	if (rs) {
		i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_A);
		i2cTxBuf[1] = cmd;
		i2cTxBuf[2] = 0b00001100;	// 0b0000 . LED . RS . R/!W . E
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
		if (stat != HAL_OK) {
			return 1;
		}
		/* Wait until transfer has completed */
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}

		i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_B);
		i2cTxBuf[1] = 0b00001101;	// 0b0000 . LED . RS . R/!W . E
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
		if (stat != HAL_OK) {
			return 1;
		}
	}
	else {
		i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_A);
		i2cTxBuf[1] = cmd;
		i2cTxBuf[2] = 0b00001001;	// 0b0000 . LED . RS . R/!W . E
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
		if (stat != HAL_OK) {
			return 1;
		}
	}
	/* Wait until transfer has completed */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}
	HAL_Delay(1);

	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_B);
	i2cTxBuf[1] = 0b00001000;	// 0b0000 . LED . RS . R/!W . E
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}
	HAL_Delay(1);

	return 0;
}

uint8_t i2cMCP23017_Lcd16x2_ClrScr(void)
{
	/* ClrScr */
	return i2cMCP23017_Lcd16x2_Write(0x01U, 0U);
}

uint8_t i2cMCP23017_Lcd16x2_SetAddr(uint8_t row, uint8_t col)
{
	row &= 0x01U;
	col &= 0x0fU;
	uint8_t cmd = 0x80 | (row << 6) | col;

	/* Set DDRAM address */
	return i2cMCP23017_Lcd16x2_Write(cmd, 0U);
}

uint8_t i2cMCP23017_Lcd16x2_WriteStr(uint8_t* str, uint8_t len)
{
	for (; len; --len) {
		/* Character */
		if (i2cMCP23017_Lcd16x2_Write(*(str++), 1U)) {
			return 1;
		}
	}
	return 0;
}

static uint8_t i2cMCP23017_Lcd16x2_Init(void)
{
	uint8_t i2cTxBuf[3];
	HAL_StatusTypeDef stat;

	/* IO-Dir of port A/B */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_IODIR_A);
	i2cTxBuf[1] = 0xffU;	// Input until R/!W signal is stable
	i2cTxBuf[2] = 0xf0U;	// Output for all used pins
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* Pull-up of port A/B */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPPU_A);
	i2cTxBuf[1] = 0xffU;	// Pull up all data pins
	i2cTxBuf[2] = 0xf0U;	// Pull up all unused pins
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* GPIO bits of port A/B - turn backlight on */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_A);
	i2cTxBuf[1] = 0x00U;		//
	i2cTxBuf[2] = 0b00001000;	// 0b0000 . LED . RS . R/!W . E
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* IOCON for port A/B */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_IOCON_A);
	i2cTxBuf[1] = 0x00;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* Turn Port A to output direction */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_IODIR_A);
	i2cTxBuf[1] = 0x00U;	// Output mode
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }


	/* RESET sequence starts */

	/* Function set Interface has to be sent 4 times */
	for (uint8_t cnt = 4; cnt; --cnt) {
		/* Function Set */
		if (i2cMCP23017_Lcd16x2_Write(0x38U, 0U)) {
			return 1;
		}
	}

	/* Display OFF */
	if (i2cMCP23017_Lcd16x2_Write(0x08U, 0U)) {
		return 1;
	}

	i2cMCP23017_Lcd16x2_ClrScr();

	/* Entry Mode Set */
	if (i2cMCP23017_Lcd16x2_Write(0x06U, 0U)) {
		return 1;
	}

	/* Display ON */
	if (i2cMCP23017_Lcd16x2_Write(0x0cU, 0U)) {
		return 1;
	}

	return 0;
}

void i2cMCP23017_Lcd16x2_Welcome(void)
{
	i2cMCP23017_Lcd16x2_Init();

	/* Goto first line */
	i2cMCP23017_Lcd16x2_SetAddr(0U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr((uint8_t*)I2c_Lcd16x2_Welcome_L0_str, sizeof(I2c_Lcd16x2_Welcome_L0_str) - 1);

	/* Goto second line */
	i2cMCP23017_Lcd16x2_SetAddr(1U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr((uint8_t*)I2c_Lcd16x2_Welcome_L1_str, sizeof(I2c_Lcd16x2_Welcome_L1_str) - 1);
}

void i2cMCP23017_Lcd16x2_OCXO_HeatingUp(int16_t temp, uint32_t tAcc)
{
	uint8_t line0_str[] = "== Heating up ==";
	uint8_t line1_str[] = "                ";

	if (temp && tAcc) {
		if (tAcc > 999UL) {
			tAcc = 999UL;
		}
		snprintf((char*)line1_str, sizeof(line1_str), "%02d%cC / Acc %3ldns", temp, 0xdfU, tAcc);
	}

	/* First line */
	i2cMCP23017_Lcd16x2_SetAddr(0U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr(line0_str, sizeof(line0_str) - 1);

	/* Second line */
	i2cMCP23017_Lcd16x2_SetAddr(1U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr(line1_str, sizeof(line1_str) - 1);
}

void i2cMCP23017_Lcd16x2_Locked(int16_t temp, uint32_t tAcc, int32_t sumDev)
{
	uint8_t line0_str[17];
	uint8_t line1_str[17];

	if (tAcc > 999UL) {
		tAcc = 999UL;
	}

	snprintf((char*)line0_str, sizeof(line0_str), "== Lockd %02d%cC ==", temp, 0xdfU);
	snprintf((char*)line1_str, sizeof(line1_str), "%+05ldps/s, %3ldns", sumDev, tAcc);

	/* First line */
	i2cMCP23017_Lcd16x2_SetAddr(0U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr(line0_str, sizeof(line0_str) - 1);

	/* Second line */
	i2cMCP23017_Lcd16x2_SetAddr(1U, 0U);
	i2cMCP23017_Lcd16x2_WriteStr(line1_str, sizeof(line1_str) - 1);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
