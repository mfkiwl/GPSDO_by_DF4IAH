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

/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "string.h"
#include "math.h"

#include "usart.h"


const uint8_t			I2c_Lcd_Welcome_L0_P1_str[] 	= "(s)GPSDO a la";
const uint8_t 			I2c_Lcd_Welcome_L1_P1_str[]		= "DF4IAH";
const uint8_t 			I2c_Lcd_Welcome_L1_P2_str[]		= "V0.73";

const uint8_t 			I2c_Lcd_Welcome_L2_str[] 		= "on board:";
const uint8_t 			I2c_Lcd_Welcome_L3_str[] 		= "  - OCXO: 10 MHz";
const uint8_t 			I2c_Lcd_Welcome_L4_str[] 		= "  - GPS:  ublox NEO-x       @  1 PPS";
const uint8_t 			I2c_Lcd_Welcome_L5_str[] 		= "  - DAC:  MCP4725 12-Bit";
const uint8_t 			I2c_Lcd_Welcome_L6_str[] 		= "  - MCU:  STM32L432KC       @ 60 MHz";

const uint8_t 			DacGfxPos_x_min 				=   9U;
const uint8_t 			DacGfxPos_x_max 				= 230U;
const uint8_t 			DacGfxPos_y_bot 				=  43U;
const uint8_t 			DacGfxPos_y_top 				= DacGfxPos_y_bot - 30U;

uint8_t  				i2cDacModeLast					= 0U;
uint8_t  				i2cDacMode						= 0U;
uint16_t 				i2cDacValLast					= 0U;
uint16_t 				i2cDacVal 						= 0U;
float 	 				i2cDacFraction 					= 0.0f;

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
  hi2c1.Init.Timing = 0x00403E5A;
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
				/* LCD Gfx 240x128 via Smart-LCD  */
				*i2cDevicesBF |= I2C_DEVICE_LCD_1;
				break;

			case I2C_CHIP_ADDR_LCD_DIGPOT_1:
				/* LCD Gfx 240x128 via Smart-LCD (Digital POT if installed) */
				*i2cDevicesBF |= I2C_DEVICE_LCD_DIGPOT_1;
				break;

			case I2C_CHIP_ADDR_DAC_MCP4725_0:
				/* DAC 0 */
				*i2cDevicesBF |= I2C_DEVICE_DAC_MCP4725_0;
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
		return 1U;
	}

	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
		return 2U;
	}
	return 0U;
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
			return 1U;
		}
		/* Wait until transfer has completed */
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}

		i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_B);
		i2cTxBuf[1] = 0b00001101;	// 0b0000 . LED . RS . R/!W . E
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
		if (stat != HAL_OK) {
			return 1U;
		}
	}
	else {
		i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_A);
		i2cTxBuf[1] = cmd;
		i2cTxBuf[2] = 0b00001001;	// 0b0000 . LED . RS . R/!W . E
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 3);
		if (stat != HAL_OK) {
			return 1U;
		}
	}
	/* Wait until transfer has completed */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}
	HAL_Delay(1UL);

	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_GPIO_B);
	i2cTxBuf[1] = 0b00001000;	// 0b0000 . LED . RS . R/!W . E
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
	}
	HAL_Delay(1UL);
	return 0U;
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
			return 1U;
		}
	}
	return 0U;
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
		return 1U;
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
		return 1U;
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
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* IOCON for port A/B */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_IOCON_A);
	i2cTxBuf[1] = 0x00;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }

	/* Turn Port A to output direction */
	i2cTxBuf[0] = (uint8_t) (I2C_MCP23017_ADDR_IODIR_A);
	i2cTxBuf[1] = 0x00U;	// Output mode
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_0 << 1), i2cTxBuf, 2);
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }


	/* RESET sequence starts */

	/* Function set Interface has to be sent 4 times */
	for (uint8_t cnt = 4; cnt; --cnt) {
		/* Function Set */
		if (i2cMCP23017_Lcd16x2_Write(0x38U, 0U)) {
			return 1U;
		}
	}

	/* Display OFF */
	if (i2cMCP23017_Lcd16x2_Write(0x08U, 0U)) {
		return 1U;
	}

	i2cMCP23017_Lcd16x2_ClrScr();

	/* Entry Mode Set */
	if (i2cMCP23017_Lcd16x2_Write(0x06U, 0U)) {
		return 1U;
	}

	/* Display ON */
	if (i2cMCP23017_Lcd16x2_Write(0x0cU, 0U)) {
		return 1U;
	}
	return 0U;
}

void i2cMCP23017_Lcd16x2_Welcome(void)
{
	i2cMCP23017_Lcd16x2_Init();

	/* Goto first line */
	i2cMCP23017_Lcd16x2_SetAddr(0U,  1U);
	i2cMCP23017_Lcd16x2_WriteStr((uint8_t*)I2c_Lcd_Welcome_L0_P1_str, sizeof(I2c_Lcd_Welcome_L0_P1_str) - 1);

	/* Goto second line */
	i2cMCP23017_Lcd16x2_SetAddr(1U,  1U);
	i2cMCP23017_Lcd16x2_WriteStr((uint8_t*)I2c_Lcd_Welcome_L1_P1_str, sizeof(I2c_Lcd_Welcome_L1_P1_str) - 1);

	i2cMCP23017_Lcd16x2_SetAddr(1U, 10U);
	i2cMCP23017_Lcd16x2_WriteStr((uint8_t*)I2c_Lcd_Welcome_L1_P2_str, sizeof(I2c_Lcd_Welcome_L1_P2_str) - 1);
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


static uint8_t i2cSmartLCD_Gfx240x128_Read(uint8_t cmd)
{
	HAL_StatusTypeDef stat;
	uint8_t i2cTxBuf[1];
	uint8_t i2cRxBuf[1]	= { 0 };

	i2cTxBuf[0] = cmd;

	stat = HAL_I2C_Master_Seq_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, sizeof(i2cTxBuf), I2C_FIRST_FRAME);
	if (stat != HAL_OK) {
		return 0x00U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
		/* No ACK */
		return 0x00U;
	}

	stat = HAL_I2C_Master_Seq_Receive_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cRxBuf, sizeof(i2cRxBuf), I2C_LAST_FRAME);
	if (stat != HAL_OK) {
		return 0x00U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	if (HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF) {
		/* No ACK */
		return 0x00U;
	}

	/* Returned byte */
	return i2cRxBuf[0];
}

static uint8_t i2cSmartLCD_Gfx240x128_Busy_wait(uint32_t timeout_ms)
{
	const uint32_t 	timeout_ts = timeout_ms + HAL_GetTick();
	uint32_t 		now_ts;
	uint8_t 		lcd1State;

	do {
		lcd1State = i2cSmartLCD_Gfx240x128_Read(LCD1_SMART_LCD_CMD_GET_STATE);

		if (!(lcd1State & 0x01)) {
			/* Not busy - ready for new command */
			return 0U;
		}

		/* Check for current timestamp */
		now_ts = HAL_GetTick();

		if (timeout_ts <= now_ts) {
			break;
		}

		/* Delay for next test */
		HAL_Delay(1UL);
	} while (1);
	return 1U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Write_parcnt0(uint8_t cmd)
{
	HAL_StatusTypeDef stat;
	uint8_t i2cTxBuf[1];

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	i2cTxBuf[0] = cmd;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, sizeof(i2cTxBuf));
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	/* Check for ACK */
	if (HAL_I2C_GetError(&hi2c1) == I2C_FLAG_AF) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Write_parcnt1(uint8_t cmd, uint8_t par1)
{
	HAL_StatusTypeDef stat;
	uint8_t i2cTxBuf[2];

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	i2cTxBuf[0] = cmd;
	i2cTxBuf[1] = par1;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, sizeof(i2cTxBuf));
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	/* Check for ACK */
	if (HAL_I2C_GetError(&hi2c1) == I2C_FLAG_AF) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Write_parcnt2(uint8_t cmd, uint8_t par1, uint8_t par2)
{
	HAL_StatusTypeDef stat;
	uint8_t i2cTxBuf[3];

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	i2cTxBuf[0] = cmd;
	i2cTxBuf[1] = par1;
	i2cTxBuf[2] = par2;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, sizeof(i2cTxBuf));
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	/* Check for ACK */
	if (HAL_I2C_GetError(&hi2c1) == I2C_FLAG_AF) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Write_parcnt3(uint8_t cmd, uint8_t par1, uint8_t par2, uint8_t par3)
{
	HAL_StatusTypeDef stat;
	uint8_t i2cTxBuf[4];

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	i2cTxBuf[0] = cmd;
	i2cTxBuf[1] = par1;
	i2cTxBuf[2] = par2;
	i2cTxBuf[3] = par3;
	stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, sizeof(i2cTxBuf));
	if (stat != HAL_OK) {
		return 1U;
	}
	/* Wait until transfer has completed */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    }
	/* Check for ACK */
	if (HAL_I2C_GetError(&hi2c1) == I2C_FLAG_AF) {
		return 1U;
	}
	return 0U;
}

uint8_t i2cSmartLCD_Gfx240x128_GetVer(void)
{
	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	return i2cSmartLCD_Gfx240x128_Read(LCD1_SMART_LCD_CMD_GET_VER);
}

uint8_t i2cSmartLCD_Gfx240x128_WriteText(uint8_t pos_x, uint8_t pos_y, uint8_t len, const uint8_t* str)
{
	HAL_StatusTypeDef stat;
	uint8_t remaining;
	uint8_t i2cTxBuf[256] = { 0 };

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= pos_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= pos_y) {
			Error_Handler();
			return 3U;
		}

		/* Overflow */
		if (((LCD1_SMART_LCD_SIZE_X - 1U) - (len * LCD1_SYSFONT_WIDTH)) < pos_x) {
			Error_Handler();
			return 4U;
		}
	}

	while (len) {
		/* Partitioning */
		if (len > LCD1_SMART_LCD_STR_MAXLEN_BUG) {
			remaining = LCD1_SMART_LCD_STR_MAXLEN_BUG;
		} else {
			remaining = len;
		}
		len -= remaining;

		/* Set cursor */
		if (i2cSmartLCD_Gfx240x128_Write_parcnt2(LCD1_SMART_LCD_CMD_SET_POS_X_Y, pos_x, pos_y)) {
			return 1U;
		}

		/* Copy send buffer */
		i2cTxBuf[0] = LCD1_SMART_LCD_CMD_WRITE;
		i2cTxBuf[1] = remaining;
		for (uint8_t idx = 0U; idx < remaining; ++idx) {
			i2cTxBuf[2 + idx] = *(str++);
		}

		/* Delay until display not busy */
		i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

		/* Busy flag does not work reliable when printing glyphs, add extra delay */
		HAL_Delay(2UL);

		/* Write Text since pen position */
		stat = HAL_I2C_Master_Transmit_IT(&hi2c1, (I2C_CHIP_ADDR_LCD_1 << 1), i2cTxBuf, (remaining + 2));
		if (stat != HAL_OK) {
			return 1U;
		}
		/* Wait until transfer has completed */
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
		}
		/* Check for ACK */
		if (HAL_I2C_GetError(&hi2c1) == I2C_FLAG_AF) {
			return 1U;
		}

		/* Busy flag does not work reliable when printing glyphs, add extra delay */
		HAL_Delay(2UL);

		pos_x += remaining * LCD1_SYSFONT_WIDTH;  // Smart-LCD: sysfont->width
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Draw_SetStartPos(uint8_t fromPos_x, uint8_t fromPos_y)
{
	/* Smart-LCD: TWI_SMART_LCD_CMD_SET_POS_X_Y */

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= fromPos_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= fromPos_y) {
			Error_Handler();
			return 3U;
		}
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	/* Set cursor */
	if (i2cSmartLCD_Gfx240x128_Write_parcnt2(LCD1_SMART_LCD_CMD_SET_POS_X_Y, fromPos_x, fromPos_y)) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Draw_Line_to(uint8_t toPos_x, uint8_t toPos_y, uint8_t fillType)
{
	/* Smart-LCD: TWI_SMART_LCD_CMD_DRAW_LINE */

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= toPos_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= toPos_y) {
			Error_Handler();
			return 3U;
		}
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	/* Set cursor */
	if (i2cSmartLCD_Gfx240x128_Write_parcnt3(LCD1_SMART_LCD_CMD_DRAW_LINE, toPos_x, toPos_y, fillType)) {
		return 1U;
	}
	return 0U;
}

#if defined(PLL_BY_SOFTWARE)
static uint8_t i2cSmartLCD_Gfx240x128_Draw_Point(uint8_t pos_x, uint8_t pos_y, uint8_t fillType)
{
	uint8_t ret;

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= pos_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= pos_y) {
			Error_Handler();
			return 3U;
		}
	}

	/* There is no explicit draw pixel in Smart-LCD */
	ret  = i2cSmartLCD_Gfx240x128_Draw_SetStartPos(pos_x, pos_y);
	ret |= i2cSmartLCD_Gfx240x128_Draw_Line_to(pos_x, pos_y, fillType);
	return ret;
}
#endif

static uint8_t i2cSmartLCD_Gfx240x128_Draw_Rect(uint8_t pos_LT_x, uint8_t pos_LT_y, uint8_t width, uint8_t height, uint8_t lineType)
{
	/* Smart-LCD: TWI_SMART_LCD_CMD_SET_POS_X_Y */
	/* Smart-LCD: TWI_SMART_LCD_CMD_DRAW_RECT */

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= pos_LT_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= pos_LT_y) {
			Error_Handler();
			return 3U;
		}

		/* Overflow */
		if (LCD1_SMART_LCD_SIZE_X <= (pos_LT_x + (width - 1U))) {
			Error_Handler();
			return 4U;
		}

		/* Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= (pos_LT_y + (height - 1U))) {
			Error_Handler();
			return 5U;
		}
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	/* Set cursor */
	if (i2cSmartLCD_Gfx240x128_Write_parcnt2(LCD1_SMART_LCD_CMD_SET_POS_X_Y, pos_LT_x, pos_LT_y)) {
		return 1U;
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	if (i2cSmartLCD_Gfx240x128_Write_parcnt3(LCD1_SMART_LCD_CMD_DRAW_RECT, width, height, lineType)) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Draw_Rect_filled(uint8_t pos_LT_x, uint8_t pos_LT_y, uint8_t width, uint8_t height, uint8_t fillType)
{
	/* Smart-LCD: TWI_SMART_LCD_CMD_SET_POS_X_Y */
	/* Smart-LCD: TWI_SMART_LCD_CMD_DRAW_FILLED_RECT */

	/* Frame checks */
	{
		/* Underflow */
		if (LCD1_SMART_LCD_SIZE_X <= pos_LT_x) {
			Error_Handler();
			return 2U;
		}

		/* Underflow/Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= pos_LT_y) {
			Error_Handler();
			return 3U;
		}

		/* Overflow */
		if (LCD1_SMART_LCD_SIZE_X <= (pos_LT_x + (width - 1U))) {
			Error_Handler();
			return 4U;
		}

		/* Overflow */
		if (LCD1_SMART_LCD_SIZE_Y <= (pos_LT_y + (height - 1U))) {
			Error_Handler();
			return 5U;
		}
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	/* Set cursor */
	if (i2cSmartLCD_Gfx240x128_Write_parcnt2(LCD1_SMART_LCD_CMD_SET_POS_X_Y, pos_LT_x, pos_LT_y)) {
		return 1U;
	}

	/* Delay until display not busy */
	i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

	if (i2cSmartLCD_Gfx240x128_Write_parcnt3(LCD1_SMART_LCD_CMD_DRAW_FILLED_RECT, width, height, fillType)) {
		return 1U;
	}
	return 0U;
}

static uint8_t i2cSmartLCD_Gfx240x128_Init(void)
{
	/* GetVer */
	uint8_t ver = i2cSmartLCD_Gfx240x128_GetVer();

	if (ver >= 0x11) {
		/* Delay until display not busy */
		i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

		/* SetMode */
		i2cSmartLCD_Gfx240x128_Write_parcnt1(LCD1_SMART_LCD_CMD_SET_MODE, LCD1_SMART_LCD_MODE_SMARTLCD);

		/* Delay until display not busy */
		i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

		/* ClrScr */
		if (i2cSmartLCD_Gfx240x128_Write_parcnt0(LCD1_SMART_LCD_CMD_CLS)) {
			return 2U;
		}

		/* Delay until display not busy */
		i2cSmartLCD_Gfx240x128_Busy_wait(1000UL);

		/* Default: Pen ON */
		if (i2cSmartLCD_Gfx240x128_Write_parcnt1(LCD1_SMART_LCD_CMD_SET_PIXEL_TYPE, LCD1_PIXEL_SET)) {
			return 1U;
		}

		HAL_Delay(10UL);
		return 0U;
	}
	return 1U;
}

uint8_t i2cSmartLCD_Gfx240x128_Template(uint32_t bf)
{
	/* Display init */
	if (bf & 0x80000000UL) {
		if (i2cSmartLCD_Gfx240x128_Init()) {
			return 1U;
		}
	}

	/* Header Line */
	if (bf & 0x00000001UL) {
		i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
				0U,
				(0U + (LCD1_SYSFONT_HEIGHT * 1U) + 1U));

		i2cSmartLCD_Gfx240x128_Draw_Line_to(
				(LCD1_SMART_LCD_SIZE_X - 1U),
				(0U + (LCD1_SYSFONT_HEIGHT * 1U) + 1U),
				LCD1_PIXEL_SET);
	}

	/* Header Text */
	if (bf & 0x00000002UL) {
		i2cSmartLCD_Gfx240x128_WriteText(
				0 + (LCD1_SYSFONT_WIDTH  *  5U),
				0 + (LCD1_SYSFONT_HEIGHT *  0U),
				strlen((char*)I2c_Lcd_Welcome_L0_P1_str), I2c_Lcd_Welcome_L0_P1_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				0 + (LCD1_SYSFONT_WIDTH  * 19U),
				0 + (LCD1_SYSFONT_HEIGHT *  0U),
				strlen((char*)I2c_Lcd_Welcome_L1_P1_str), I2c_Lcd_Welcome_L1_P1_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				0 + (LCD1_SYSFONT_WIDTH  * 27U),
				0 + (LCD1_SYSFONT_HEIGHT *  0U),
				strlen((char*)I2c_Lcd_Welcome_L1_P2_str), I2c_Lcd_Welcome_L1_P2_str);
	}


	/* LCKD print */
	if (bf & 0x00000010UL) {
		uint8_t line_str[] = "LCKD";
		if (i2cSmartLCD_Gfx240x128_WriteText(
				0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  0U),
				0U + ((LCD1_SYSFONT_HEIGHT + 0U) *  0U),
				strlen((char*)line_str), line_str)) {
			return 1U;
		}
	}


	/* Timing scale below the header line */
	if (bf & 0x00000100UL) {
		uint8_t pos_x 		= DacGfxPos_x_min;
		uint8_t mrk10Min	= 0U;
		uint8_t mrk5Dac		= 0U;

		/* Draw plot line x-axis */
		{
			i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
					DacGfxPos_x_min,
					DacGfxPos_y_bot);

			i2cSmartLCD_Gfx240x128_Draw_Line_to(
					DacGfxPos_x_max,
					DacGfxPos_y_bot,
					LCD1_PIXEL_SET);
		}

		/* Draw x-axis scale lines */
		while (pos_x <= DacGfxPos_x_max) {
			uint8_t drawHrExtra = (!(mrk10Min % 6)) ?  2U : 0U;

			i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
					pos_x,
					(DacGfxPos_y_bot + (1U + drawHrExtra)));

			i2cSmartLCD_Gfx240x128_Draw_Line_to(
					pos_x,
					(DacGfxPos_y_bot + 1U),
					LCD1_PIXEL_SET);

			/* New scale mark at every 10 minutes */
			++mrk10Min;
			pos_x = DacGfxPos_x_min + (10U * mrk10Min);
		}

		/* Draw plot line y-axis */
		{
			i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
					DacGfxPos_x_min,
					DacGfxPos_y_bot);

			i2cSmartLCD_Gfx240x128_Draw_Line_to(
					DacGfxPos_x_min,
					DacGfxPos_y_top,
					LCD1_PIXEL_SET);
		}

		/* Draw y-axis scale lines */
		while ((DacGfxPos_y_bot - (mrk5Dac * 5U)) >= DacGfxPos_y_top) {
			uint8_t drawExtra = (mrk5Dac == 3U) ?  2U : 0U;

			i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
					(DacGfxPos_x_min - (1 + drawExtra)),
					(DacGfxPos_y_bot - (mrk5Dac * 5U)));

			i2cSmartLCD_Gfx240x128_Draw_Line_to(
					(DacGfxPos_x_min - 1U),
					(DacGfxPos_y_bot - (mrk5Dac * 5U)),
					LCD1_PIXEL_SET);

			++mrk5Dac;
		}

		/* Write 'DAC' at y-axis */
		{
			uint8_t buf[] = "DAC";

			i2cSmartLCD_Gfx240x128_WriteText(
					(DacGfxPos_x_min - (LCD1_SYSFONT_WIDTH + 3U)),
					(DacGfxPos_y_bot - (3U * (LCD1_SYSFONT_HEIGHT + 2U) + 1U)),
					1U, &(buf[0]));

			i2cSmartLCD_Gfx240x128_WriteText(
					(DacGfxPos_x_min - (LCD1_SYSFONT_WIDTH + 3U)),
					(DacGfxPos_y_bot - (2U * (LCD1_SYSFONT_HEIGHT + 2U) + 1U)),
					1U, &(buf[1]));

			i2cSmartLCD_Gfx240x128_WriteText(
					(DacGfxPos_x_min - (LCD1_SYSFONT_WIDTH + 3U)),
					(DacGfxPos_y_bot - (1U * (LCD1_SYSFONT_HEIGHT + 2U) + 1U)),
					1U, &(buf[2]));
		}
	}
	return 0U;
}

uint8_t i2cSmartLCD_Gfx240x128_Welcome(void)
{
	if (i2cSmartLCD_Gfx240x128_Template(0x80000003UL)) {
		return 1U;
	}

	/* Write welcome */
	{
		i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  1U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  2U)),
				strlen((char*)I2c_Lcd_Welcome_L2_str), I2c_Lcd_Welcome_L2_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  1U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  3U)),
				strlen((char*)I2c_Lcd_Welcome_L3_str), I2c_Lcd_Welcome_L3_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  1U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  4U)),
				strlen((char*)I2c_Lcd_Welcome_L4_str), I2c_Lcd_Welcome_L4_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  1U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  5U)),
				strlen((char*)I2c_Lcd_Welcome_L5_str), I2c_Lcd_Welcome_L5_str);

		i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) *  1U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  6U)),
				strlen((char*)I2c_Lcd_Welcome_L6_str), I2c_Lcd_Welcome_L6_str);
	}
	return 0U;
}

uint8_t i2cSmartLCD_Gfx240x128_OCXO_HeatingUp(int16_t temp, uint32_t tAcc)
{
	/* Draw message box */
	{
		if (i2cSmartLCD_Gfx240x128_Draw_Rect(
				(      ((LCD1_SYSFONT_WIDTH  + 0U) * 11U)  - 4U),
				(      ((LCD1_SYSFONT_HEIGHT + 3U) *  8U)  - 4U),
				( 8U + ((LCD1_SYSFONT_WIDTH  + 0U) * 17U)      ),
				(10U + ((LCD1_SYSFONT_HEIGHT + 3U) *  3U)      ),
				LCD1_PIXEL_SET)) {
			return 1U;
		}
		HAL_Delay(1UL);
	}

	/* Write Heating up Header */
	{
		uint8_t line0_str[] = "== Heating up ==";

		if (i2cSmartLCD_Gfx240x128_WriteText(
				(0U + ((LCD1_SYSFONT_WIDTH  + 0U) * 11U)),
				(0U + ((LCD1_SYSFONT_HEIGHT + 3U) *  8U)),
				strlen((char*)line0_str), line0_str)) {
			return 1U;
		}

		if (temp) {
			/* Update OCXO temperature */
			uint8_t line1_str[32];

			snprintf((char*)line1_str, sizeof(line1_str) - 1, "OCXO temp:  %2d%cC", temp, 0x7eU);

			if (i2cSmartLCD_Gfx240x128_WriteText(
					(0U + ((LCD1_SYSFONT_WIDTH  + 0U) * 11U)),
					(2U + ((LCD1_SYSFONT_HEIGHT + 3U) *  9U)),
					strlen((char*)line1_str), line1_str)) {
				return 1U;
			}
		}

		if (tAcc) {
			/* Update ublox NEO tAcc */
			uint8_t line2_str[32];

			snprintf((char*)line2_str, sizeof(line2_str) - 1, "NEO  tAcc: %3ld ns", (tAcc > 999U ?  999U : tAcc));

			if (i2cSmartLCD_Gfx240x128_WriteText(
					(0U + ((LCD1_SYSFONT_WIDTH  + 0U) * 11U)),
					(2U + ((LCD1_SYSFONT_HEIGHT + 3U) * 10U)),
					strlen((char*)line2_str), line2_str)) {
				return 1U;
			}
		}
	}
	return 0U;
}


static uint8_t i2cSmartLCD_Gfx240x128_locator_print(const uint8_t* locatorStr)
{
	uint8_t line_str[16];

	snprintf((char*)line_str, sizeof(line_str) - 1, "%6s", locatorStr);

	if (i2cSmartLCD_Gfx240x128_WriteText(
			(((LCD1_SYSFONT_WIDTH  + 0U) * 34U) - 1U),
			(((LCD1_SYSFONT_HEIGHT + 0U) *  0U) - 0U),
			strlen((char*)line_str), line_str)) {
		return 1U;
	}
	return 0U;
}

void i2cSmartLCD_Gfx240x128_Locked(uint32_t maxUntil, int16_t temp, uint32_t tAcc, int32_t sumDev, float devPsS, uint16_t dacVal, float dacFraction, uint16_t gDOP, uint8_t svPosElevCnt, uint8_t svElevSort[UBLOX_MAX_CH], UbloxNavSvinfo_t* svInfo, const uint8_t* locatorStr)
{
#   define SvCno_max											40U
#   define SvPosElevCnt_max										15U
#	define SvElev_max											90U

	static uint8_t	s_locatorStrLast[16]					= 	{ 0 };
#if defined(PLL_BY_SOFTWARE)
	static uint32_t s_dacPlotNxtTim							=	0UL;
	static uint8_t	s_dacPlotPosOfs							=	0U;
#endif
	static uint8_t 	s_svPosElevCnt_last 					= 	0U;
	static uint8_t 	s_svId_last[SvPosElevCnt_max]			= 	{ 0 };
	static uint8_t 	s_svPosElevCno_last[SvPosElevCnt_max]	= 	{ 0 };
	static uint8_t 	s_svPosElevElev_last[SvPosElevCnt_max]	= 	{ 0 };
	static uint16_t s_svPosElevAzim_last[SvPosElevCnt_max]	= 	{ 0 };
	uint32_t now;

	/* Each time drop a channel two force a redraw */
	static uint8_t	s_svPosElevRefresh_last					= 	0U;
	s_svId_last[s_svPosElevRefresh_last++] = 0xffU;
	s_svPosElevRefresh_last %= SvPosElevCnt_max;


	/* Limit to display max 16 channels to fit onto the display */
	if (svPosElevCnt > SvPosElevCnt_max) {
		svPosElevCnt = SvPosElevCnt_max;
	}

	/* Timeout check */
	now = HAL_GetTick();
	if (now >= maxUntil) {
		return;
	}

	/* Wipe out section not in use*/
	if (s_svPosElevCnt_last > svPosElevCnt) {
		/* Wipe out cleared field entries */
		i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
				svPosElevCnt * 10,								(LCD1_SMART_LCD_SIZE_Y - 1) - (((LCD1_SYSFONT_HEIGHT + 1) * 3U) + (1 + SvCno_max)),
				((s_svPosElevCnt_last - svPosElevCnt) * 10),	(((LCD1_SYSFONT_HEIGHT + 1) * 3U) + (1 + SvCno_max)),
				LCD1_PIXEL_CLR);

		for (uint8_t thisIdx = svPosElevCnt; thisIdx < s_svPosElevCnt_last; ++thisIdx) {
			s_svId_last[thisIdx]			= 	0U;
			s_svPosElevCno_last[thisIdx]	=	0U;
			s_svPosElevElev_last[thisIdx]	= 	0U;
			s_svPosElevAzim_last[thisIdx]	= 	0U;
		}

		/* Store for next time */
		s_svPosElevCnt_last = svPosElevCnt;
	}

	/* Timeout check */
	now = HAL_GetTick();
	if (now >= maxUntil) {
		return;
	}

	/* Print Locator */
	if (strcmp((char*)s_locatorStrLast, (char*)locatorStr)) {
		i2cSmartLCD_Gfx240x128_locator_print(locatorStr);

		/* Write back changed string */
		strncpy((char*)s_locatorStrLast, (char*)locatorStr, sizeof(s_locatorStrLast) - 1);
	}

	/* Timeout check */
	now = HAL_GetTick();
	if (now >= maxUntil) {
		return;
	}

#if defined(PLL_BY_SOFTWARE)
	/* Plot DAC graph element */
	if (now >= s_dacPlotNxtTim)
	{
		/* Calculate for next minute */
		s_dacPlotNxtTim = (now + 60000UL) - ((now + 60000UL) % 60000UL);

		/* Calculate x position*/
		s_dacPlotPosOfs++;
		if (s_dacPlotPosOfs > (DacGfxPos_x_max - DacGfxPos_x_min)) {
			s_dacPlotPosOfs = 0U;
		}

		/* Erase x-slot + 2 more columns right of it*/
		{
			i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
					DacGfxPos_x_min + s_dacPlotPosOfs,
					DacGfxPos_y_top,
					3U,
					(DacGfxPos_y_bot - DacGfxPos_y_top),
					LCD1_PIXEL_CLR);
		}

		/* Plot DAC value */
		{
			float 	ampt_y		= (DacGfxPos_y_bot - DacGfxPos_y_top) / 2.0f;
			uint8_t midPos_y 	= DacGfxPos_y_bot - ampt_y;
			int16_t dacValPlot	= (int16_t)dacVal - (int16_t)I2C_DAC_MCP4725_0_VAL;

			/* Clipping */
			if (dacValPlot >  (int16_t)ampt_y) {
				dacValPlot =  (int16_t)ampt_y;
			}
			if (dacValPlot < -(int16_t)ampt_y) {
				dacValPlot = -(int16_t)ampt_y;
			}

			i2cSmartLCD_Gfx240x128_Draw_Point(
					DacGfxPos_x_min + s_dacPlotPosOfs,
					(uint8_t) (midPos_y - dacValPlot),
					LCD1_PIXEL_SET);
		}
	}
#endif

	/* Timeout check */
	now = HAL_GetTick();
	if (now >= maxUntil) {
		return;
	}


	/* Write OCXO temp & tAcc */
	{
		static int16_t	s_tempLast 			= 0;
		static uint32_t s_tAccLast 			= 0UL;
		static float	s_devPsSLast		= 999.999f;
#if defined(PLL_BY_SOFTWARE)
		static uint8_t  s_dacValLast 		= 0U;
		static float	s_dacFractionLast	= 1.0f;
#endif
		static float	s_gDOPLast			= 0.0f;

		if (temp) {
			/* Update OCXO temperature */
			if (s_tempLast != temp) {
				uint8_t line1_str[32];
				snprintf((char*)line1_str, sizeof(line1_str) - 1, "Temp:   %2d%cC", temp, 0x7e);

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) *  7) - 0U),
						strlen((char*)line1_str), line1_str)) {
					return;
				}
				s_tempLast = temp;
			}
		}

		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		if (gDOP) {
			/* Update ublox NEO gDOP */
			if (s_gDOPLast != gDOP) {
				uint8_t line2_str[32];
				snprintf((char*)line2_str, sizeof(line2_str) - 1, "gDOP:  %2d.%02d", (gDOP / 100), (gDOP % 100));

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) *  8) - 0U),
						strlen((char*)line2_str), line2_str)) {
					return;
				}
				s_gDOPLast = gDOP;
			}
		}

		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		if (tAcc) {
			/* Update ublox NEO tAcc */
			if (s_tAccLast != tAcc) {
				uint8_t line2_str[32];
				snprintf((char*)line2_str, sizeof(line2_str) - 1, "tAcc:  %3ld ns", (tAcc > 999 ?  999 : tAcc));

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) *  9) - 0U),
						strlen((char*)line2_str), line2_str)) {
					return;
				}
				s_tAccLast = tAcc;
			}
		}

		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		if (devPsS) {
			/* Update Software-PLL Long Term Deviation (LTD) value */
			if (s_devPsSLast != devPsS) {
				uint8_t line2_str[32];

				/* Clamping */
				{
					if (devPsS >  99.9999f) {
						devPsS =  99.9999f;
					}
					if (devPsS < -99.9999f) {
						devPsS = -99.9999f;
					}
				}

				snprintf((char*)line2_str, sizeof(line2_str) - 1, "LTD: %+08.4f", devPsS);

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) * 10) - 0U),
						strlen((char*)line2_str), line2_str)) {
					return;
				}
				s_devPsSLast = devPsS;
			}
		}

#if defined(PLL_BY_SOFTWARE)
		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		if (dacVal) {
			/* Update DAC value with fraction component */
			if (s_dacValLast != dacVal) {
				uint8_t line2_str[32];
				snprintf((char*)line2_str, sizeof(line2_str) - 1, "DAC:    %04d", dacVal);

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) * 11) - 0U),
						strlen((char*)line2_str), line2_str)) {
					return;
				}
				s_dacValLast = dacVal;
			}
		}
#endif

#if defined(PLL_BY_SOFTWARE)
		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		if (dacFraction) {
			/* Update DAC value with fraction component */
			if (s_dacFractionLast != dacFraction) {
				uint8_t line2_str[32];
				snprintf((char*)line2_str, sizeof(line2_str) - 1, "Frac: %+7.4f", dacFraction);

				if (i2cSmartLCD_Gfx240x128_WriteText(
						(((LCD1_SYSFONT_WIDTH  + 0) * 27) - 1U),
						(((LCD1_SYSFONT_HEIGHT + 3) * 12) - 0U),
						strlen((char*)line2_str), line2_str)) {
					return;
				}
				s_dacFractionLast = dacFraction;
			}
		}
#endif
	}


	/* Show SV information */
	for (uint8_t svChIdx = 0; svChIdx < svPosElevCnt; ++svChIdx) {
		uint8_t svCh	= svElevSort[svChIdx];
		uint8_t svId 	= svInfo->svid[svCh];
		int8_t  svElev	= svInfo->elev[svCh];
		int16_t svAzim	= svInfo->azim[svCh];
		int8_t  svCno	= svInfo->cno[svCh];

		/* Timeout check */
		now = HAL_GetTick();
		if (now >= maxUntil) {
			return;
		}

		/* Limit signal strength to fit onto the display */
		if (svCno > SvCno_max) {
			svCno = SvCno_max;
		}

		/* Fix for pixel length */
		svElev = (int8_t) ((((LCD1_SYSFONT_HEIGHT + 1L) * 3L) * (2 + svElev)) / SvElev_max);


		/* SV ID slice into each digit */
		uint8_t svIdPos0	= 0x30U + ( svId         / 100U);
		uint8_t svIdPos1	= 0x30U + ((svId % 100U) /  10U);
		uint8_t svIdPos2	= 0x30U + ((svId %  10U)       );

		/* Modify Display for SVs */
		if (	(s_svId_last[svChIdx] 			!= svId) 	||
				(s_svPosElevElev_last[svChIdx] 	!= svElev)	||
				(s_svPosElevAzim_last[svChIdx] 	!= svAzim)	||
				(s_svPosElevCno_last[svChIdx] 	!= svCno)) {
			/* Write back changed values */
			s_svId_last[svChIdx] 			= svId;
			s_svPosElevElev_last[svChIdx] 	= svElev;
			s_svPosElevAzim_last[svChIdx]	= svAzim;
			s_svPosElevCno_last[svChIdx] 	= svCno;

			/* Write SV ID from bottom to top */
			{
				i2cSmartLCD_Gfx240x128_WriteText((2 + svChIdx * 10), LCD1_SMART_LCD_SIZE_Y - ((LCD1_SYSFONT_HEIGHT + 1) * 1U), 1U, &svIdPos2);
				i2cSmartLCD_Gfx240x128_WriteText((2 + svChIdx * 10), LCD1_SMART_LCD_SIZE_Y - ((LCD1_SYSFONT_HEIGHT + 1) * 2U), 1U, &svIdPos1);
				i2cSmartLCD_Gfx240x128_WriteText((2 + svChIdx * 10), LCD1_SMART_LCD_SIZE_Y - ((LCD1_SYSFONT_HEIGHT + 1) * 3U), 1U, &svIdPos0);
			}

			/* Draw bar of elevation - solid bottom */
			{
				i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
						(0 + svChIdx * 10), 	(LCD1_SMART_LCD_SIZE_Y - 1) 		- (1 + svElev),
						1, 						(1 + svElev),
						LCD1_PIXEL_SET);

				/* Draw bar of elevation - cleared top */
				i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
						(0 + svChIdx * 10), 	(LCD1_SMART_LCD_SIZE_Y - 1) 		- ((LCD1_SYSFONT_HEIGHT + 1L) * 3L),
						1, 						((LCD1_SYSFONT_HEIGHT + 1L) * 3L) 	- (2 + svElev),
						LCD1_PIXEL_CLR);
			}

			/* Draw bar of signal strength 'CNO' - solid bottom */
			{
				i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
						(1 + svChIdx * 10), 	LCD1_SMART_LCD_SIZE_Y - ((LCD1_SYSFONT_HEIGHT + 1) * 3U) - (1 + svCno)		- 2,
						8, 						(1 + svCno),
						LCD1_PIXEL_SET);

				/* Draw bar of signal strength 'CNO' - cleared top */
				i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
						(1 + svChIdx * 10), 	LCD1_SMART_LCD_SIZE_Y - ((LCD1_SYSFONT_HEIGHT + 1) * 3U) - (1 + SvCno_max)	- 3,
						8, 						(1 + SvCno_max - svCno),
						LCD1_PIXEL_CLR);
			}

			/* SV azimuth */
			{
				const float 	ArrowSize = 4.49f;
				const float 	pntOrig_x = 4.5f + (svChIdx * 10U);
				const float 	pntOrig_y = 56.0f;

				uint8_t pntWipeTL_x	= (uint8_t) (pntOrig_x - 4.5f);
				uint8_t pntWipeTL_y	= (uint8_t) (pntOrig_y - 4.5f);
				uint8_t lenWipe_xy	= 10U;

				uint8_t pntFront_x	= (uint8_t) (pntOrig_x + ArrowSize * sin(M_PI *  svAzim			/ 180.0f));
				uint8_t pntFront_y	= (uint8_t) (pntOrig_y + ArrowSize * cos(M_PI *  svAzim			/ 180.0f));

				uint8_t pntLeft_x	= (uint8_t) (pntOrig_x + ArrowSize * sin(M_PI * (svAzim - 155)	/ 180.0f));
				uint8_t pntLeft_y	= (uint8_t) (pntOrig_y + ArrowSize * cos(M_PI * (svAzim - 155)	/ 180.0f));

				uint8_t pntRight_x	= (uint8_t) (pntOrig_x + ArrowSize * sin(M_PI * (svAzim + 155)	/ 180.0f));
				uint8_t pntRight_y	= (uint8_t) (pntOrig_y + ArrowSize * cos(M_PI * (svAzim + 155)	/ 180.0f));

				/* Draw bar of signal strength 'CNO' - cleared top */
				i2cSmartLCD_Gfx240x128_Draw_Rect_filled(
						pntWipeTL_x, 	pntWipeTL_y,
						lenWipe_xy, 	lenWipe_xy,
						LCD1_PIXEL_CLR);

				i2cSmartLCD_Gfx240x128_Draw_SetStartPos(
						pntFront_x, 	pntFront_y);

				i2cSmartLCD_Gfx240x128_Draw_Line_to(
						pntLeft_x, 		pntLeft_y,
						LCD1_PIXEL_SET);

#if 0
				i2cSmartLCD_Gfx240x128_Draw_Line_to(
						pntOrig_x, 		pntOrig_y,
						LCD1_PIXEL_SET);
#endif

				i2cSmartLCD_Gfx240x128_Draw_Line_to(
						pntRight_x, 	pntRight_y,
						LCD1_PIXEL_SET);

				i2cSmartLCD_Gfx240x128_Draw_Line_to(
						pntFront_x, 	pntFront_y,
						LCD1_PIXEL_SET);
			}
		}
	}

#   undef SvCno_max
#   undef SvPosElevCnt_max
#	undef SvElev_max
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
