/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */

/* I2C chip addresses */
#define I2C_CHIP_ADDR_GPIO_MCP23017_0								0x20
#define I2C_CHIP_ADDR_DAC_MCP4725_0									0x60

#define I2C_CHIP_ADDR_LCD_0											( I2C_CHIP_ADDR_GPIO_MCP23017_0 )

#define I2C_CHIP_ADDR_LCD_1											0x22
#define I2C_CHIP_ADDR_LCD_DIGPOT_1									0x2f


/* Bitfield for devices list */
#define I2C_DEVICE_DAC_MCP4725_0									0x00000001
#define I2C_DEVICE_LCD_0											0x00000100
#define I2C_DEVICE_LCD_1											0x00001000
#define I2C_DEVICE_LCD_DIGPOT_1										0x00002000


/* DAC default value */
#define I2C_DAC_MCP4725_0_VAL										1742


/* MCP23017 address map */
#define I2C_MCP23017_ADDR_IODIR_A									0x00
#define I2C_MCP23017_ADDR_IODIR_B									0x01
#define I2C_MCP23017_ADDR_IPOL_A									0x02
#define I2C_MCP23017_ADDR_IPOL_B									0x03
#define I2C_MCP23017_ADDR_GPINTEN_A									0x04
#define I2C_MCP23017_ADDR_GPINTEN_B									0x05
#define I2C_MCP23017_ADDR_DEFVAL_A									0x06
#define I2C_MCP23017_ADDR_DEFVAL_B									0x07
#define I2C_MCP23017_ADDR_INTCON_A									0x08
#define I2C_MCP23017_ADDR_INTCON_B									0x09
#define I2C_MCP23017_ADDR_IOCON_A									0x0a
#define I2C_MCP23017_ADDR_IOCON_B									0x0b
#define I2C_MCP23017_ADDR_GPPU_A									0x0c
#define I2C_MCP23017_ADDR_GPPU_B									0x0d
#define I2C_MCP23017_ADDR_INTF_A									0x0e
#define I2C_MCP23017_ADDR_INTF_B									0x0f
#define I2C_MCP23017_ADDR_INTCAP_A									0x10
#define I2C_MCP23017_ADDR_INTCAP_B									0x11
#define I2C_MCP23017_ADDR_GPIO_A									0x12
#define I2C_MCP23017_ADDR_GPIO_B									0x13
#define I2C_MCP23017_ADDR_OLAT_A									0x14
#define I2C_MCP23017_ADDR_OLAT_B									0x15


/* LCD 16x2 mapping to MCP23017 */
#define I2C_MCP23017_LCD16X2_GPA_0_DB0_OI							0x01
#define I2C_MCP23017_LCD16X2_GPA_1_DB1_OI							0x02
#define I2C_MCP23017_LCD16X2_GPA_2_DB2_OI							0x04
#define I2C_MCP23017_LCD16X2_GPA_3_DB3_OI							0x08
#define I2C_MCP23017_LCD16X2_GPA_4_DB4_OI							0x10
#define I2C_MCP23017_LCD16X2_GPA_5_DB5_OI							0x20
#define I2C_MCP23017_LCD16X2_GPA_6_DB6_OI							0x40
#define I2C_MCP23017_LCD16X2_GPA_7_DB7_OI							0x80
#define I2C_MCP23017_LCD16X2_GPB_0_E_O								0x01
#define I2C_MCP23017_LCD16X2_GPB_1_RW_O								0x02
#define I2C_MCP23017_LCD16X2_GPB_2_RS_O								0x04
#define I2C_MCP23017_LCD16X2_GPB_3_LED_O							0x08
#define I2C_MCP23017_LCD16X2_GPB_4_NC_I								0x10
#define I2C_MCP23017_LCD16X2_GPB_5_NC_I								0x20
#define I2C_MCP23017_LCD16X2_GPB_6_NC_I								0x40
#define I2C_MCP23017_LCD16X2_GPB_7_NC_I								0x80


/* LCD Gfx 240x128 via Smart-LCD */
// The unique commands of the Smart-LCD device for all modes
#define LCD1_SMART_LCD_CMD_NOOP										0x00
#define LCD1_SMART_LCD_CMD_GET_VER									0x01
#define LCD1_SMART_LCD_CMD_SET_MODE									0x02
#define LCD1_SMART_LCD_CMD_GET_STATE								0x03

// Mode 0x10 commands (Smart-LCD draw box)
#define LCD1_SMART_LCD_CMD_RESET									0x10
#define LCD1_SMART_LCD_CMD_CLS										0x11
#define LCD1_SMART_LCD_CMD_SET_PIXEL_TYPE							0x14
#define LCD1_SMART_LCD_CMD_SET_POS_X_Y								0x20
#define LCD1_SMART_LCD_CMD_WRITE									0x30
#define LCD1_SMART_LCD_CMD_DRAW_LINE								0x32
#define LCD1_SMART_LCD_CMD_DRAW_RECT								0x34
#define LCD1_SMART_LCD_CMD_DRAW_FILLED_RECT							0x36
#define LCD1_SMART_LCD_CMD_DRAW_CIRC								0x38
#define LCD1_SMART_LCD_CMD_DRAW_FILLED_CIRC							0x3A
//#define LCD1_SMART_LCD_CMD_GET_ROTBUT								0x60
//#define LCD1_SMART_LCD_CMD_GET_LIGHT								0x64
#define LCD1_SMART_LCD_CMD_GET_TEMP									0x65
//#define LCD1_SMART_LCD_CMD_SET_LEDS								0x70
//#define LCD1_SMART_LCD_CMD_SET_BEEP								0x71
#define LCD1_SMART_LCD_CMD_SET_BACKLIGHT							0x74
#define LCD1_SMART_LCD_CMD_SET_CONTRAST								0x75

#define LCD1_SMART_LCD_STR_MAXLEN_BUG								7

#define LCD1_SYSFONT_WIDTH											6
#define LCD1_SYSFONT_HEIGHT											7

enum LCD1_SMART_LCD_MODE__ENUM {
	LCD1_SMART_LCD_MODE_UNIQUE										= 0x00,
	LCD1_SMART_LCD_MODE_SMARTLCD									= 0x10,
	LCD1_SMART_LCD_MODE_REFOSC										= 0x20,
};

/** Pixel operations */
enum LCD1_gfx_mono_color {
	/** Pixel is cleared */
	LCD1_PIXEL_CLR 													= 0,

	/** Pixel is set on screen (OR) */
	LCD1_PIXEL_SET 													= 1,

	/** Pixel is XORed */
	LCD1_PIXEL_XOR 													= 2,
};


/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
