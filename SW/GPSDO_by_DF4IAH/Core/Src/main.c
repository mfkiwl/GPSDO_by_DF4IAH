/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Please adjust the f_comp value here */
  #define F_COMP_HZ 1
//#define F_COMP_HZ 1000
//#define F_COMP_HZ 10000
//#define F_COMP_HZ 100000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern GPIO_PinState 	gpioLockedLED;
extern GPIO_PinState	gpioHoRelayOut;

extern uint8_t 			owDevices[ONEWIRE_DEVICES_MAX][8];
extern uint8_t 			owDevicesCount;
extern int16_t 			owDs18b20_Temp[ONEWIRE_DEVICES_MAX];
extern float 			owDs18b20_Temp_f[ONEWIRE_DEVICES_MAX];

extern float 			tim2Ch2_ppm;
extern int32_t 			timTicksDiff;
extern uint32_t 		timTicksEvt;
extern int32_t 			timTicksSumDev;

extern uint16_t			adcCh9_val;
extern uint16_t			adcCh10_val;
extern uint16_t			adcCh16_val;
extern uint16_t 		adcVrefint_val;
extern const float 		VREFINT_CAL;
extern float 			adc_VDDA;
extern float 			adcCh9_volts;
extern float 			adcCh10_volts;
extern float 			adcCh16_volts;

extern uint8_t  		i2cDacModeLast;
extern uint8_t  		i2cDacMode;
extern uint16_t 		i2cDacValLast;
extern uint16_t 		i2cDacVal;
extern float 			i2cDacFraction;

extern uint32_t			ubloxRespBf;
extern UbloxNavDop_t	ubloxNavDop;
extern UbloxNavClock_t	ubloxNavClock;
extern UbloxNavSvinfo_t	ubloxNavSvinfo;
extern uint32_t			ubloxTimeAcc;


uint8_t					gMelevSortTgtPosElevCnt					= 0U;
uint8_t 				gMelevSortTgtCh[UBLOX_MAX_CH]			= { 0 };

uint32_t 				gMtempWaitUntil[ONEWIRE_DEVICES_MAX]	= { 0 };
uint8_t  				gMowSensorIdx 							= 0;

float					gMdevPsS								= 0.0f;

uint32_t 				gMLoop_Tim2_00_ubloxResp				= 0UL;
uint32_t 				gMLoop_Tim2_01_tempResp					= 0UL;
uint32_t 				gMLoop_Tim2_02_adcResp					= 0UL;
uint32_t 				gMLoop_Tim2_03_deviationCalc			= 0UL;
uint32_t 				gMLoop_Tim2_04_pllCalc					= 0UL;
uint32_t				gMLoop_Tim2_05_svSort					= 0UL;
uint32_t 				gMLoop_Tim2_10_ubloxReq					= 0UL;
uint32_t 				gMLoop_Tim2_11_tempReq					= 0UL;
uint32_t 				gMLoop_Tim2_20_hoRelayDacOut			= 0UL;
uint32_t 				gMLoop_Tim2_21_ubloxPrint				= 0UL;
uint32_t 				gMLoop_Tim2_22_deviationPrint			= 0UL;
uint32_t 				gMLoop_Tim2_23_pllPrint					= 0UL;
uint32_t 				gMLoop_Tim2_24_adcPrint					= 0UL;
uint32_t 				gMLoop_Tim2_25_tempPrint				= 0UL;
uint32_t 				gMLoop_Tim2_26_lcd16x2Print				= 0UL;
uint32_t 				gMLoop_Tim2_27_lcd240x128Print			= 0UL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* GPIO & Onewire */
extern uint8_t onewire_CRC8_calc(uint8_t* fields, uint8_t len);
extern GPIO_PinState onewireMasterCheck_presence(void);
extern uint8_t onewireMasterTree_search(uint8_t searchAlarms, uint8_t devicesMax, uint8_t onewireDevices[][8]);
extern void onewireDS18B20_readROMcode(uint8_t* romCode);
extern void onewireDS18B20_setAdcWidth(uint8_t width, int8_t tempAlarmHi, int8_t tempAlarmLo, uint8_t* romCode);
extern uint32_t onewireDS18B20_tempReq(uint8_t* romCode);
extern int16_t onewireDS18B20_tempRead(uint32_t waitUntil, uint8_t* romCode);

/* UART */
extern void ubloxUartSpeedFast(void);
extern void ubloxFlush(void);
extern uint8_t ubloxSetFrequency(uint16_t frequency);
extern void ubloxMsgsTurnOff(void);
extern void ublox_NavDop_req(UbloxNavDop_t* dop);
extern void ublox_NavClock_req(UbloxNavClock_t* ubloxNavClock);
extern void ublox_NavSvinfo_req(UbloxNavSvinfo_t* ubloxNavSvinfo);
extern uint32_t ublox_All_resp(void);
extern void ublox_NavDop_print(UbloxNavDop_t* ubloxNavDop);
extern void ublox_NavClock_print(UbloxNavClock_t* ubloxNavClock);
extern void ublox_NavSvinfo_print(UbloxNavSvinfo_t* ubloxNavSvinfo);

/* I2C */
extern uint8_t i2cBusGetDeviceList(uint32_t* i2cDevicesBF);
extern uint8_t i2cDeviceDacMcp4725_set(uint8_t chipAddr, uint8_t pdMode, uint16_t dac_12b);
extern uint8_t i2cMCP23017_Lcd16x2_ClrScr(void);
extern uint8_t i2cMCP23017_Lcd16x2_SetAddr(uint8_t row, uint8_t col);
extern uint8_t i2cMCP23017_Lcd16x2_WriteStr(uint8_t* str, uint8_t len);
extern void i2cMCP23017_Lcd16x2_Welcome(void);
extern void i2cMCP23017_Lcd16x2_OCXO_HeatingUp(int16_t temp, uint32_t tAcc);
extern void i2cMCP23017_Lcd16x2_Locked(int16_t temp, uint32_t tAcc, int32_t sumDev);

extern uint8_t i2cSmartLCD_Gfx240x128_Template(void);
extern uint8_t i2cSmartLCD_Gfx240x128_Welcome(void);
extern uint8_t i2cSmartLCD_Gfx240x128_OCXO_HeatingUp(int16_t temp, uint32_t tAcc);
extern uint8_t i2cSmartLCD_Gfx240x128_Locked_Template(void);
extern void i2cSmartLCD_Gfx240x128_Locked(int16_t temp, uint32_t tAcc, int32_t sumDev, float devPsS, uint16_t dacVal, float dacFraction, uint16_t gDOP, uint8_t svPosElevCnt, uint8_t svElevSort[UBLOX_MAX_CH], UbloxNavSvinfo_t* svInfo);

/* Timer */
extern void tim_start(void);
extern void tim_capture_ch2(void);
extern uint32_t tim_get_timeStamp(TIM_HandleTypeDef *htim);

/* ADC */
extern void adc_init(void);
extern void adc_start(void);
extern void adc_stop(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void uDelay(uint16_t uDelay)
{
	uint32_t uCnt = (uDelay * 66UL) / 10;

	for (; uCnt; --uCnt) {
	}
}

void memclear(uint8_t* ary, uint16_t len)
{
	while (len--) {
		*(ary++) = 0U;
	}
}


void mainLoop_PLL_calc(void)
{
#if defined(PLL_BY_SOFTWARE)

  /* Software PLL logics */
  {
	  /* Default value for everything is okay */
	  gpioLockedLED = GPIO_PIN_SET;

	  /* DAC output mode */
	  i2cDacMode = 0b00;

	  /* Do not tune when primary temp sensor is out of temp range of OCXO */
	  if (owDevicesCount) {
		  if (owDs18b20_Temp_f[0] < ONEWIRE_DS18B20_ALARM_LO) {
			  /* Keep sum-up registers cleared */
			  timTicksDiff 	= 0L;
			  timTicksEvt	= 1UL;

			  /* Not locked in */
			  gpioLockedLED = GPIO_PIN_RESET;
		  }
	  }

	  /* Check if ubox NEO is locked in */
	  if (ubloxTimeAcc >= 250UL) {  // when worse than that stop time tracking
		  /* Keep sum-up registers cleared */
		  timTicksDiff 	= 0L;
		  timTicksEvt	= 1UL;

		  /* Not locked in */
		  gpioLockedLED = GPIO_PIN_RESET;
	  }

	  if (timTicksEvt > 12) {
		  /* Fractions accounting */
		  if (0 < timTicksDiff) {
			  if (tim2Ch2_ppm > 0.0f) {
				  i2cDacFraction -= timTicksDiff /  10000.0f;
			  } else {
				  i2cDacFraction -= timTicksDiff / 100000.0f;
			  }
		  }
		  else if (timTicksDiff < 0) {
			  if (tim2Ch2_ppm < 0.0f) {
				  i2cDacFraction -= timTicksDiff /  10000.0f;
			  } else {
				  i2cDacFraction -= timTicksDiff / 100000.0f;
			  }
		  }

		  /* Fractions to DAC value */
		  if (i2cDacFraction > +0.501f) {
			  if (i2cDacVal < 2046) {
				  ++i2cDacVal;
			  }

			  i2cDacFraction -= 1.0f;

			  if (i2cDacFraction > +0.501f) {
				  i2cDacFraction = +0.5f;

				  /* Not locked in */
				  gpioLockedLED = GPIO_PIN_RESET;
			  }
		  }
		  else if (i2cDacFraction < -0.501f) {
			  if (i2cDacVal > 0) {
				  --i2cDacVal;
			  }

			  i2cDacFraction += 1.0f;

			  if (i2cDacFraction < -0.501f) {
				  i2cDacFraction = -0.5f;

				  /* Not locked in */
				  gpioLockedLED = GPIO_PIN_RESET;
			  }
		  }
	  }  // if (timTicksEvt > 12)
	  else {
		  /* To early */
		  gpioLockedLED = GPIO_PIN_RESET;
	  }
  }

#else

  /* OCXO controlled by hardware PLL */
  {
	  /* DAC high impedance mode */
	  i2cDacMode	= 0b11;
	  i2cDacVal		= I2C_DAC_MCP4725_0_VAL;
  }

#endif
}

void mainLoop_PLL_print(void)
{
#if defined(LOGGING)

# if defined(PLL_BY_SOFTWARE)

	  /* Show PLL Lock state */
	  {
		  uint8_t msg[64];
		  int len;

		  len = snprintf(((char*) msg), sizeof(msg), "\r\n*** Software-PLL: DAC value = %04u - fractions = %+8.5f\r\n", i2cDacVal, i2cDacFraction);
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }

# else

	  /* Show PLL Lock state */
	  {
		  uint8_t msg[64];
		  int len;

		  len = snprintf(((char*) msg), sizeof(msg), "\r\n*** PLL Lock state = %d\r\n", HAL_GPIO_ReadPin(D10_PLL_LCKD_GPIO_I_GPIO_Port, D10_PLL_LCKD_GPIO_I_Pin));
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }

# endif

#endif
}

void mainLoop_ublox_requests(void)
{
	/* Request all needed messages and assign target data structures */
#if defined(LOGGING)
	{
		uint8_t msg[] = "\r\n";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	}
#endif

	ublox_NavClock_req(&ubloxNavClock);
	ublox_NavDop_req(&ubloxNavDop);
	ublox_NavSvinfo_req(&ubloxNavSvinfo);
}

void mainLoop_ublox_waitForResponses(void)
{
	/* Blocks until new second starts */
	ubloxRespBf = ublox_All_resp();

	/* ublox data is assigned to customers */
	ubloxTimeAcc = ubloxNavClock.tAcc;
}

uint8_t mainLoop_ublox_svinfo_sort(uint8_t elevSortTgtCh[UBLOX_MAX_CH])
{
	uint8_t elevSortSrcCh[UBLOX_MAX_CH];
	uint8_t srcSize = UBLOX_MAX_CH;
	uint8_t posElevCnt = 0U;

	/* Prepare src ballot box for all channels */
	for (uint8_t srcIdx = 0U; srcIdx < UBLOX_MAX_CH; ++srcIdx) {
		elevSortSrcCh[srcIdx] = srcIdx;
		elevSortTgtCh[srcIdx] = 0xffU;  // Signal for 'entry not valid'
	}

	/* Find each target element */
	for (uint8_t tgtIdx = 0U; tgtIdx < UBLOX_MAX_CH; ++tgtIdx) {
		uint8_t elevMaxCh 	= 0xffU;
		int8_t  elevMaxVal 	= -127;
		uint8_t srcIdxHit	= 0U;

		for (uint8_t srcIdx = 0U; srcIdx < srcSize; ++srcIdx) {
			uint8_t elevCh	= elevSortSrcCh[srcIdx];
			int8_t  elevVal	= ubloxNavSvinfo.elev[elevCh];

			if (elevVal > elevMaxVal) {
				srcIdxHit	= srcIdx;
				elevMaxCh 	= elevCh;
				elevMaxVal 	= elevVal;
			}
		}

		/* Count SVs with positive elevation */
		if (elevMaxVal > 0) {
			++posElevCnt;
		}

		/* Fill target */
		elevSortTgtCh[tgtIdx] = elevMaxCh;

		/* Shrink source ballot box by one entry */
		--srcSize;
		for (uint8_t srcIdx = srcIdxHit; srcIdx < srcSize; ++srcIdx) {
			elevSortSrcCh[srcIdx] = elevSortSrcCh[srcIdx + 1];
		}
		elevSortSrcCh[srcSize] = 0xffU;
	}

	return posElevCnt;
}

void mainLoop_ublox_print(void)
{
#if defined(LOGGING)
	/* Print all data, that was received */
	if (ubloxRespBf & USART_UBLOX_RESP_BF_NAV_DOP) {
# if 1
		ublox_NavDop_print(&ubloxNavDop);
# endif
	}

	if (ubloxRespBf & USART_UBLOX_RESP_BF_NAV_CLOCK) {
# if 1
		ublox_NavClock_print(&ubloxNavClock);
# endif
	}

	if (ubloxRespBf & USART_UBLOX_RESP_BF_NAV_SVINFO) {
# if 0
		ublox_NavSvinfo_print(&ubloxNavSvinfo);
# endif
	}
#endif
}

void mainLoop_ow_temp_waitForResponse(uint32_t tempWaitUntil, uint8_t owDeviceIdx)
{
	/* Onewire handling */
	owDs18b20_Temp[owDeviceIdx]		= onewireDS18B20_tempRead(tempWaitUntil, owDevices[owDeviceIdx]);
	owDs18b20_Temp_f[owDeviceIdx]	= owDs18b20_Temp[owDeviceIdx] / 16.0f;
}

void mainLoop_ow_temp_print(void)
{
#if defined(LOGGING)
	uint8_t msg[64];

	for (uint8_t idx = 0U; idx < owDevicesCount; ++idx) {
		int16_t  t_int			= (owDs18b20_Temp[idx] >> 4);
		uint16_t t_frac			= (owDs18b20_Temp[idx] & 0xfU);
		if (t_int < 0) {
			t_frac = ~t_frac;
			++t_frac;
			t_frac %= 1000U;
		}

		uint16_t t_fv1000	= 0U;
		if (t_frac & 0b1000) {
			t_fv1000 += 500U;
		}
		if (t_frac & 0b0100) {
			t_fv1000 += 250U;
		}
		if (t_frac & 0b0010) {
			t_fv1000 += 125U;
		}
		if (t_frac & 0b0001) {
			t_fv1000 +=  62U;
		}

		int len = snprintf(((char*) msg), sizeof(msg), "\r\n*** Temperature sensor %d: %+02d,%02u degC\r\n", idx, t_int, (t_fv1000 + 5) / 10);
		HAL_UART_Transmit(&huart2, msg, len, 25);
	}
#endif
}

void mainLoop_ow_tempAlarm_print(void)
{
#if defined(LOGGING)
	uint8_t onewireAlarms[2][8] = { 0 };
	uint8_t onewireAlarmsCount = onewireMasterTree_search(1U, owDevicesCount, onewireAlarms);

	if (onewireAlarmsCount) {
		uint8_t msg[64];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** Temperature ALARM: %d sensor(s) out of limits.\r\n", onewireAlarmsCount);
		HAL_UART_Transmit(&huart2, msg, len, 25);
	}
#endif
}


void mainLoop_adc_volts_resp(void)
{
	adc_VDDA 		= (3.0f * VREFINT_CAL) / adcVrefint_val;  // p. 448f
	adcCh9_volts	= ( adcCh9_val * adc_VDDA / 65536.0f);
	adcCh10_volts	= (adcCh10_val * adc_VDDA / 65536.0f);
	adcCh16_volts	= (adcCh16_val * adc_VDDA / 65536.0f);
}

void mainLoop_adc_volts_print(void)
{
#if defined(LOGGING)
	/* Show ADC values */
	uint8_t msg[128];
	int len;

	len = snprintf(((char*) msg), sizeof(msg), "\r\n*** ADC values:\r\n");
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf(((char*) msg), sizeof(msg), "  * VDDA                 = %1.4f V\r\n"
											   "  *\r\n",
		  adc_VDDA);
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf(((char*) msg), sizeof(msg), "  * (Ch09) V_OCXO        = 0x%04x = %05d  -->  V_OCXO   = %1.3f V\r\n",
		  adcCh9_val,
		  adcCh9_val,
		  adcCh9_volts);
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf(((char*) msg), sizeof(msg), "  * (Ch10) V_HOLD        = 0x%04x = %05d  -->  V_HOLD   = %1.3f V\r\n",
		  adcCh10_val,
		  adcCh10_val,
		  adcCh10_volts);
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf(((char*) msg), sizeof(msg), "  * (Ch16) V_DCF77_DEMOD = 0x%04x = %05d  -->  V_DCFAMP = %1.3f V\r\n",
		  adcCh16_val,
		  adcCh16_val,
		  adcCh16_volts);
	HAL_UART_Transmit(&huart2, msg, len, 25);
#endif
}


void mainLoop_tim_deviation_resp(void)
{
	/* Export accumulated deviation */
	if (timTicksDiff >= 0L) {
		timTicksSumDev = (int32_t) (+0.5f + timTicksDiff * 100.0f / (6.0f * timTicksEvt));
	}
	else {
		timTicksSumDev = (int32_t) (-0.5f + timTicksDiff * 100.0f / (6.0f * timTicksEvt));
	}

	gMdevPsS = timTicksDiff * 100.0f / (6.0f * timTicksEvt);
}

void mainLoop_tim_deviation_print(void)
{
#if defined(LOGGING)
	/* Get last time deviation in PPMs */
	{
		uint32_t ticks_d, ticks_f;
		uint8_t chr;
		uint8_t msg[128];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** OCXO deviation against GPS PPS pulses:\r\n");
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  *%+12.2f ps/s\r\n", 1e6 * tim2Ch2_ppm);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  *%011.2f Hz\r\n", (110e6 + tim2Ch2_ppm * 10.0f));
		msg[3] = ' ';
		HAL_UART_Transmit(&huart2, msg, len, 25);

		if (timTicksDiff >= 0) {
		  ticks_d = (uint32_t)timTicksDiff / 10;
		  ticks_f = (uint32_t)timTicksDiff % 10;
		  chr = '+';
		} else {
		  ticks_d = (uint32_t)(-timTicksDiff) / 10;
		  ticks_f = (uint32_t)(-timTicksDiff) % 10;
		  chr = '-';
		}
		len = snprintf(((char*) msg), sizeof(msg), "  * ?%lu.%01lu accumulated deviation ticks  during  runtime = %lu sec  (%.2f ps/s).\r\n",
			  ticks_d, ticks_f,
			  timTicksEvt,
			  gMdevPsS);
		msg[4] = chr;
		HAL_UART_Transmit(&huart2, msg, len, 25);
	}
#endif
}

void mainLoop_dbg_tim2_ts_print(void)
{
#if defined(LOGGING)
# if 1
	/* Print all LOOP times */
	{
		uint8_t msg[128];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** LOOP TIMES:\r\n");
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 00_ubloxResp        %8ld us   @ %07ld ticks.\r\n", 0UL, gMLoop_Tim2_00_ubloxResp);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 01_tempResp         %8ld us.\r\n", (gMLoop_Tim2_01_tempResp 		- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 02_adcResp          %8ld us.\r\n", (gMLoop_Tim2_02_adcResp 			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 03_deviationCalc    %8ld us.\r\n", (gMLoop_Tim2_03_deviationCalc	- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 04_pllCalc          %8ld us.\r\n", (gMLoop_Tim2_04_pllCalc			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 05_svSort           %8ld us.\r\n", (gMLoop_Tim2_05_svSort			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 10_ubloxReq         %8ld us.\r\n", (gMLoop_Tim2_10_ubloxReq			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 11_tempReq          %8ld us.\r\n", (gMLoop_Tim2_11_tempReq			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 20_hoRelayDacOut    %8ld us.\r\n", (gMLoop_Tim2_20_hoRelayDacOut	- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 21_ubloxPrint       %8ld us.\r\n", (gMLoop_Tim2_21_ubloxPrint		- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 22_deviationPrint   %8ld us.\r\n", (gMLoop_Tim2_22_deviationPrint	- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 23_pllPrint         %8ld us.\r\n", (gMLoop_Tim2_23_pllPrint			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 24_adcPrint         %8ld us.\r\n", (gMLoop_Tim2_24_adcPrint			- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 25_tempPrint        %8ld us.\r\n", (gMLoop_Tim2_25_tempPrint		- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 26_lcd16x2Print     %8ld us.\r\n", (gMLoop_Tim2_26_lcd16x2Print		- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 27_lcd240x128Print  %8ld us.\r\n", (gMLoop_Tim2_27_lcd240x128Print	- gMLoop_Tim2_00_ubloxResp) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "***\r\n\r\n");
		HAL_UART_Transmit(&huart2, msg, len, 25);
	}
# endif
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  for (uint32_t cnt = 0x000c0000UL; cnt; --cnt) {
	  /* Delay for two seconds to get internal & external
	   * oscillators to come up and voltages to stabilize
	   */
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

#if 0
  static uint8_t enableMe = 0;
  while (!enableMe) {
  }
#endif

#if 1
#endif


#if defined(LOGGING)
  /* UART: DEBUGGING terminal */
  {
	uint8_t msg[] = "\r\n\r\n**************************\r\n*** sGPSDO a la DF4IAH ***\r\n**************************\r\n\r\n";
	HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
  }
#endif


  /* I2C: Get list of all I2C devices */
  uint32_t i2cDevicesBF = 0UL;
  uint8_t i2cBusCnt = i2cBusGetDeviceList(&i2cDevicesBF);  (void) i2cBusCnt;

  /* I2C: DAC */
  if (i2cDevicesBF & I2C_DEVICE_DAC_MCP4725_0) {
	  /* Switch DAC to high impedance (500kR) mode */
	  i2cDacModeLast	= 0b11;
	  i2cDacMode		= 0b11;
	  i2cDacValLast		= I2C_DAC_MCP4725_0_VAL;
	  i2cDacVal 		= I2C_DAC_MCP4725_0_VAL;

	  i2cDeviceDacMcp4725_set(0, i2cDacMode, i2cDacVal);
  }

  /* I2C: LCD 16x2 */
  if (i2cDevicesBF & I2C_DEVICE_LCD_0) {
	  /* Init and welcome string */
	  i2cMCP23017_Lcd16x2_Welcome();
  }

  /* I2C: LCD Gfx 240x128 */
  if (i2cDevicesBF & I2C_DEVICE_LCD_1) {
	  i2cSmartLCD_Gfx240x128_Welcome();
  }

#if defined(LOGGING)
  {
	uint8_t msg[32] = { 0 };
	int len;

	len = snprintf((char*)msg, sizeof(msg) - 1, "*** I2C bus scan:\r\n");
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf((char*)msg, sizeof(msg) - 1, "  * %d device(s) found.\r\n", i2cBusCnt);
	HAL_UART_Transmit(&huart2, msg, len, 25);

	len = snprintf((char*)msg, sizeof(msg) - 1, "  * bitfield = 0x%08lx\r\n\r\n", i2cDevicesBF);
	HAL_UART_Transmit(&huart2, msg, len, 25);
  }
#endif


  /* GPIO: Acoustic boot check */
  {
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);
	  HAL_Delay(250UL);
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_SET);
	  HAL_Delay(250UL);
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);
  }


  /* GPIO: Turn off Locked LED */
  gpioLockedLED = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, gpioLockedLED);


  /* NEO: Turn NMEA messages off */
  ubloxMsgsTurnOff();

  /* NEO: Change baudrate of the u-blox */
  ubloxUartSpeedFast();

  /* Default setting for hold relay */
  gpioHoRelayOut = GPIO_PIN_RESET;

  /* NEO: Change 1PPS pulse frequency we need */
  uint8_t ubloxRetries = 3U;
  do {
	  if (ubloxSetFrequency(F_COMP_HZ)) {
#if defined(LOGGING)
		  {
			  uint8_t msg[] = "*** u-blox TimePulse has not changed - keeping in Hold mode. - trying again ...\r\n";
			  HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		  }
#endif
		  if (!(--ubloxRetries)) {
			  /* RESET */
			  volatile uint32_t* AIRCR = (uint32_t*) 0xe000ed0cUL;
			  uint32_t aircr_val = 0x05fa0304UL;
			  *AIRCR = aircr_val;
		  }

		  HAL_Delay(1300);
	  }
	  else {
#if defined(LOGGING)
		  {
			  uint8_t msg[] = "*** u-blox TimePulse modification has worked - switching from Hold to PLL mode.\r\n";
			  HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
		  }
#endif

#if defined(PLL_BY_SOFTWARE)
		  /* Switching to Hold mode */
		  gpioHoRelayOut = GPIO_PIN_SET;
#endif
	  }
	  break;
  } while (1);

  /* Update hold relay */
  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, gpioHoRelayOut);


  /* ADC: Prepare */
  adc_init();


  /* TIMER: Prepare the Time capture for CH2 (GPS PPS) & CH4 (DCF77 Phase) */
  tim_start();

  if (i2cDevicesBF & I2C_DEVICE_LCD_0) {
	  /* Inform about firing up the OCXO and GPS */
	  i2cMCP23017_Lcd16x2_OCXO_HeatingUp(0U, 0U);
  }

  if (i2cDevicesBF & I2C_DEVICE_LCD_1) {
	  /* Inform about firing up the OCXO and GPS */
	  i2cSmartLCD_Gfx240x128_OCXO_HeatingUp(0U, 0U);
  }


  /* GPIO / ONEWIRE: Init the DS18B20 temperature sensor(s)  */
  {
	  memclear((uint8_t*) owDevices, sizeof(owDevices));
	  owDevicesCount = onewireMasterTree_search(0U, ONEWIRE_DEVICES_MAX, owDevices);

#if defined(LOGGING)
	  {
		  uint8_t msg[64];
		  int len;

		  len = snprintf(((char*) msg), sizeof(msg), "\r\n*** 1-wire Temperature sensors found: %d\r\n", owDevicesCount);
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }
#endif

	  /* Set configuration and temp alarm limits */
	  for (uint8_t idx = 0; idx < owDevicesCount; ++idx) {
#if   defined(ONEWIRE_DS18B20_ADC_12B)
		  onewireDS18B20_setAdcWidth(12, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, owDevices[idx]);
#elif defined(ONEWIRE_DS18B20_ADC_11B)
		  onewireDS18B20_setAdcWidth(11, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, owDevices[idx]);
#elif defined(ONEWIRE_DS18B20_ADC_10B)
		  onewireDS18B20_setAdcWidth(10, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, owDevices[idx]);
#elif defined(ONEWIRE_DS18B20_ADC_09B)
		  onewireDS18B20_setAdcWidth( 9, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, owDevices[idx]);
#endif
	  }
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t loopEntry = 1U;

  // xxx start of WHILE LOOP
  while (1)
  {
	  /* RESPONSE SECTION */
	  if (!loopEntry) {
		  /* Wait for ublox NEO responses - duration: blocking until new second starts */
		  mainLoop_ublox_waitForResponses();
		  gMLoop_Tim2_00_ubloxResp = tim_get_timeStamp(&htim2);

#if defined(PLL_BY_SOFTWARE)
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_RESET);
#endif

		  /* Wait for temperature data - duration: abt. 12.5 ms / blocking about until 750 ms after start */
		  for (uint8_t owDeviceIdx = 0; owDeviceIdx < owDevicesCount; ++owDeviceIdx) {
			  if (gMtempWaitUntil[owDeviceIdx]) {
				  mainLoop_ow_temp_waitForResponse(gMtempWaitUntil[owDeviceIdx], owDeviceIdx);
				  gMtempWaitUntil[owDeviceIdx] = 0UL;
			  }
		  }
		  gMLoop_Tim2_01_tempResp = tim_get_timeStamp(&htim2);


		  /* Stop ADC in case something still runs */
		  adc_stop();

		  /* Get ADC voltages - duration: abt. 4 us */
		  mainLoop_adc_volts_resp();
		  gMLoop_Tim2_02_adcResp = tim_get_timeStamp(&htim2);


		  /* Calculate timing deviation - duration: abt. 4 us */
		  mainLoop_tim_deviation_resp();
		  gMLoop_Tim2_03_deviationCalc = tim_get_timeStamp(&htim2);

		  /* The PLL control - duration: abt. 4 us */
		  mainLoop_PLL_calc();
		  gMLoop_Tim2_04_pllCalc = tim_get_timeStamp(&htim2);


		  /* NEO NAV-SVINFO sorting for desc. Elevations - duration: abt. 300 us */
		  gMelevSortTgtPosElevCnt = mainLoop_ublox_svinfo_sort(gMelevSortTgtCh);
		  gMLoop_Tim2_05_svSort = tim_get_timeStamp(&htim2);
	  }  // /* RESPONSE SECTION */


	  /* REQUEST SECTION */
	  {
#if defined(PLL_BY_SOFTWARE)
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_SET);
#endif

		  /* Send ublox NEO requests - duration: abt. 15 ms */
		  mainLoop_ublox_requests();
		  gMLoop_Tim2_10_ubloxReq = tim_get_timeStamp(&htim2);

		  /* Start Onewire temp sensor - one per second - duration: abt. 11 ms */
		  if (owDevicesCount) {
			  /* Switch to the next sensor */
			  ++gMowSensorIdx;
			  gMowSensorIdx %= owDevicesCount;

			  /* Request next temperature value of next sensor */
			  gMtempWaitUntil[gMowSensorIdx] = onewireDS18B20_tempReq(owDevices[gMowSensorIdx]);
		  }
		  gMLoop_Tim2_11_tempReq = tim_get_timeStamp(&htim2);

		  /* Start ADC channel scan */
		  adc_start();

		  /* Last of cycle: print time stamp values of the WHILE LOOP */
		  mainLoop_dbg_tim2_ts_print();
	  }  // /* REQUEST SECTION */


	  /* OUTPUT SECTION */
	  if (!loopEntry) {
		  /* Update relay and DAC setting - duration: abt. 2 us */
		  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, gpioHoRelayOut);
		  if (gpioHoRelayOut == GPIO_PIN_SET) {
			  /* Check for DAC */
			  if (i2cDevicesBF & I2C_DEVICE_DAC_MCP4725_0) {
				  if ((i2cDacModeLast != i2cDacMode) || (i2cDacValLast != i2cDacVal)) {
					  i2cDeviceDacMcp4725_set(0, i2cDacMode, i2cDacVal);

					  /* Store current settings */
					  i2cDacModeLast 	= i2cDacMode;
					  i2cDacValLast 	= i2cDacVal;
				  }
			  }
		  }
		  gMLoop_Tim2_20_hoRelayDacOut = tim_get_timeStamp(&htim2);

#if !defined(PLL_BY_SOFTWARE)
		  /* Update Locked-LED */
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, gpioLockedLED);
#endif


		  /* Show all NEO data - duration: abt. 37 ms (without NAV-SVINFO) */
		  mainLoop_ublox_print();
		  gMLoop_Tim2_21_ubloxPrint = tim_get_timeStamp(&htim2);

		  /* Show deviation values - duration: abt. 15 ms */
		  mainLoop_tim_deviation_print();
		  gMLoop_Tim2_22_deviationPrint = tim_get_timeStamp(&htim2);

		  /* Show PLL settings - duration: abt. 5.5 ms */
		  mainLoop_PLL_print();
		  gMLoop_Tim2_23_pllPrint = tim_get_timeStamp(&htim2);

		  /* Show ADC voltages - duration: abt. 24 ms */
		  mainLoop_adc_volts_print();
		  gMLoop_Tim2_24_adcPrint = tim_get_timeStamp(&htim2);

		  /* Temp values and alarms - duration: abt. 8 ms */
		  mainLoop_ow_temp_print();
		  mainLoop_ow_tempAlarm_print();
		  gMLoop_Tim2_25_tempPrint = tim_get_timeStamp(&htim2);


		  /* Update LCD16x2 - duration: abt. 1 us (not connected) */
		  if (i2cDevicesBF & I2C_DEVICE_LCD_0) {
			  if (!gpioLockedLED) {
				  i2cMCP23017_Lcd16x2_OCXO_HeatingUp((owDs18b20_Temp[gMowSensorIdx] >> 4), ubloxTimeAcc);
			  }
			  else {
				  i2cMCP23017_Lcd16x2_Locked((owDs18b20_Temp[gMowSensorIdx] >> 4), ubloxTimeAcc, timTicksSumDev);
			  }
		  }
		  gMLoop_Tim2_26_lcd16x2Print = tim_get_timeStamp(&htim2);

		  /* Update LCD240x128 - duration: abt. 2 us (no data presented) */
		  if (i2cDevicesBF & I2C_DEVICE_LCD_1) {
			  static uint8_t lcd1StateLast = 0U;

			  if (!gpioLockedLED) {
				  if (lcd1StateLast) {
					  /* Welcome template */
					  i2cSmartLCD_Gfx240x128_Welcome();
				  }

				  i2cSmartLCD_Gfx240x128_OCXO_HeatingUp(
						  (owDs18b20_Temp[gMowSensorIdx] >> 4),
						  ubloxTimeAcc);
				  lcd1StateLast = 0U;
			  }
			  else {
				  if (!lcd1StateLast) {
					  /* Locked template */
					  i2cSmartLCD_Gfx240x128_Locked_Template();
				  }

				  i2cSmartLCD_Gfx240x128_Locked(
						  (owDs18b20_Temp[gMowSensorIdx] >> 4),
						  ubloxTimeAcc,
						  timTicksSumDev,
						  gMdevPsS,
						  i2cDacVal,
						  i2cDacFraction,
						  ubloxNavDop.gDOP,
						  gMelevSortTgtPosElevCnt,
						  gMelevSortTgtCh,
						  &ubloxNavSvinfo);
				  lcd1StateLast = 1U;
			  }
		  }
		  gMLoop_Tim2_27_lcd240x128Print = tim_get_timeStamp(&htim2);
	  }  // /* OUTPUT SECTION */
	  loopEntry = 0U;


	  /* TOOL-BOX SECTION */
	  {
#if 0
		  /* DAC value calibration helper */
		  {
			  uint8_t  sel2 = (uint8_t) (now % 2);
			  switch (sel2) {
			  case 0:
			  default:
				  i2cDeviceDacMcp4725_set(0, 0b11, I2C_DAC_MCP4725_0_VAL);
				  break;

			  case 1:

				  i2cDeviceDacMcp4725_set(0, 0b00, I2C_DAC_MCP4725_0_VAL);
				  break;
			  }
		  }
#endif
	  }  // /* TOOL-BOX SECTION */
	  // xxx end of WHILE LOOP

	/* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
