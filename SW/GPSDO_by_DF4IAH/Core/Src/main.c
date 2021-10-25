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
#define F_COMP_HZ	1000
//#define F_COMP_HZ	10000
//#define F_COMP_HZ	100000

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

extern float 			giTim2Ch2_ppm;
extern int32_t 			giTim2Ch2_TicksDiff;
extern uint32_t 		giTim2Ch2_TicksEvt;
extern int32_t 			giTim2Ch2_TicksSumDev;

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
extern UbloxNavPosllh_t	ubloxNavPosllh;
extern UbloxNavClock_t	ubloxNavClock;
extern UbloxNavDop_t	ubloxNavDop;
extern UbloxNavSvinfo_t	ubloxNavSvinfo;
extern uint32_t			ubloxTimeAcc;


uint8_t 				owAlarmDevices[2][8] 					= { 0 };
uint8_t					owAlarmCount							= 0U;

uint8_t					gMelevSortTgtPosElevCnt					= 0U;
uint8_t 				gMelevSortTgtCh[UBLOX_MAX_CH]			= { 0 };

uint32_t 				gMtempWaitUntil[ONEWIRE_DEVICES_MAX]	= { 0 };
uint8_t  				gMowSensorIdx 							= 0;

float					gMdevPsS								= 0.0f;

uint8_t					gLocator[7]								= { 0 };

uint8_t					gDcfPhaseMod[512]						= { 0 };

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
extern void ublox_NavPosllh_req(UbloxNavPosllh_t* ubloxNavPosllh);
extern void ublox_NavClock_req(UbloxNavClock_t* ubloxNavClock);
extern void ublox_NavDop_req(UbloxNavDop_t* dop);
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

extern uint8_t i2cSmartLCD_Gfx240x128_Template(uint32_t bf);
extern uint8_t i2cSmartLCD_Gfx240x128_Welcome(void);
extern uint8_t i2cSmartLCD_Gfx240x128_OCXO_HeatingUp(int16_t temp, uint32_t tAcc);
extern void i2cSmartLCD_Gfx240x128_Locked(uint32_t maxUntil, int16_t temp, uint32_t tAcc, int32_t sumDev, float devPsS, uint16_t dacVal, float dacFraction, uint16_t gDOP, uint8_t svPosElevCnt, uint8_t svElevSort[UBLOX_MAX_CH], UbloxNavSvinfo_t* svInfo, const uint8_t* locatorStr);

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

/*
Verification of the PZN Code

Reference:	Empfang und Dekodierung des phasenmodulierten Signals des DCF77- Zeitzeichensenders - 2014 - 13, 41, 31.
			Mittweida, Hochschule Mittweida, Fakultät Elektro- und Informationstechnik, Diplomarbeit, 2014

			page 58.

Pseudozufallsfolge (PZF) des DCF77
{
	0,0,0,0,1,0,0,0,
	1,1,0,0,0,0,1,0,
	0,1,1,1,0,0,1,0,
	1,0,1,0,1,1,0,0,

	0,0,1,1,0,1,1,1,
	1,0,1,0,0,1,1,0,
	1,1,1,0,0,1,0,0,
	0,1,0,1,0,0,0,0,

	1,0,1,0,1,1,0,1,
	0,0,1,1,1,1,1,1,
	0,1,1,0,0,1,0,0,
	1,0,0,1,0,1,1,0,

	1,1,1,1,1,1,0,0,
	1,0,0,1,1,0,1,0,
	1,0,0,1,1,0,0,1,
	1,0,0,0,0,0,0,0,

	1,1,0,0,0,1,1,0,
	0,1,0,1,0,0,0,1,
	1,0,1,0,0,1,0,1,
	1,1,1,1,1,1,0,1,

	0,0,0,1,0,1,1,0,
	0,0,1,1,1,0,1,0,
	1,1,0,0,1,0,1,1,
	0,0,1,1,1,1,0,0,

	0,1,1,1,1,1,0,1,
	1,1,0,1,0,0,0,0,
	0,1,1,0,1,0,1,1,
	0,1,1,0,1,1,1,0,

	1,1,0,0,0,0,0,1,
	0,1,1,0,1,0,1,1,
	1,1,1,0,1,0,1,0,
	1,0,1,0,0,0,0,0,

	0,1,0,1,0,0,1,0,
	1,0,1,1,1,1,0,0,
	1,0,1,1,1,0,1,1,
	1,0,0,0,0,0,0,1,

	1,1,0,0,1,1,1,0,
	1,0,0,1,0,0,1,1,
	1,1,0,1,0,1,1,1,
	0,1,0,1,0,0,0,1,

	0,0,1,0,0,0,0,1,
	1,0,0,1,1,1,0,0,
	0,0,1,0,1,1,1,1,
	0,1,1,0,1,1,0,0,

	1,1,0,1,0,0,0,0,
	1,1,1,0,1,1,1,1,
	0,0,0,0,1,1,1,1,
	1,1,1,1,1,0,0,0,

	0,0,1,1,1,1,0,1,
	1,1,1,1,0,0,0,1,
	0,1,1,1,0,0,1,1,
	0,0,1,0,0,0,0,0,

	1,0,0,1,0,1,0,0,
	1,1,1,0,1,1,0,1,
	0,0,0,1,1,1,1,0,
	0,1,1,1,1,1,0,0,

	1,1,0,1,1,0,0,0,
	1,0,1,0,1,0,0,1,
	0,0,0,1,1,1,0,0,
	0,1,1,0,1,1,0,1,

	0,1,0,1,1,1,0,0,
	0,1,0,0,1,1,0,0,
	0,1,0,0,0,1,0,0,
	0,0,0,0,0,0,1,0

};  Anzahl Einsen: 256 Anzahl Nullen: 256.
*/

void calcDcfPhasemod(void)
{
	/* The result of this function is OK verified against the table above */
	uint16_t shift = 1U;

#if 0
	/* Prepare */
	memclear(gDcfPhaseMod, 512);
#endif

	for (uint16_t idx = 0; idx < 512U; ++idx) {
		uint8_t b5		= (0 != (shift & 0b000010000U));
		uint8_t b9		= (0 != (shift & 0b100000000U));
		uint8_t xor59	= (b5 != b9);
		shift <<= 1;
		if (xor59) {
			shift |= 0x01U;
		}

		gDcfPhaseMod[idx] 		 =  xor59;
	}
}


uint8_t main_get_MaidenheadLocator_from_LatLon(uint8_t maxLen, uint8_t* tgtStr, float lat, float lon)
{
	if (
			(maxLen >= 7)   &&
			( -90.0f < lat) && (lat <  +90.0f) &&
			(-180.0f < lon) && (lon < +180.0f))
	{
		/* Grid movement */
		lon += 180.0f;
		lat +=  90.0f;

		lon *= 25U;
		lon	-= 0.5f;
		lon /= 2U;
		uint32_t lon_i = (uint32_t) lon;

		lat *= 25U;
		lat	-= 0.5f;
		uint32_t lat_i = (uint32_t) lat;

		uint8_t	lon_p0	= (uint8_t) (lon_i / 250UL);
		uint8_t	lat_p0	= (uint8_t) (lat_i / 250UL);

		lon_i -= lon_p0 * 250UL;
		lat_i -= lat_p0 * 250UL;

		uint8_t lon_p1 = (uint8_t) (lon_i / 25UL);
		uint8_t lat_p1 = (uint8_t) (lat_i / 25UL);

		lon_i -= lon_p1 * 25UL;
		lat_i -= lat_p1 * 25UL;

		uint8_t lon_p2 = lon_i;
		uint8_t lat_p2 = lat_i;

		/* Output string */
		*(tgtStr + 0)	= 'A' + lon_p0;
		*(tgtStr + 1)	= 'A' + lat_p0;
		*(tgtStr + 2)	= '0' + lon_p1;
		*(tgtStr + 3)	= '0' + lat_p1;
		*(tgtStr + 4)	= 'a' + lon_p2;
		*(tgtStr + 5)	= 'a' + lat_p2;
		*(tgtStr + 6)	= 0;

		return 0;
	}

	/* Error */
	return 1;
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
			  giTim2Ch2_TicksDiff 	= 0L;
			  giTim2Ch2_TicksEvt	= 0UL;

			  /* Not locked in */
			  gpioLockedLED = GPIO_PIN_RESET;
		  }
	  }

	  /* Check if ubox NEO is locked in */
	  if (ubloxTimeAcc >= 250UL) {  // when worse than that stop time tracking
		  /* Keep sum-up registers cleared */
		  giTim2Ch2_TicksDiff 	= 0L;
		  giTim2Ch2_TicksEvt	= 0UL;

		  /* Not locked in */
		  gpioLockedLED = GPIO_PIN_RESET;
	  }

	  if (giTim2Ch2_TicksEvt > 15) {
		  /* Fractions accounting */
		  if (0 < giTim2Ch2_TicksDiff) {
			  if (giTim2Ch2_ppm > 0.0f) {
				  i2cDacFraction -= giTim2Ch2_TicksDiff / SW_PLL_TUNE_FAST;
			  } else {
				  i2cDacFraction += giTim2Ch2_TicksDiff / SW_PLL_TUNE_SLOW;
			  }
		  }
		  else if (giTim2Ch2_TicksDiff < 0) {
			  if (giTim2Ch2_ppm < 0.0f) {
				  i2cDacFraction -= giTim2Ch2_TicksDiff / SW_PLL_TUNE_FAST;
			  } else {
				  i2cDacFraction += giTim2Ch2_TicksDiff / SW_PLL_TUNE_SLOW;
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
		  giTim2Ch2_TicksDiff	= 0UL;
		  gpioLockedLED = GPIO_PIN_RESET;
	  }
  }

#else

  /* OCXO controlled by hardware PLL */
  {
	  /* DAC high impedance mode */
	  i2cDacMode	= 0b11;
	  i2cDacVal		= I2C_DAC_MCP4725_0_VAL;

	  /* Do not tune when primary temp sensor is out of temp range of OCXO */
	  if (owDevicesCount) {
		  if (owDs18b20_Temp_f[0] < ONEWIRE_DS18B20_ALARM_LO) {
			  /* Keep sum-up registers cleared */
			  giTim2Ch2_TicksDiff 	= 0L;
			  giTim2Ch2_TicksEvt	= 0UL;

			  /* Not locked in */
			  gpioLockedLED = GPIO_PIN_RESET;
		  }
	  }

	  /* Check if ubox NEO is locked in */
	  if (ubloxTimeAcc >= 250UL) {  // when worse than that stop time tracking
		  /* Keep sum-up registers cleared */
		  giTim2Ch2_TicksDiff 	= 0L;
		  giTim2Ch2_TicksEvt	= 0UL;

		  /* Not locked in */
		  gpioLockedLED = GPIO_PIN_RESET;
	  }

	  /* Prevent to early PLL lock indication */
	  if (giTim2Ch2_TicksEvt > 15) {
#if 1
		  /* Forward PLL lock state from the hardware line */
		  gpioLockedLED = HAL_GPIO_ReadPin(D10_PLL_LCKD_GPIO_I_GPIO_Port, D10_PLL_LCKD_GPIO_I_Pin);
#else
		  gpioLockedLED	= GPIO_PIN_SET;		// xxx  remove me!
#endif
	  }
	  else {
		  /* To early */
		  gpioLockedLED	= GPIO_PIN_RESET;
	  }

	  /* Clear the sum deviation register as long as the PLL is not locked */
	  if (gpioLockedLED == GPIO_PIN_RESET) {
		  giTim2Ch2_TicksDiff	= 0UL;
	  }
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

	/* Request only when needed */
	{
		if (!ubloxNavPosllh.iTOW) {
			ublox_NavPosllh_req(&ubloxNavPosllh);
		}

		if (!ubloxNavClock.iTOW) {
			ublox_NavClock_req(&ubloxNavClock);
		}

		if (!ubloxNavDop.iTOW) {
			ublox_NavDop_req(&ubloxNavDop);
		}

		if (!ubloxNavSvinfo.iTOW) {
			ublox_NavSvinfo_req(&ubloxNavSvinfo);
		}
	}
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
			uint8_t elevOk	= (ubloxNavSvinfo.quality[elevCh] & 0x0dU) && !(ubloxNavSvinfo.quality[elevCh] & 0x10U);

			if ((elevVal > elevMaxVal) && elevOk) {
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

void mainLoop_ow_tempAlarm_req(void)
{
	uint8_t owAlarmDevices[2][8] = { 0 };

	owAlarmCount = onewireMasterTree_search(1U, owDevicesCount, owAlarmDevices);
}

void mainLoop_ow_tempAlarm_print(void)
{
#if defined(LOGGING)
	if (owAlarmCount) {
		uint8_t msg[64];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** Temperature ALARM: %d sensor(s) out of limits.\r\n", owAlarmCount);
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
	if (giTim2Ch2_TicksEvt) {
		/* Export accumulated deviation */
		if (giTim2Ch2_TicksDiff >= 0L) {
			giTim2Ch2_TicksSumDev = (int32_t) (+0.5f + giTim2Ch2_TicksDiff * 100.0f / (6.0f * giTim2Ch2_TicksEvt));
		}
		else {
			giTim2Ch2_TicksSumDev = (int32_t) (-0.5f + giTim2Ch2_TicksDiff * 100.0f / (6.0f * giTim2Ch2_TicksEvt));
		}

		gMdevPsS = giTim2Ch2_TicksDiff * 100.0f / (6.0f * giTim2Ch2_TicksEvt);
	}
	else {
		giTim2Ch2_TicksSumDev 	= 0L;
		gMdevPsS 		= 0.0f;
	}
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

		len = snprintf(((char*) msg), sizeof(msg), "  *%+12.2f ps/s\r\n", 1e6 * giTim2Ch2_ppm);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  *%011.2f Hz\r\n", (110e6 + giTim2Ch2_ppm * 10.0f));
		msg[3] = ' ';
		HAL_UART_Transmit(&huart2, msg, len, 25);

		if (giTim2Ch2_TicksDiff >= 0) {
		  ticks_d = (uint32_t)giTim2Ch2_TicksDiff / 10;
		  ticks_f = (uint32_t)giTim2Ch2_TicksDiff % 10;
		  chr = '+';
		} else {
		  ticks_d = (uint32_t)(-giTim2Ch2_TicksDiff) / 10;
		  ticks_f = (uint32_t)(-giTim2Ch2_TicksDiff) % 10;
		  chr = '-';
		}
		len = snprintf(((char*) msg), sizeof(msg), "  * ?%lu.%01lu accumulated deviation ticks  during  runtime = %lu sec  (%.2f ps/s).\r\n",
			  ticks_d, ticks_f,
			  giTim2Ch2_TicksEvt,
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
		const uint32_t tps = 60000000UL;
		uint8_t msg[128];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** LOOP TIMES:\r\n");
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 00_ubloxResp        %8ld us   @ %07ld ticks.\r\n", 0UL, gMLoop_Tim2_00_ubloxResp);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 01_tempResp         %8ld us.\r\n", ((tps + gMLoop_Tim2_01_tempResp 			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 02_adcResp          %8ld us.\r\n", ((tps + gMLoop_Tim2_02_adcResp 			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 03_deviationCalc    %8ld us.\r\n", ((tps + gMLoop_Tim2_03_deviationCalc		- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 04_pllCalc          %8ld us.\r\n", ((tps + gMLoop_Tim2_04_pllCalc			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 05_svSort           %8ld us.\r\n", ((tps + gMLoop_Tim2_05_svSort			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 10_ubloxReq         %8ld us.\r\n", ((tps + gMLoop_Tim2_10_ubloxReq			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 11_tempReq          %8ld us.\r\n", ((tps + gMLoop_Tim2_11_tempReq			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 20_hoRelayDacOut    %8ld us.\r\n", ((tps + gMLoop_Tim2_20_hoRelayDacOut		- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 21_ubloxPrint       %8ld us.\r\n", ((tps + gMLoop_Tim2_21_ubloxPrint		- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 22_deviationPrint   %8ld us.\r\n", ((tps + gMLoop_Tim2_22_deviationPrint	- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 23_pllPrint         %8ld us.\r\n", ((tps + gMLoop_Tim2_23_pllPrint			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 24_adcPrint         %8ld us.\r\n", ((tps + gMLoop_Tim2_24_adcPrint			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 25_tempPrint        %8ld us.\r\n", ((tps + gMLoop_Tim2_25_tempPrint			- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 26_lcd16x2Print     %8ld us.\r\n", ((tps + gMLoop_Tim2_26_lcd16x2Print		- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
		HAL_UART_Transmit(&huart2, msg, len, 25);

		len = snprintf(((char*) msg), sizeof(msg), "  * 27_lcd240x128Print  %8ld us.\r\n", ((tps + gMLoop_Tim2_27_lcd240x128Print	- gMLoop_Tim2_00_ubloxResp) % tps) / 60);
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

  /* Early setting, if possible */
  MX_GPIO_Init();

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
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

#if 0
  static uint8_t enableMe = 0;
  while (!enableMe) {
  }
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


#if 0
  /* GPIO: Acoustic boot check */
  {
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);
	  HAL_Delay(250UL);
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_SET);
	  HAL_Delay(250UL);
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);
  }
#endif

  /* Default setting for hold relay */
  gpioHoRelayOut = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, gpioHoRelayOut);

  /* GPIO: Turn off Locked LED */
  gpioLockedLED = GPIO_PIN_RESET;
  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, gpioLockedLED);


  /* NEO: Turn NMEA messages off */
  ubloxMsgsTurnOff();

  /* NEO: Change baudrate of the u-blox */
  ubloxUartSpeedFast();

  /* NEO: Change 1PPS pulse frequency we need */
  uint8_t ubloxRetries = 3U;
  do {

#if defined(PLL_BY_SOFTWARE)
	  if (ubloxSetFrequency(1U)) {
#else
	  if (ubloxSetFrequency(F_COMP_HZ)) {
#endif

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

		  HAL_Delay(1300UL);
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


  /* TIMER: Prepare the Time Capture for TIM2 CH2 (GPS PPS), TIM15 CH1 fractional reload and Time Capture of TIM15 CH2 (DCF77 Phase) */
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

  /* Generate DCF77 pseudo phase noise modulation */
  calcDcfPhasemod();


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
# if 0
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_RESET);
# endif
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


		  /* Calculate Maidenhead Locator if not done, yet */
		  if ((gLocator[0] == 0) && ubloxNavPosllh.iTOW) {
			  main_get_MaidenheadLocator_from_LatLon(sizeof(gLocator), gLocator, ubloxNavPosllh.lat * 1e-7, ubloxNavPosllh.lon * 1e-7);
		  }

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
		  /* Request these frames */
		  ubloxNavClock.iTOW	= 0UL;
		  ubloxNavDop.iTOW		= 0UL;
		  ubloxNavSvinfo.iTOW	= 0UL;

		  /* Send ublox NEO requests - duration: abt. 15 ms */
		  mainLoop_ublox_requests();
		  gMLoop_Tim2_10_ubloxReq = tim_get_timeStamp(&htim2);

		  /* Request all sensors being in alarm state */
		  mainLoop_ow_tempAlarm_req();

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

		  /* Update Locked-LED */
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, gpioLockedLED);

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

		  float temp = (owDs18b20_Temp[gMowSensorIdx] >> 4) + 0.5f;
		  if (temp > 99.0f) {
			  temp = 99.0f;
		  }
		  else if (temp < 0.0f) {
			  temp = 0.0f;
		  }


		  /* Drop NEO data when falling back to out-of-lock state */
		  if (!gpioLockedLED) {
			  ubloxNavPosllh.iTOW 	= 0UL;
			  gLocator[0] 			= 0x00U;
		  }


		  /* Update LCD16x2 - duration: abt. 1 us (not connected) */
		  if (i2cDevicesBF & I2C_DEVICE_LCD_0) {
			  if (!gpioLockedLED) {
				  i2cMCP23017_Lcd16x2_OCXO_HeatingUp(((int16_t) temp), ubloxTimeAcc);
			  }
			  else {
				  i2cMCP23017_Lcd16x2_Locked(((int16_t) temp), ubloxTimeAcc, giTim2Ch2_TicksSumDev);
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
						  ((int16_t) temp),
						  ubloxTimeAcc);
				  lcd1StateLast = 0U;
			  }
			  else {
				  const uint32_t tps = 60000000UL;

				  if (!lcd1StateLast) {
					  /* Locked template */
#if defined(PLL_BY_SOFTWARE)
					  /* With DAC graph template */
					  i2cSmartLCD_Gfx240x128_Template(0x80000113UL);
#else
					  /* Without DAC graph template */
					  i2cSmartLCD_Gfx240x128_Template(0x80000013UL);
#endif
				  }

				  i2cSmartLCD_Gfx240x128_Locked(
						  (HAL_GetTick() + (700UL - ((tps + gMLoop_Tim2_26_lcd16x2Print - gMLoop_Tim2_00_ubloxResp) % tps) / 60000)),
						  ((int16_t) temp),
						  ubloxTimeAcc,
						  giTim2Ch2_TicksSumDev,
						  gMdevPsS,
						  i2cDacVal,
						  i2cDacFraction,
						  ubloxNavDop.gDOP,
						  gMelevSortTgtPosElevCnt,
						  gMelevSortTgtCh,
						  &ubloxNavSvinfo,
						  gLocator);
				  lcd1StateLast = 1U;
			  }
		  }
		  gMLoop_Tim2_27_lcd240x128Print = tim_get_timeStamp(&htim2);

#if defined(PLL_BY_SOFTWARE)
# if 0
		  HAL_GPIO_WritePin(D2_OCXO_LCKD_GPIO_O_GPIO_Port, D2_OCXO_LCKD_GPIO_O_Pin, GPIO_PIN_SET);
# endif
#endif
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
