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
#define F_COMP_HZ 1000

#define LOGGING

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint16_t	adcCh9_val;
extern uint16_t	adcCh10_val;
extern uint16_t	adcCh16_val;
extern uint16_t adcVrefint_val;
extern const float VREFINT_CAL;

extern uint8_t onewireDevices[ONEWIRE_DEVICES_COUNT_MAX][8];
extern uint8_t onewireDeviceCount;


GPIO_PinState hoRelayOut						= GPIO_PIN_RESET;

UbloxNavDop_t		ubloxNavDop					= { 0 };
UbloxNavClock_t		ubloxNavClock				= { 0 };
UbloxNavSvinfo_t	UbloxNavSvinfo				= { 0 };

#if 0
uint8_t Onewire_useDeviceWithRomCode[8] = {				// LSB byte first
		ONEWIRE_DS18B20_DEV0_ROM_CODE0_FAMILY,			// Family Code
		ONEWIRE_DS18B20_DEV0_ROM_CODE1,					// 48-Bit device Code
		ONEWIRE_DS18B20_DEV0_ROM_CODE2,
		ONEWIRE_DS18B20_DEV0_ROM_CODE3,
		ONEWIRE_DS18B20_DEV0_ROM_CODE4,
		ONEWIRE_DS18B20_DEV0_ROM_CODE5,
		ONEWIRE_DS18B20_DEV0_ROM_CODE6,
		ONEWIRE_DS18B20_DEV0_ROM_CODE7_CRC				// CRC8 Code
};
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

extern void ADC_init(void);
extern void ADC_start(void);
extern void ADC_stop(void);

extern uint8_t onewire_CRC8_calc(uint8_t* fields, uint8_t len);
extern GPIO_PinState onewireMasterCheck_presence(void);
extern uint8_t onewireMasterTree_search(uint8_t searchAlarms, uint8_t devicesMax, uint8_t onewireDevices[][8]);
extern void onewireDS18B20_readROMcode(uint8_t* romCode);
extern void onewireDS18B20_setAdcWidth(uint8_t width, int8_t tempAlarmHi, int8_t tempAlarmLo, uint8_t* romCode);
extern uint32_t onewireDS18B20_tempReq(uint8_t* romCode);
extern int16_t onewireDS18B20_tempRead(uint32_t waitUntil, uint8_t* romCode);

extern uint8_t i2cBusGetDeviceList(uint32_t* i2cDevicesBF);
extern uint8_t i2cDeviceDacMcp4725_set(uint8_t chipAddr, uint8_t pdMode, uint16_t dac_12b);
extern void ubloxUartSpeedFast(void);
extern void ubloxFlush(void);
extern void ubloxMsgsTurnOff(void);
extern void ublox_NavDop_get(UbloxNavDop_t* dop);
extern void ublox_NavClock_get(UbloxNavClock_t* ubloxNavClock);
extern void ublox_NavSvinfo_get(UbloxNavSvinfo_t* ubloxNavSvinfo);
extern uint8_t ubloxSetFrequency(uint16_t frequency);

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


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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

#if defined(LOGGING)
  {
	uint8_t msg[] = "\r\n\r\n************************\r\n*** sGPSDO by DF4IAH ***\r\n************************\r\n\r\n";
	HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
  }
#endif

  /* Switching to Hold mode */
  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_SET);

  /* Get list of all I2C devices */
  uint32_t i2cDevicesBF = 0UL;
  uint8_t i2cBusCnt = i2cBusGetDeviceList(&i2cDevicesBF);

  if (i2cDevicesBF & I2C_DEVICE_DAC_MCP4725_0) {
	  /* Switch DAC to high impedance (500kR) mode */
	  i2cDeviceDacMcp4725_set(0, 0b11, I2C_DAC_MCP4725_0_VAL);
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

  /* Prepare the ADC */
  ADC_init();

  /* Init the temperature sensor DS18B20 */
  {
	  memclear((uint8_t*) onewireDevices, sizeof(onewireDevices));
	  onewireDeviceCount = onewireMasterTree_search(0U, ONEWIRE_DEVICES_COUNT_MAX, onewireDevices);

#if defined(LOGGING)
	  {
		  uint8_t msg[64];
		  int len;

		  len = snprintf(((char*) msg), sizeof(msg), "\r\n*** 1-wire Temperature sensors found: %d\r\n", onewireDeviceCount);
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }
#endif

	  /* Set configuration and temp alarm limits */
#if   defined(ONEWIRE_DS18B20_ADC_12B)
	  onewireDS18B20_setAdcWidth(12, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, onewireDevices[0]);
#elif defined(ONEWIRE_DS18B20_ADC_11B)
	  onewireDS18B20_setAdcWidth(11, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, onewireDevices[0]);
#elif defined(ONEWIRE_DS18B20_ADC_10B)
	  onewireDS18B20_setAdcWidth(10, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, onewireDevices[0]);
#elif defined(ONEWIRE_DS18B20_ADC_09B)
	  onewireDS18B20_setAdcWidth( 9, ONEWIRE_DS18B20_ALARM_HI, ONEWIRE_DS18B20_ALARM_LO, onewireDevices[0]);
#endif
  }

  /* Turn off many of the NMEA messages */
  //HAL_Delay(2000UL);
  ubloxMsgsTurnOff();

  /* Change baudrate of the u-blox */
  ubloxUartSpeedFast();

  if (ubloxSetFrequency(F_COMP_HZ)) {
#if defined(LOGGING)
	  {
		uint8_t msg[] = "*** u-blox TimePulse has not changed - keeping in Hold mode.\r\n";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	  }
#endif
  }
  else {
#if defined(LOGGING)
	  {
		uint8_t msg[] = "*** u-blox TimePulse modification has worked - switching from Hold to PLL mode.\r\n";
		HAL_UART_Transmit(&huart2, msg, sizeof(msg) - 1, 25);
	  }
#endif
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t tempWaitUntil = 0UL;
	  uint32_t now = HAL_GetTick() / 1000UL;  (void) now;

	  /* Start ADC channel scan */
	  ADC_start();

#if 0
	  uint8_t onewireAlarms[2][8] = { 0 };
	  uint8_t onewireAlarmsCount = onewireMasterTree_search(1U, 2U, onewireAlarms);

	  if (onewireAlarmsCount) {
#if defined(LOGGING)
	  {
		uint8_t msg[64];
		int len;

		len = snprintf(((char*) msg), sizeof(msg), "\r\n*** Temperature ALARM: %d sensor(s) out of limits.\r\n", onewireAlarmsCount);
		HAL_UART_Transmit(&huart2, msg, len, 25);
	  }
#endif
	  }
#endif

#if 1
	  if (tempWaitUntil) {
		uint8_t msg[64];

#if defined(LOGGING)
		int len = snprintf(((char*) msg), sizeof(msg), "\r\n");
		HAL_UART_Transmit(&huart2, msg, len, 25);
#endif

		for (uint8_t idx = 0; idx < onewireDeviceCount; ++idx) {
			/* Onewire handling */
			int16_t owDs18b20_Temp = onewireDS18B20_tempRead(tempWaitUntil, onewireDevices[idx]);

			int16_t t_int		= (owDs18b20_Temp >> 4);

			uint16_t t_frac		= (owDs18b20_Temp & 0xfU);
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

#if defined(LOGGING)
			int len = snprintf(((char*) msg), sizeof(msg), "*** Temperature sensor %d: %+02d,%02u degC\r\n", idx, t_int, (t_fv1000 + 5) / 10);
			HAL_UART_Transmit(&huart2, msg, len, 25);
#endif
		}
  }
#endif

#if defined(LOGGING)
	  /* Show PLL Lock state */
	  {
		  uint8_t msg[64];
		  int len;

		  len = snprintf(((char*) msg), sizeof(msg), "\r\n*** PLL Lock state = %d\r\n", HAL_GPIO_ReadPin(D10_PLL_LCKD_GPIO_I_GPIO_Port, D10_PLL_LCKD_GPIO_I_Pin));
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }

#endif
	  /* Start Onewire temp sensor - one per second */
	  {
		  static uint8_t onewireSensorIdx = 0;

		  /* Request next temperature value of one sensor */
		  tempWaitUntil = onewireDS18B20_tempReq(onewireDevices[onewireSensorIdx]);

		  /* Switch to the next sensor */
		  ++onewireSensorIdx;
		  onewireSensorIdx %= onewireDeviceCount;
	  }

	  /* Blocks until new frame comes in */
	  static uint8_t  sel3 = 0U;

	  ++sel3;
	  sel3 %= 3;
	  switch (sel3) {
	  case 0:
	  default:
		  ublox_NavClock_get(&ubloxNavClock);
		  break;

	  case 1:
		  ublox_NavDop_get(&ubloxNavDop);
		  break;

	  case 2:
		  ublox_NavSvinfo_get(&UbloxNavSvinfo);
		  break;
	  }


#if 0
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
#endif


#if 0
	  static uint32_t uwTick_last = 0UL;
	  uint32_t uwTick_now;

	  HAL_GPIO_WritePin(D6_HoRelay_GPIO_O_GPIO_Port, D6_HoRelay_GPIO_O_Pin, GPIO_PIN_RESET);

	  while (1) {
		  uwTick_now = HAL_GetTick();
		  if (uwTick_last + 1000UL <= uwTick_now) {
			  uwTick_last = uwTick_now;
			  break;
		  }
	  }

	  HAL_GPIO_WritePin(D6_HoRelay_GPIO_O_GPIO_Port, D6_HoRelay_GPIO_O_Pin, GPIO_PIN_SET);

	  while (1) {
		  uwTick_now = HAL_GetTick();
		  if (uwTick_last + 1000UL <= uwTick_now) {
			  uwTick_last = uwTick_now;
			  break;
		  }
	  }
#endif

	  /* Stop ADC in case something still runs */
	  ADC_stop();

#if defined(LOGGING)
	  /* Show ADC values */
	  {
		  uint8_t msg[128];
		  int len;

		  const float adc_VDDA = (3.0f * VREFINT_CAL) / adcVrefint_val;  // p. 448f

		  len = snprintf(((char*) msg), sizeof(msg), "\t\t\t*** ADC values:\r\n");
		  HAL_UART_Transmit(&huart2, msg, len, 25);

		  len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * VDDA                 = %1.4f V\r\n"
				  	  	  	  	  	  	  	  	  	 "\t\t\t  *\r\n",
				  adc_VDDA);
		  HAL_UART_Transmit(&huart2, msg, len, 25);

		  len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * (Ch09) V_OCXO        = 0x%04x = %05d  -->  V_OCXO   = %1.3f V\r\n",
				  adcCh9_val,
				  adcCh9_val,
				  (adcCh9_val * adc_VDDA / 65536.0f));
		  HAL_UART_Transmit(&huart2, msg, len, 25);

		  len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * (Ch10) V_HOLD        = 0x%04x = %05d  -->  V_HOLD   = %1.3f V\r\n",
				  adcCh10_val,
				  adcCh10_val,
				  (adcCh10_val * adc_VDDA / 65536.0f));
		  HAL_UART_Transmit(&huart2, msg, len, 25);

		  len = snprintf(((char*) msg), sizeof(msg), "\t\t\t  * (Ch16) V_DCF77_DEMOD = 0x%04x = %05d  -->  V_DCFAMP = %1.3f V\r\n",
				  adcCh16_val,
				  adcCh16_val,
				  (adcCh16_val * adc_VDDA / 65536.0f));
		  HAL_UART_Transmit(&huart2, msg, len, 25);
	  }

#endif

	  /* Update relay */
	  HAL_GPIO_WritePin(D12_HoRelay_GPIO_O_GPIO_Port, D12_HoRelay_GPIO_O_Pin, hoRelayOut);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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
