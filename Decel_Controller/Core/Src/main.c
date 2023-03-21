/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max11270.h"
#include "mcp33131.h"
#include "max6225.h"
#include "board.h"
#include "ad5763.h"
#include "misc_funcs.h"
#include "stdint.h"
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_rcc.h"
#include "errno.h"
#include "spi_bare.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t new_usb_data; 			// flag to indicate new usb data has been received
uint8_t buffer_usb_rx[64]; 		// will automatically contain new USB received data
uint8_t new_uart_data; 			// flag to indicate new uart data has been received
uint8_t buffer_uart_rx[64]; 	// will automatically contain new UART received data
struct command cmd;				// holds the most recent command received over either USB or UART
uint8_t new_cmd;				// flag to indicate if the newest command has been acted on
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

	SPIConfig();
	SPI_Enable(SPI1);
	SPI_Enable(SPI2);

	led_twinkle(80); //allows voltage to settle before attempting calibration

	// allows access to clock cycles
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	uint8_t verbose = 0; 	// levels 0, 1, 2 - produces increasingly descriptive serial output

	new_usb_data = 0;
	new_uart_data = 0;
	new_cmd = 0;
	HAL_UART_Receive_IT (&huart3, buffer_uart_rx, 64); // start listening for data over fibre-optic UART

	splash_message(verbose);	// says a little hello and some system information and settings

	// Initialise and setup the various peripheral devices, ADC's, DAC's etc.

	struct board decel_board = init_board(verbose);

	struct max11270_device adc_slow_hv1_fb =
		  init_max11270(CS_ADC_SLOW_HV1_FB_Pin, CS_ADC_SLOW_HV1_FB_GPIO_Port,
				  RDY_ADC_SLOW_HV1_FB_Pin, RDY_ADC_SLOW_HV1_FB_GPIO_Port, 0, SPI2, verbose, "Vfb_slow_ADC");

	struct mcp33131_device adc_fast_hv1_fb =
		  init_mcp33131(CS_ADC_FAST_HV1_FB_Pin, CS_ADC_FAST_HV1_FB_GPIO_Port, SPI2, verbose, "Vfb_ADC");/*
	struct mcp33131_device adc_hv1_fg =
		  init_mcp33131(CS_ADC_HV1_FG_Pin, CS_ADC_HV1_FG_GPIO_Port, SPI2, verbose, "Vfg_ADC");
	struct mcp33131_device adc_temp_dac_hv1_ctrl =
		  init_mcp33131(CS_ADC_TEMP_DAC_HV1_CTRL_Pin, CS_ADC_TEMP_DAC_HV1_CTRL_GPIO_Port, SPI2, verbose, "DAC_temp_ADC");
	struct mcp33131_device adc_hv1_electrode_current =
		  init_mcp33131(CS_ADC_HV1_ELECTRODE_CURRENT_Pin, CS_ADC_HV1_ELECTRODE_CURRENT_GPIO_Port, SPI2, verbose, "current_ADC");

	struct ad5763_device dac_hv1 =
		  init_ad5763(SYNC_DAC_HV1_CTRL_Pin, SYNC_DAC_HV1_CTRL_GPIO_Port, SPI1,
				  HV1_COARSE_OFFSET, HV1_COARSE_GAIN, HV1_FINE_OFFSET, HV1_FINE_GAIN, verbose, "DAC_HV", "coarse", "fine");
	struct ad5763_device dac_ultravolt =
		  init_ad5763(SYNC_DAC_ULTRAVOLT_CTRL_Pin, SYNC_DAC_ULTRAVOLT_CTRL_GPIO_Port, SPI1,
				  UVA_OFFSET, UVA_GAIN, UVB_OFFSET, UVB_GAIN, verbose, "DAC_UV", "A", "B");*/

	struct max6225_device vref = init_max6225(verbose);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t lastTick = HAL_GetTick();
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// checks for newly received USB data, attempts to parse and act on it
		if (new_usb_data != 0)
		{
			cmd = handle_data(&buffer_usb_rx, verbose);
			if (cmd.valid == 1) new_cmd = 1;
			new_usb_data = 0;
		}

		// checks for newly received UART data, attempts to parse and act on it
		if (new_uart_data != 0)
		{
			cmd = handle_data(&buffer_uart_rx, verbose);
			if (cmd.valid == 1)new_cmd = 1;
			new_uart_data = 0;
		}

		// acts on new commands, calling appropriate function with appropriate arguments
		if (new_cmd == 1)
		{
			new_cmd = 0;
			if (verbose == 1) println("Command acknowledged.");
			if (verbose > 1)
			{
				print("Command received...\n\rType: ");
				println(cmd.type);
				print("Object: ");
				println(cmd.object);
				print("Value: ");
				printfloat(cmd.value, 1);
			}
			if (strcmp(cmd.type, "setSpeed") == 0)
			{
				if (strcmp(cmd.object, adc_slow_hv1_fb.name) == 0)
					cont_conversion(&adc_slow_hv1_fb, verbose, (uint8_t)cmd.value);
				else {} //no other devices with variable speed
			}
			else if (strcmp(cmd.type, "setVoltage") == 0)
			{
			}
			else if (strcmp(cmd.type, "requestValue") == 0)
			{
				if (strcmp(cmd.object, adc_slow_hv1_fb.name) == 0)
					printfloat(adc_slow_hv1_fb.result_uV, 1);
				else if (strcmp(cmd.object, "decelVoltageSlow") == 0)
					printfloat(decel_board.decel_voltage_slow, 1);
				else if (strcmp(cmd.object, "decelVoltageFast") == 0)
					printfloat(decel_board.decel_voltage_fast, 1);
				else if (strcmp(cmd.object, "decelVoltages") == 0)
				{
					printfloat(decel_board.decel_voltage_slow, 1);
					printfloat(decel_board.decel_voltage_fast, 1);
				}
			}
		}

		// checks for new ADC sample on the HV feedback, updates decel_board struct
		if (check_available_max11270(&adc_slow_hv1_fb) == 1)
		{
			read_max11270(&adc_slow_hv1_fb, &vref, verbose);
			updateDecelReadingSlow(&decel_board, &adc_slow_hv1_fb, verbose);
			printfloat(adc_slow_hv1_fb.result_bits, 1);
			printfloat(adc_slow_hv1_fb.result_uV, 1);
			printfloat(adc_fast_hv1_fb.result_bits, 1);
			printfloat(adc_fast_hv1_fb.result_uV, 1);
			println("");
		}

		if (check_available_mcp33131(&adc_fast_hv1_fb) == 1)
		{
			read_mcp33131(&adc_fast_hv1_fb, &vref, verbose);
			updateDecelReadingFast(&decel_board, &adc_fast_hv1_fb, verbose);
		}

		/*// updates DAC values, should check if necesarry first though
		if (decel_board.hv_enable == 1)
		{
			set_stage_uV(&dac_hv1, 1, 0, 1); 		// sets dac_coarse to 1V
			set_stage_uV(&dac_hv1, 2, 0, 1); 		// sets dac_fine to 1V
			set_stage_uV(&dac_ultravolt, 1, 0, 1); 	// sets ultravolt A input to 0V
			set_stage_uV(&dac_ultravolt, 2, 0, 1); 	// sets ultravolt B input to 0V
		}*/

		// heartbeat
		if (HAL_GetTick() - lastTick > 500)
		{
			HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
			lastTick = HAL_GetTick();
		}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_ADC_HV1_ELECTRODE_CURRENT_Pin|CS_ADC_HV1_FG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GAIN_B_Pin|GAIN_A_Pin|LED_3_Pin|HV_ENABLE_RETURN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_DISPLAY_Pin|CS_ADC_SLOW_HV1_FB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS_ADC_FAST_HV1_FB_Pin|CS_TC_RES_STACK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|LED_2_Pin|HV_ENABLE_SOFTWARE_Pin|SYNC_DAC_ULTRAVOLT_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_ADC_TEMP_DAC_HV1_CTRL_GPIO_Port, CS_ADC_TEMP_DAC_HV1_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYNC_DAC_HV1_CTRL_GPIO_Port, SYNC_DAC_HV1_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_ADC_HV1_ELECTRODE_CURRENT_Pin CS_ADC_HV1_FG_Pin */
  GPIO_InitStruct.Pin = CS_ADC_HV1_ELECTRODE_CURRENT_Pin|CS_ADC_HV1_FG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GAIN_B_Pin GAIN_A_Pin LED_3_Pin HV_ENABLE_RETURN_Pin */
  GPIO_InitStruct.Pin = GAIN_B_Pin|GAIN_A_Pin|LED_3_Pin|HV_ENABLE_RETURN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RDY_ADC_SLOW_HV1_FB_Pin */
  GPIO_InitStruct.Pin = RDY_ADC_SLOW_HV1_FB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RDY_ADC_SLOW_HV1_FB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_DISPLAY_Pin */
  GPIO_InitStruct.Pin = CS_DISPLAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_DISPLAY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ADC_SLOW_HV1_FB_Pin SYNC_DAC_HV1_CTRL_Pin */
  GPIO_InitStruct.Pin = CS_ADC_SLOW_HV1_FB_Pin|SYNC_DAC_HV1_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ADC_FAST_HV1_FB_Pin CS_TC_RES_STACK_Pin */
  GPIO_InitStruct.Pin = CS_ADC_FAST_HV1_FB_Pin|CS_TC_RES_STACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_2_Pin HV_ENABLE_SOFTWARE_Pin SYNC_DAC_ULTRAVOLT_CTRL_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_2_Pin|HV_ENABLE_SOFTWARE_Pin|SYNC_DAC_ULTRAVOLT_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_ADC_TEMP_DAC_HV1_CTRL_Pin */
  GPIO_InitStruct.Pin = CS_ADC_TEMP_DAC_HV1_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(CS_ADC_TEMP_DAC_HV1_CTRL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
		led_twinkle(10);
		println("ERROR!");

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
