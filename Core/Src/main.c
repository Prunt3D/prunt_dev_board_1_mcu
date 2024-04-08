/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcu_comms.h"
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

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
struct interrupt_inputs_type {
	uint16_t adc_sequence;
	uint16_t temperatures[4];
	uint16_t tachs[4];
};

static volatile struct interrupt_inputs_type interrupt_inputs = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM17_Init(void);
static void MX_IWDG_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	// From https://github.com/olikraus/stm32g031/tree/main/enable_boot0
	if ((FLASH->OPTR & FLASH_OPTR_nBOOT_SEL) != 0) {
		/* Clear the LOCK bit in FLASH->CR (precondition for option byte flash) */
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
		/* Clear the OPTLOCK bit in FLASH->CR */
		FLASH->OPTKEYR = 0x08192A3B;
		FLASH->OPTKEYR = 0x4C5D6E7F;

		/* Enable legacy mode (BOOT0 bit defined by BOOT0 pin) */
		/* by clearing the nBOOT_SELection bit */
		FLASH->OPTR &= ~FLASH_OPTR_nBOOT_SEL;

		/* check if there is any flash operation */
		while ((FLASH->SR & FLASH_SR_BSY1) != 0) {
		}

		/* start the option byte flash */
		FLASH->CR |= FLASH_CR_OPTSTRT;
		/* wait until flashing is done */
		while ((FLASH->SR & FLASH_SR_BSY1) != 0) {
		}

		/* do a busy delay, for about one second, check BSY1 flag to avoid compiler loop optimization */
		for (unsigned long i = 0; i < 2000000; i++) {
			if ((FLASH->SR & FLASH_SR_BSY1) != 0) {
				break;
			}
		}

		/* load the new value and do a system reset */
		/* this will behave like a goto to the begin of this main procedure */
		FLASH->CR |= FLASH_CR_OBL_LAUNCH;
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
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM14_Init();
	MX_TIM17_Init();
	MX_IWDG_Init();
	MX_CRC_Init();
	/* USER CODE BEGIN 2 */

	HAL_ADC_Start_IT(&hadc1);

	uint8_t message_buffer[Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING];
	BitStream bit_stream;
	Packet_Type packet;
	int asn1_err;

	if (HAL_I2C_Slave_Receive(&hi2c1, message_buffer,
	Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING, HAL_MAX_DELAY) != HAL_OK) {
		Error_Handler();
	}

	BitStream_AttachBuffer(&bit_stream, message_buffer,
	Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING);
	Packet_Type_ACN_Decode(&packet, &bit_stream, &asn1_err);
	if (asn1_err || packet.packet_data.kind != setup_PRESENT || packet.packet_data.u.setup.version != 0) {
		Error_Handler();
	}

//	__disable_irq();
//	HAL_GPIO_WritePin(Stepper_1_OE_GPIO_Port, Stepper_1_OE_Pin, 1);
//	HAL_GPIO_WritePin(Stepper_2_OE_GPIO_Port, Stepper_2_OE_Pin, 1);
//	HAL_GPIO_WritePin(Stepper_3_OE_GPIO_Port, Stepper_3_OE_Pin, 1);
//	HAL_GPIO_WritePin(Stepper_4_OE_GPIO_Port, Stepper_4_OE_Pin, 1);
//	HAL_GPIO_WritePin(Stepper_5_OE_GPIO_Port, Stepper_5_OE_Pin, 1);
//	HAL_GPIO_WritePin(Stepper_6_OE_GPIO_Port, Stepper_6_OE_Pin, 1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_IWDG_Refresh(&hiwdg);

		while (HAL_GPIO_ReadPin(Unused_PA14_Boot0_GPIO_Port, Unused_PA14_Boot0_Pin)) {
			// Force a reset in to bootloader.
		}

		__disable_irq();
		struct interrupt_inputs_type interrupt_inputs_copy = interrupt_inputs;
		__enable_irq();

		packet.packet_data.kind = inputs_PRESENT;
		packet.packet_data.u.inputs.adc_sequential_id = interrupt_inputs_copy.adc_sequence;
		packet.packet_data.u.inputs.adc_temp_value.arr[0] = interrupt_inputs_copy.temperatures[0];
		packet.packet_data.u.inputs.adc_temp_value.arr[1] = interrupt_inputs_copy.temperatures[1];
		packet.packet_data.u.inputs.adc_temp_value.arr[2] = interrupt_inputs_copy.temperatures[2];
		packet.packet_data.u.inputs.adc_temp_value.arr[3] = interrupt_inputs_copy.temperatures[3];
		packet.packet_data.u.inputs.fan_tach_count.arr[0] = interrupt_inputs_copy.tachs[0];
		packet.packet_data.u.inputs.fan_tach_count.arr[1] = interrupt_inputs_copy.tachs[1];
		packet.packet_data.u.inputs.fan_tach_count.arr[2] = interrupt_inputs_copy.tachs[2];
		packet.packet_data.u.inputs.fan_tach_count.arr[3] = interrupt_inputs_copy.tachs[3];
		packet.packet_data.u.inputs.stepper_fault.arr[0] = HAL_GPIO_ReadPin(
		Stepper_1_Fault_GPIO_Port, Stepper_1_Fault_Pin);
		packet.packet_data.u.inputs.stepper_fault.arr[1] = HAL_GPIO_ReadPin(
		Stepper_2_Fault_GPIO_Port, Stepper_2_Fault_Pin);
		packet.packet_data.u.inputs.stepper_fault.arr[2] = HAL_GPIO_ReadPin(
		Stepper_3_Fault_GPIO_Port, Stepper_3_Fault_Pin);
		packet.packet_data.u.inputs.stepper_fault.arr[3] = HAL_GPIO_ReadPin(
		Stepper_4_Fault_GPIO_Port, Stepper_4_Fault_Pin);
		packet.packet_data.u.inputs.stepper_fault.arr[4] = HAL_GPIO_ReadPin(
		Stepper_5_Fault_GPIO_Port, Stepper_5_Fault_Pin);
		packet.packet_data.u.inputs.stepper_fault.arr[5] = HAL_GPIO_ReadPin(
		Stepper_6_Fault_GPIO_Port, Stepper_6_Fault_Pin);

		BitStream_Init(&bit_stream, message_buffer, Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING);
		Packet_Type_ACN_Encode(&packet, &bit_stream, &asn1_err, true);
		if (asn1_err) {
			Error_Handler();
		}

		if (HAL_I2C_Slave_Transmit(&hi2c1, message_buffer,
		Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING, HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}

		if (HAL_I2C_Slave_Receive(&hi2c1, message_buffer,
		Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING, HAL_MAX_DELAY) != HAL_OK) {
			Error_Handler();
		}

		BitStream_AttachBuffer(&bit_stream, message_buffer,
		Packet_Type_REQUIRED_BYTES_FOR_ACN_ENCODING);
		Packet_Type_ACN_Decode(&packet, &bit_stream, &asn1_err);
		if (asn1_err || packet.packet_data.kind != outputs_PRESENT) {
			Error_Handler();
		}

		HAL_GPIO_WritePin(Heater_En_GPIO_Port, Heater_En_Pin, packet.packet_data.u.outputs.heater_enable);
		HAL_GPIO_WritePin(Stepper_1_OE_GPIO_Port, Stepper_1_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[0]);
		HAL_GPIO_WritePin(Stepper_2_OE_GPIO_Port, Stepper_2_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[1]);
		HAL_GPIO_WritePin(Stepper_3_OE_GPIO_Port, Stepper_3_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[2]);
		HAL_GPIO_WritePin(Stepper_4_OE_GPIO_Port, Stepper_4_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[3]);
		HAL_GPIO_WritePin(Stepper_5_OE_GPIO_Port, Stepper_5_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[4]);
		HAL_GPIO_WritePin(Stepper_6_OE_GPIO_Port, Stepper_6_OE_Pin,
				packet.packet_data.u.outputs.stepper_output_enable.arr[5]);
		HAL_GPIO_WritePin(Stepper_1_En_GPIO_Port, Stepper_1_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[0]);
		HAL_GPIO_WritePin(Stepper_2_En_GPIO_Port, Stepper_2_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[1]);
		HAL_GPIO_WritePin(Stepper_3_En_GPIO_Port, Stepper_3_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[2]);
		HAL_GPIO_WritePin(Stepper_4_En_GPIO_Port, Stepper_4_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[3]);
		HAL_GPIO_WritePin(Stepper_5_En_GPIO_Port, Stepper_5_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[4]);
		HAL_GPIO_WritePin(Stepper_6_En_GPIO_Port, Stepper_6_En_Pin, packet.packet_data.u.outputs.stepper_enable.arr[5]);
		TIM17->CCR1 = packet.packet_data.u.outputs.fan_pwm.arr[0];
		TIM3->CCR3 = packet.packet_data.u.outputs.fan_pwm.arr[1];
		TIM1->CCR1 = packet.packet_data.u.outputs.fan_pwm.arr[2];
		TIM14->CCR1 = packet.packet_data.u.outputs.fan_pwm.arr[3];
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x40003EFF;
	hi2c1.Init.OwnAddress1 = 230;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
	hiwdg.Init.Window = 4095;
	hiwdg.Init.Reload = 4095;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 639;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 639;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 0;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 639;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */
	HAL_TIM_MspPostInit(&htim14);

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 0;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 639;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */

	/* USER CODE END TIM17_Init 2 */
	HAL_TIM_MspPostInit(&htim17);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Stepper_6_En_GPIO_Port, Stepper_6_En_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, Stepper_3_En_Pin | Fan_PWM_OE_Pin | Stepper_6_OE_Pin | Stepper_5_OE_Pin | Heater_En_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Stepper_3_OE_Pin | Stepper_1_En_Pin | Stepper_1_OE_Pin | Stepper_2_OE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, Stepper_2_En_Pin | Stepper_4_En_Pin | Stepper_4_OE_Pin | Stepper_5_En_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Stepper_2_Fault_Pin Stepper_3_Fault_Pin Stepper_1_Fault_Pin */
	GPIO_InitStruct.Pin = Stepper_2_Fault_Pin | Stepper_3_Fault_Pin | Stepper_1_Fault_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Stepper_6_Fault_Pin */
	GPIO_InitStruct.Pin = Stepper_6_Fault_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Stepper_6_Fault_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Stepper_6_En_Pin */
	GPIO_InitStruct.Pin = Stepper_6_En_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Stepper_6_En_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : SPI1_MOSI_Pin SPI1_SCK_Pin SPI1_MISO_Pin Unused_PA14_Boot0_Pin */
	GPIO_InitStruct.Pin = SPI1_MOSI_Pin | SPI1_SCK_Pin | SPI1_MISO_Pin | Unused_PA14_Boot0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Unused_PA3_Pin Unused_PA10_Pin Unused_PA13_Pin */
	GPIO_InitStruct.Pin = Unused_PA3_Pin | Unused_PA10_Pin | Unused_PA13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI1_CSN_Pin */
	GPIO_InitStruct.Pin = SPI1_CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Unused_PB11_Pin */
	GPIO_InitStruct.Pin = Unused_PB11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Unused_PB11_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Stepper_3_En_Pin Fan_PWM_OE_Pin Stepper_6_OE_Pin Stepper_5_OE_Pin
	 Heater_En_Pin */
	GPIO_InitStruct.Pin = Stepper_3_En_Pin | Fan_PWM_OE_Pin | Stepper_6_OE_Pin | Stepper_5_OE_Pin | Heater_En_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Fan_3_Tach_Pin Fan_4_Tach_Pin */
	GPIO_InitStruct.Pin = Fan_3_Tach_Pin | Fan_4_Tach_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Stepper_3_OE_Pin Stepper_1_En_Pin Stepper_1_OE_Pin Stepper_2_OE_Pin */
	GPIO_InitStruct.Pin = Stepper_3_OE_Pin | Stepper_1_En_Pin | Stepper_1_OE_Pin | Stepper_2_OE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Fan_2_Tach_Pin Fan_1_Tach_Pin */
	GPIO_InitStruct.Pin = Fan_2_Tach_Pin | Fan_1_Tach_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Stepper_2_En_Pin Stepper_4_En_Pin Stepper_4_OE_Pin Stepper_5_En_Pin */
	GPIO_InitStruct.Pin = Stepper_2_En_Pin | Stepper_4_En_Pin | Stepper_4_OE_Pin | Stepper_5_En_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : Stepper_5_Fault_Pin Stepper_4_Fault_Pin */
	GPIO_InitStruct.Pin = Stepper_5_Fault_Pin | Stepper_4_Fault_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	static int next_sensor = 0;

	interrupt_inputs.temperatures[next_sensor] = HAL_ADC_GetValue(hadc);

	++interrupt_inputs.adc_sequence;
	++next_sensor;
	if (next_sensor == 4) {
		next_sensor = 0;
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t pin) {
	switch (pin) {
	case Fan_1_Tach_Pin:
		++interrupt_inputs.tachs[0];
		break;
	case Fan_2_Tach_Pin:
		++interrupt_inputs.tachs[1];
		break;
	case Fan_3_Tach_Pin:
		++interrupt_inputs.tachs[2];
		break;
	case Fan_4_Tach_Pin:
		++interrupt_inputs.tachs[3];
		break;
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
