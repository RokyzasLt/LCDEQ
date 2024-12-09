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

#include "arm_math.h"
#include "liquidcrystal_i2c.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Number_Of_Samples 512
#define Number_Of_Bins 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

arm_cfft_instance_f32 fft_instance;

uint32_t ADC_Val = 0;
uint32_t ADC_Values[Number_Of_Samples * 2] = {0};
int ADC_Val_Index = 0;

float32_t input_data_left [Number_Of_Samples * 2];
float32_t input_data_right[Number_Of_Samples * 2];
float32_t output_data_left[Number_Of_Samples];
float32_t output_data_right[Number_Of_Samples];
float32_t spectrum[Number_Of_Bins *2];
//{0,5,8,1,14,15,16,1,9,10};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

void Calculate_Spectrum();
void Display_Spectrum();
void Create_Special_Characters();


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
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  	HD44780_Init(2, &hi2c1);
 	HD44780_Init(2, &hi2c2);
	Create_Special_Characters();
	HAL_TIM_Base_Start_IT(&htim2);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
    	ADC_Values[ADC_Val_Index] = HAL_ADC_GetValue(&hadc1);
    	ADC_Val_Index++;
    	if(ADC_Val_Index == Number_Of_Samples * 2){
    		ADC_Val_Index = 0;
    		Calculate_Spectrum();
    		Display_Spectrum();
    	}
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) // Check which timer triggered the callback
    {
    	HAL_ADC_Start_IT(&hadc1);

    }
}

void Calculate_Spectrum(){
	// Prepare FFT input
	for (int i = 0; i < Number_Of_Samples; i++) {
		input_data_left[2 * i] = (float32_t)ADC_Values[2*i];  // Real part
		input_data_left[2 * i + 1] = 0;                    // Imaginary part

		input_data_right[2 * i] = (float32_t)ADC_Values[2 * i + 1]; // Right channel (real)
		input_data_right[2 * i + 1] = 0;


	}

	// Perform FFT
	arm_cfft_f32(&fft_instance, input_data_left, 0, 1);
	arm_cfft_f32(&fft_instance, input_data_right, 0, 1);

	// Compute Magnitude
	for (int i = 0; i < Number_Of_Samples; i++) {

		output_data_left[i] = sqrtf(input_data_left[2 * i] * input_data_left[2 * i] +
		input_data_left[2 * i + 1] * input_data_left[2 * i + 1]);

		output_data_right[i] = sqrtf(input_data_right[2 * i] * input_data_right[2 * i] +
		input_data_right[2 * i + 1] * input_data_right[2 * i + 1]);
	}

	int band_width = Number_Of_Samples / 2 / Number_Of_Bins;
	float32_t spectrum_min, spectrum_max;


    for (int i = 0; i < Number_Of_Bins; i++) {
        float32_t avg_left = 0;
        float32_t avg_right = 0;
        for (int j = 0; j < band_width; j++) {
        	avg_left += output_data_left[i * band_width + j];
        	avg_right += output_data_right[i * band_width + j];
        }

        avg_left /= band_width;
        avg_right /= band_width;

        spectrum[i] = logf(1.0f + avg_left);          // Spectrum for Left
        spectrum[i + Number_Of_Bins] = logf(1.0f + avg_right); // Spectrum for Right

                // Update min/max values for normalization
        if (spectrum[i] < spectrum_min) spectrum_min = spectrum[i];
        if (spectrum[i] > spectrum_max) spectrum_max = spectrum[i];
        if (spectrum[i + Number_Of_Bins] < spectrum_min) spectrum_min = spectrum[i + Number_Of_Bins];
        if (spectrum[i + Number_Of_Bins] > spectrum_max) spectrum_max = spectrum[i + Number_Of_Bins];
    }

    // Normalize for display
    float32_t range = spectrum_max - spectrum_min;
    if (range < 1e-6) range = 1.0f;  // Avoid division by zero
    for (int i = 0; i < Number_Of_Bins; i++) {
        spectrum[i] = (spectrum[i] - spectrum_min) * (16.0f / range);          // Normalize Left
        spectrum[i + Number_Of_Bins] = (spectrum[i + Number_Of_Bins] - spectrum_min) * (16.0f / range); // Normalize Right
    }

}

void Display_Spectrum(){
//	int Strength = 0;
//	HD44780_Clear();
//	for(int i = 0; i < Number_Of_Bins;i++){
//		Strength = (int)spectrum[i];
//		HD44780_SetCursor(i, 1);
//		if(Strength > 8){
//			HD44780_PrintSpecialChar(8);
//			HD44780_SetCursor(i, 0);
//			HD44780_PrintSpecialChar(Strength - 8);
//		}
//		else if(Strength > 0){
//			HD44780_PrintSpecialChar(Strength);
//		}
//	}

	int Strength1 = 0;
	int Strength2 = 0;

	    // Display Left Channel on I2C1
	    HD44780_SelectLCD(&hi2c1); // Select I2C1
	    HD44780_Clear();
	    for (int i = 0; i < Number_Of_Bins; i++) {
	        Strength1 = (int)spectrum[i];
	        HD44780_SetCursor(i, 1);
	        if (Strength1 > 8) {
	            HD44780_PrintSpecialChar(8);
	            HD44780_SetCursor(i, 0);
	            HD44780_PrintSpecialChar(Strength1 - 8);
	        } else if (Strength1 > 0) {
	            HD44780_PrintSpecialChar(Strength1);
	        }
	    }

	    // Display Right Channel on I2C2
	    HD44780_SelectLCD(&hi2c2); // Select I2C2
	    HD44780_Clear();
	    for (int i = 0; i < Number_Of_Bins; i++) {
	        Strength2 = (int)spectrum[i + Number_Of_Bins];
	        HD44780_SetCursor(i, 1);
	        if (Strength2 > 8) {
	            HD44780_PrintSpecialChar(8);
	            HD44780_SetCursor(i, 0);
	            HD44780_PrintSpecialChar(Strength2 - 8);
	        } else if (Strength2 > 0) {
	            HD44780_PrintSpecialChar(Strength2);
	        }
	    }

}

void Create_Special_Characters(){

  	uint8_t Level1[8] = {0,0,0,0,0,0,0,31};
  	uint8_t Level2[8] = {0,0,0,0,0,0,31,31};
  	uint8_t Level3[8] = {0,0,0,0,0,31,31,31};
  	uint8_t Level4[8] = {0,0,0,0,31,31,31,31};
  	uint8_t Level5[8] = {0,0,0,31,31,31,31,31};
  	uint8_t Level6[8] = {0,0,31,31,31,31,31,31};
  	uint8_t Level7[8] = {0,31,31,31,31,31,31,31};
  	uint8_t Level8[8] = {31,31,31,31,31,31,31,31};

  	HD44780_CreateSpecialChar(1, Level1);
  	HD44780_CreateSpecialChar(2, Level2);
  	HD44780_CreateSpecialChar(3, Level3);
  	HD44780_CreateSpecialChar(4, Level4);
  	HD44780_CreateSpecialChar(5, Level5);
  	HD44780_CreateSpecialChar(6, Level6);
  	HD44780_CreateSpecialChar(7, Level7);
  	HD44780_CreateSpecialChar(8, Level8);
}



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
