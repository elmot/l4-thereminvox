/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"

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
DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

OPAMP_HandleTypeDef hopamp2;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_DAC1_Init(void);

static void MX_I2C1_Init(void);

static void MX_OPAMP2_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int volume = 500;

int __unused _write(int __unused file, char *data, int len) {
    return HAL_UART_Transmit(&huart2, (uint8_t *) data, len, 1000) == HAL_OK ? len : 0;
}

VL53L1_Dev_t leftSensor = {
        .I2cDevAddr = 0x62,
        .new_data_ready_poll_duration_ms = 1000,
};

VL53L1_Dev_t centerSensor = {
        .I2cDevAddr = 0x52,
        .new_data_ready_poll_duration_ms = 1000,
};

void setupSensor(VL53L1_DEV dev) {
    uint16_t wordData;
    uint8_t byteData;
    __unused int status = 0;

/* Those basic I2C read functions can be used to check your own I2C functions */
    status += VL53L1_RdByte(dev, 0x010F, &byteData);
    printf("VL53L1X Model_ID: %X\n", byteData);
    status += VL53L1_RdByte(dev, 0x0110, &byteData);
    printf("VL53L1X Module_Type: %X\n", byteData);
    status += VL53L1_RdWord(dev, 0x010F, &wordData);
    printf("VL53L1X: %X\n", wordData);
    status += VL53L1_WaitDeviceBooted(dev);
    printf("Chip booted\n");

    /* This function must to be called to initialize the sensor with the default setting  */
    status += VL53L1_DataInit(dev);
    status += VL53L1_StaticInit(dev);
    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status += VL53L1_SetPresetMode(dev, VL53L1_PRESETMODE_LITE_RANGING);
    status += VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
    status += VL53L1_SetMeasurementTimingBudgetMicroSeconds(dev, 10000);
    status += VL53L1_SetInterMeasurementPeriodMilliSeconds(dev, 0);


//  status = VL53L1X_SetOffset(dev,20); /* offset compensation in mm */
//  status = VL53L1X_SetROI(dev, 16, 16); /* minimum ROI 4,4 */
//	status = VL53L1X_CalibrateOffset(dev, 140, &offset); /* may take few second to perform the offset cal*/
//	status = VL53L1X_CalibrateXtalk(dev, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    status += VL53L1_StartMeasurement(dev);   /* This function has to be called to enable the ranging */

    if (status) Error_Handler();
}

#define MIN_DISTANCE 70
#define DISTANCE_RANGE 320
#define MAX_VOLUME 1000
#define MIN_FREQ 20
#define MAX_FREQ 1200

static uint32_t phase64K = 0;
static volatile uint32_t phase64KDelta = 0x20000;

__unused void TIM8_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
    phase64K = (phase64K + phase64KDelta) % (1000 * 0x10000);
    int value = 2048 + (AUDIO_SAMPLE[phase64K / 0x10000] * volume / MAX_VOLUME);
    DAC1->DHR12L2 = value;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */
    __unused int status = 0;

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
    MX_DAC1_Init();
    MX_I2C1_Init();
    MX_OPAMP2_Init();
    MX_USART2_UART_Init();
    MX_TIM8_Init();
    /* USER CODE BEGIN 2 */
    XNUCLEO53L1A1_Init();
    status += XNUCLEO53L1A1_ResetId('c', 0); // Shutdown center sensor

    status += XNUCLEO53L1A1_ResetId('l', 0); // Reset left sensor
    HAL_Delay(2);
    status += XNUCLEO53L1A1_ResetId('l', 1);
    HAL_Delay(2);
    VL53L1_SetDeviceAddress(&centerSensor, leftSensor.I2cDevAddr); //Set left sensor to secondary address

    status += XNUCLEO53L1A1_ResetId('c', 1); // run center sensor
    HAL_Delay(2);

    setupSensor(&leftSensor);
    setupSensor(&centerSensor);
    HAL_OPAMP_Start(&hopamp2);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim8);
    if (status) Error_Handler();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        VL53L1_RangingMeasurementData_t data;

        status += VL53L1_WaitMeasurementDataReady(&centerSensor);
        status += VL53L1_GetRangingMeasurementData(&centerSensor, &data);
        VL53L1_ClearInterruptAndStartMeasurement(&centerSensor);
        if (data.RangeStatus == 0) {
            int d = (data.RangeMilliMeter - MIN_DISTANCE);
            volume = d * d * MAX_VOLUME /
                     (DISTANCE_RANGE * DISTANCE_RANGE);
            if (volume > MAX_VOLUME) volume = MAX_VOLUME;
            else if (volume < 0) volume = 0;
        } else {
            volume = 0;
        }

        double freq;
        status += VL53L1_WaitMeasurementDataReady(&leftSensor);
        status += VL53L1_GetRangingMeasurementData(&leftSensor, &data);
        VL53L1_ClearInterruptAndStartMeasurement(&leftSensor);
        if (data.RangeStatus != 0) {
            volume = 0;
            freq = 0;
        } else {
            int d = (data.RangeMilliMeter - MIN_DISTANCE);
            freq = MIN_FREQ + (double) d * d * (MAX_FREQ - MIN_FREQ) /
                              (DISTANCE_RANGE * DISTANCE_RANGE);
            if (freq > MAX_FREQ) {
                freq = MAX_FREQ;//NOLINT
            } else if (freq < 20) {
                volume = 0;
            }
            phase64KDelta = (65536.0 / 48 * freq);
        }
        if (status) Error_Handler();
    }
#pragma clang diagnostic pop
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }
    /** Enable MSI Auto calibration
    */
    HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void) {

    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */
    /** DAC Initialization
    */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }
    /** DAC channel OUT1 config
    */
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /** DAC channel OUT2 config
    */
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */

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
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
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
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void) {

    /* USER CODE BEGIN OPAMP2_Init 0 */

    /* USER CODE END OPAMP2_Init 0 */

    /* USER CODE BEGIN OPAMP2_Init 1 */

    /* USER CODE END OPAMP2_Init 1 */
    hopamp2.Instance = OPAMP2;
    hopamp2.Init.PowerSupplyRange = OPAMP_POWERSUPPLY_HIGH;
    hopamp2.Init.Mode = OPAMP_FOLLOWER_MODE;
    hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC_CH;
    hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
    hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN OPAMP2_Init 2 */

    /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void) {

    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_DOWN;
    htim8.Init.Period = 1666;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Init 2 */

    /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : VL53L1X_INT_Pin */
    GPIO_InitStruct.Pin = VL53L1X_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(VL53L1X_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
