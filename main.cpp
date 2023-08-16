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
#include <stdio.h>
#include "string.h"
#include "stdint.h"
#include "bmm150.h"
#include "bmm150_defs.h"
#include <cmath>
#include <queue>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float value_offset_x = 0;
float value_offset_y = 0;
float value_offset_z = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BMM150 bmm = BMM150();






void calibrate(uint32_t timeout, I2C_HandleTypeDef* hi2c1) {
    int16_t value_x_min = 0;
    int16_t value_x_max = 0;
    int16_t value_y_min = 0;
    int16_t value_y_max = 0;
    int16_t value_z_min = 0;
    int16_t value_z_max = 0;
    uint32_t timeStart = 0;

    bmm.read_mag_data(hi2c1);
    value_x_min = bmm.raw_mag_data.raw_datax;
    value_x_max = bmm.raw_mag_data.raw_datax;
    value_y_min = bmm.raw_mag_data.raw_datay;
    value_y_max = bmm.raw_mag_data.raw_datay;
    value_z_min = bmm.raw_mag_data.raw_dataz;
    value_z_max = bmm.raw_mag_data.raw_dataz;
    HAL_Delay(100);

    timeStart = HAL_GetTick();

    while ((HAL_GetTick() - timeStart) < timeout) {
        bmm.read_mag_data(hi2c1);

        /* Update x-Axis max/min value */
        if (value_x_min > bmm.raw_mag_data.raw_datax) {
            value_x_min = bmm.raw_mag_data.raw_datax;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < bmm.raw_mag_data.raw_datax) {
            value_x_max = bmm.raw_mag_data.raw_datax;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > bmm.raw_mag_data.raw_datay) {
            value_y_min = bmm.raw_mag_data.raw_datay;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < bmm.raw_mag_data.raw_datay) {
            value_y_max = bmm.raw_mag_data.raw_datay;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > bmm.raw_mag_data.raw_dataz) {
            value_z_min = bmm.raw_mag_data.raw_dataz;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < bmm.raw_mag_data.raw_dataz) {
            value_z_max = bmm.raw_mag_data.raw_dataz;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }

        //Serial.print(".");
        HAL_Delay(100);

    }

    value_offset_x = value_x_min + (value_x_max - value_x_min) / 2;
    value_offset_y = value_y_min + (value_y_max - value_y_min) / 2;
    value_offset_z = value_z_min + (value_z_max - value_z_min) / 2;
}







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    char message[100];
    sprintf(message, "Program starting... \r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if (bmm.initialize(&hi2c1) == BMM150_E_ID_NOT_CONFORM)
  {
	    sprintf(message, "Initialization Failed: Chip ID can not read! \r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
      while (1);
  } else
  {
	    sprintf(message, "Initialization Successful... \r\n");
	    HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
  }

  sprintf(message, "Calibration in 3 seconds...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);

  calibrate(10000, &hi2c1);

  sprintf(message, "Calibration Complete...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);

  std::queue<float> queue_X;
  std::queue<float> queue_Y;
  std::queue<float> queue_Z;

  float average_X = 0;
  float average_Y = 0;
  float average_Z = 0;
  int num_samples = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  sprintf(message, "Compass Running...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)&message, strlen(message), 0xFFFF);
  while (1)
  {
	char data_stream[100];

	bmm150_mag_data value;
	bmm.read_mag_data(&hi2c1);

	value.x = bmm.raw_mag_data.raw_datax - value_offset_x;
	value.y = bmm.raw_mag_data.raw_datay - value_offset_y;
	value.z = bmm.raw_mag_data.raw_dataz - value_offset_z;

	queue_X.push(value.x);
	queue_Y.push(value.y);
	queue_Z.push(value.z);


	//sprintf(data_stream, "x: %d  y: %d  z: %d \r\n", value.x, value.y, value.z);
	//HAL_UART_Transmit(&huart2, (uint8_t*)&data_stream, strlen(data_stream), 0xFFFF);
	HAL_Delay(50);

	if(num_samples == 100)
	{
		//will be a sum then an average
		average_X = 0;
		average_Y = 0;
		average_Z = 0;

		while(!queue_X.empty())
		{
			average_X += queue_X.front();
			queue_X.pop();
		}
		average_X = average_X /100;

		while(!queue_Y.empty())
		{
			average_Y += queue_Y.front();
			queue_Y.pop();
		}
		average_Y = average_Y /100;

		while(!queue_Z.empty())
		{
			average_Z += queue_Z.front();
			queue_Z.pop();
		}
		average_Z = average_Z /100;



		float xyHeading = atan2(average_X, average_Y);
		float zxHeading = atan2(average_Z, average_X);
		float heading = xyHeading;

		if (heading < 0) {
			heading += 2 * M_PI;
		}
		if (heading > 2 * M_PI) {
			heading -= 2 * M_PI;
		}

		float headingDegrees = heading * 180 / M_PI;
		float xyHeadingDegrees = xyHeading * 180 / M_PI;
		float zxHeadingDegrees = zxHeading * 180 / M_PI;

		char RX_Buffer_Char[100];
		sprintf(RX_Buffer_Char, "Hdg deg: %f  X-Y Hdg: %f  Z-X Hdg: %f \r\n", headingDegrees,xyHeadingDegrees, zxHeadingDegrees);
		HAL_UART_Transmit(&huart2, (uint8_t*)&RX_Buffer_Char, strlen(RX_Buffer_Char), 0xFFFF);

		num_samples = 0;
	}
	num_samples++;





//	    float xyHeading = atan2(value.x, value.y);
//	    float zxHeading = atan2(value.z, value.x);
//	    float heading = xyHeading;
//
//	    if (heading < 0) {
//	        heading += 2 * M_PI;
//	    }
//	    if (heading > 2 * M_PI) {
//	        heading -= 2 * M_PI;
//	    }
//	    float headingDegrees = heading * 180 / M_PI;
//	    float xyHeadingDegrees = xyHeading * 180 / M_PI;
//	    float zxHeadingDegrees = zxHeading * 180 / M_PI;
//	    samples++;
//
//	    char RX_Buffer_Char[100];
//	    //sprintf(RX_Buffer_Char, "Hdg deg: %f  X-Y Hdg: %f  Z-X Hdg: %f \r\n", headingDegrees,xyHeadingDegrees, zxHeadingDegrees);
//
//	    HAL_UART_Transmit(&huart2, (uint8_t*)&RX_Buffer_Char, strlen(RX_Buffer_Char), 0xFFFF);
//	    HAL_Delay(100);



//	    float xyHeading = atan2(value.x, value.y);
//	    float zxHeading = atan2(value.z, value.x);
//	    float heading = xyHeading;
//
//	    if (heading < 0) {
//	        heading += 2 * M_PI;
//	    }
//	    if (heading > 2 * M_PI) {
//	        heading -= 2 * M_PI;
//	    }
//	    float headingDegrees = heading * 180 / M_PI;
//	    float xyHeadingDegrees = xyHeading * 180 / M_PI;
//	    float zxHeadingDegrees = zxHeading * 180 / M_PI;
//	    samples++;
//
//	    char RX_Buffer_Char[100];
//	    //sprintf(RX_Buffer_Char, "Hdg deg: %f  X-Y Hdg: %f  Z-X Hdg: %f \r\n", headingDegrees,xyHeadingDegrees, zxHeadingDegrees);
//
//	    HAL_UART_Transmit(&huart2, (uint8_t*)&RX_Buffer_Char, strlen(RX_Buffer_Char), 0xFFFF);
//	    HAL_Delay(100);

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.OwnAddress1 = 0x0A;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
