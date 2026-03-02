/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Average size of the rolling average
#define AVG_SIZE 8
#define WHEEL_RADIUS_MM 325.5f //(DIMENSIONS IN MM)
#define PI 3.14159265f
#define magnet_count 1 //Accounts for the amount of magnets placed on the wheel
#define RPM_TO_KMH_FACTOR ((2.0f * PI * WHEEL_RADIUS_MM * 60.0f) / 1000000.0f)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
// Use volatile so the compiler knows it changes in an ISR, Flag Variables
volatile uint8_t magnet_flag = 0;
volatile uint32_t pulse_time = 0;

//Calculation variables
uint32_t time_diffs[AVG_SIZE] = {0};
float speed_kmh = 0.0f;
uint8_t avg_index = 0;
uint32_t last_time = 0;
uint32_t last_display_time = 0;
float filtered_rpm = 0.0f;
uint8_t samples_collected = 0;
uint32_t running_sum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //Initializes the first text to the I2C display
  ssd1306_Init();
  ssd1306_SetCursor(5,5);
  ssd1306_WriteString("SPEED: ", Font_11x18, White);
  ssd1306_SetCursor(5,24);
  ssd1306_WriteString("RPM: ", Font_11x18, White);
  ssd1306_UpdateScreen();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //----------------------------------------------------------
	  // 1. MAGNET FLAG PROCESSING
	  //----------------------------------------------------------

	  //Get the current time as a reference
	  uint32_t now = HAL_GetTick();

	  //If the flag for the magnet was triggered
	  if (magnet_flag)
	  {
		  // Snapshot the time safely and reset the flag ensuring no other magnet pulse affects it
		  __disable_irq();
		  uint32_t current_pulse = pulse_time;
		  magnet_flag = 0;
		  __enable_irq();

		  if (last_time != 0)
		  {
			  uint32_t diff = current_pulse - last_time;

			  // Debounce: Ignore pulses faster than 3000 RPM (20ms)
			  if (diff > 20)
			  {
				  // 1. Subtract the OLD value from the running sum
				  running_sum -= time_diffs[avg_index];

				  // 2. Store the NEW value in the circular buffer
				  time_diffs[avg_index] = diff;

	  			  // 3. Add the NEW value to the running sum
	  			  running_sum += time_diffs[avg_index];

	  			  avg_index = (avg_index + 1) % AVG_SIZE;

	  			  if (samples_collected < AVG_SIZE)
	  				  samples_collected++;
			  }
	  		}

	  		// ALWAYS update last_time so the next pulse has a reference
	  		last_time = current_pulse;
	  	  }

	  	//----------------------------------------------------------
	  	// 2. TIMED DISPLAY & CALCULATIONS (Every 300ms)
	  	//----------------------------------------------------------

	  	if (now - last_display_time >= 300)
	  	{
	  		last_display_time = now;

	  	     // Timeout: If no magnet seen for 2.5 seconds, we are stopped
	  	     if (last_time != 0 && (now - last_time > 2500))
	  	     {
	  	    	 filtered_rpm = 0.0f;
	  	    	 speed_kmh = 0.0f;
	  	         samples_collected = 0;
	  	         running_sum = 0;
	  	         last_time = 0; // Require a fresh first pulse to start again

	  	         for(int i = 0; i < AVG_SIZE; i++)
	  	        	 time_diffs[i] = 0;
	  	     }

	  	     else if (samples_collected > 0)
	  	     {
	  	         // Formula: (RPM * 2 * PI * R * 60) / 1,000,000
	  	    	 filtered_rpm = (60000.0f * (float)samples_collected) / (float)running_sum;
	  	         speed_kmh = filtered_rpm * RPM_TO_KMH_FACTOR;
	  	     }

	  	     // Update the OLED screen
	  	     char buffer[25];
	  	     sprintf(buffer, "%.2f km/h  ", speed_kmh);

	  	     ssd1306_SetCursor(68, 5);
	  	     ssd1306_WriteString(buffer, Font_11x18, White);

	  	     sprintf(buffer, "%.2f RPM  ", filtered_rpm);
	  	     ssd1306_SetCursor(45, 24);
	  	     ssd1306_WriteString(buffer, Font_11x18, White);
	  	     ssd1306_UpdateScreen();
	  	}

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : HALLEFFECT_PIN_Pin */
  GPIO_InitStruct.Pin = HALLEFFECT_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HALLEFFECT_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//---------------------------------------------------------
//CALLBACK LOGIC FOR THE MAGNET FLAG
//--------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == HALLEFFECT_PIN_Pin)
    {
        pulse_time = HAL_GetTick(); // Get the time the microsecond it happened
        magnet_flag = 1;            // Set the flag for the main loop

        // Quick visual feedback (Keep this very fast)
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
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
#ifdef USE_FULL_ASSERT
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
