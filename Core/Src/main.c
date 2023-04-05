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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "eeprom.h"
#include "ask.h"
#include "ask_hal.h"
#include "stdbool.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t inputCode[3] = { 0 };
uint16_t SavedCode[2][3] = { 0 };
bool SaveFlag = 0, ResetFlag = 0;
uint16_t Select = 0;
uint32_t lastSendTime = 0UL;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_FLASH_Unlock();
	EE_Init();

	EE_ReadVariable(0, &Select);

	EE_ReadVariable(1, &SavedCode[0][0]);
	EE_ReadVariable(2, &SavedCode[0][1]);
	EE_ReadVariable(3, &SavedCode[0][2]);

	EE_ReadVariable(4, &SavedCode[1][0]);
	EE_ReadVariable(5, &SavedCode[1][1]);
	EE_ReadVariable(6, &SavedCode[1][2]);

	HAL_FLASH_Lock();

	ask433.fn_init_rx = ask_init_rx433;
	ask433.fn_init_tx = ask_init_tx433;
	ask433.fn_micros = ask_micros_433;
	ask433.fn_read_pin = ask_read_pin_433;
	ask433.fn_write_pin = ask_write_pin_433;
	ask433.fn_delay_ms = ask_delay_ms_433;
	ask433.fn_delay_us = ask_delay_us_433;
	ask_init(&ask433);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (!HAL_GPIO_ReadPin(Save_GPIO_Port, Save_Pin)) {
			lastSendTime = HAL_GetTick();
			while (!HAL_GPIO_ReadPin(Save_GPIO_Port, Save_Pin)) {
				if (HAL_GetTick() - lastSendTime > 3000) {
					for (uint8_t i = 0; i < 3; i++)
						SavedCode[0][i] = SavedCode[1][i] = 0;
					HAL_FLASH_Unlock();
					for (uint8_t i = 0; i < 7; i++) {
						EE_WriteVariable(i, 0);
						HAL_Delay(10);
					}
					HAL_FLASH_Lock();
					for (uint8_t i = 0; i < 3; i++) {
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
						HAL_Delay(50);
						HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
						HAL_Delay(50);
					}
					Select = 0;
					ResetFlag = 1;
					break;
				}
			}
			if (!ResetFlag)
				SaveFlag = 1;
			else
				ResetFlag = 0;
			while (!HAL_GPIO_ReadPin(Save_GPIO_Port, Save_Pin))
				;
		}

		if (ask_available(&ask433)) {
			ask_read_bytes(&ask433, inputCode);
			if (SaveFlag) {
				SaveFlag = 0;
				for (uint8_t i = 0; i < 3; i++)
					SavedCode[Select][i] = inputCode[i];

				HAL_FLASH_Unlock();

				EE_WriteVariable((Select * 3) + 1, SavedCode[Select][0]);
				HAL_Delay(10);
				EE_WriteVariable((Select * 3) + 2, SavedCode[Select][1]);
				HAL_Delay(10);
				EE_WriteVariable((Select * 3) + 3, SavedCode[Select][2]);
				HAL_Delay(10);

				HAL_FLASH_Lock();

				if (Select >= 1) {
					Select = 0;
					HAL_FLASH_Unlock();
					EE_WriteVariable(0, Select);
					HAL_Delay(10);
					HAL_FLASH_Lock();
				} else {
					Select++;
					HAL_FLASH_Unlock();
					EE_WriteVariable(0, Select);
					HAL_Delay(10);
					HAL_FLASH_Lock();
				}
				for (uint8_t i = 0; i < 3; i++) {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					HAL_Delay(100);
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
					HAL_Delay(100);
				}
			} else {
				if ((inputCode[0] == SavedCode[0][0])
						&& (inputCode[1] == SavedCode[0][1])
						&& (inputCode[2] == SavedCode[0][2])) {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 0);
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 1);
				}
				if ((inputCode[0] == SavedCode[1][0])
						&& (inputCode[1] == SavedCode[1][1])
						&& (inputCode[2] == SavedCode[1][2])) {
					HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
					HAL_GPIO_WritePin(Relay_GPIO_Port, Relay_Pin, 0);
				}
			}
			ask_wait(&ask433);
		}
		if (SaveFlag && HAL_GetTick() - lastSendTime > 500) {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			lastSendTime = HAL_GetTick();
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
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
