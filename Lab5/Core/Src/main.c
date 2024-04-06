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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"

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

UART_HandleTypeDef huart2;

/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KeypadTask */
osThreadId_t KeypadTaskHandle;
const osThreadAttr_t KeypadTask_attributes = {
  .name = "KeypadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for PIRTask */
osThreadId_t PIRTaskHandle;
const osThreadAttr_t PIRTask_attributes = {
  .name = "PIRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lightsTask */
osThreadId_t lightsTaskHandle;
const osThreadAttr_t lightsTask_attributes = {
  .name = "lightsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OLEDSemaphore */
osSemaphoreId_t OLEDSemaphoreHandle;
const osSemaphoreAttr_t OLEDSemaphore_attributes = {
  .name = "OLEDSemaphore"
};
/* USER CODE BEGIN PV */
char hold[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartLCDTask(void *argument);
void StartkeypadTask(void *argument);
void StartPIRTask(void *argument);
void StartlightsTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_CODE_SIZE 6

extern char key;

char typedCode[MAX_CODE_SIZE]; //will be used to store current code being typed by user
int typedCodeIndex = 0;
char passcode[] = "000000"; //will be used to store the confirmed code

int alarmState = 0; //where 0 is disarmed, and 1 is armed

int LCDUpdateFlag = 1;
int LightsUpdateFlag = 1;
int sixtySecondsPassed = 0;
int alarmBuzzing = 0;
int counter = 60;
int isResetted = 0;
char snum[3];
int initialCoolDown = 1;
int motionDetected = 0;



void appendTypedCode(char c){
	if(typedCodeIndex < MAX_CODE_SIZE){
		typedCode[typedCodeIndex] = c;
		typedCodeIndex++;
	}else{
		//clearing typedCode variable
		memset(typedCode, 0, MAX_CODE_SIZE);
		typedCodeIndex = 0;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */
    SSD1306_Init();
//    SSD1306_GotoXY (0,0);
//    //SSD1306_Puts ("Voltage:", &Font_11x18, 1);
//    SSD1306_Puts ("Enter Code:", &Font_11x18, 1);
//    SSD1306_GotoXY (0, 30);
//    SSD1306_UpdateScreen();
//    SSD1306_UpdateScreen();
//    HAL_Delay (500);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of OLEDSemaphore */
  OLEDSemaphoreHandle = osSemaphoreNew(1, 0, &OLEDSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* creation of KeypadTask */
  KeypadTaskHandle = osThreadNew(StartkeypadTask, NULL, &KeypadTask_attributes);

  /* creation of PIRTask */
  PIRTaskHandle = osThreadNew(StartPIRTask, NULL, &PIRTask_attributes);

  /* creation of lightsTask */
  lightsTaskHandle = osThreadNew(StartlightsTask, NULL, &lightsTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* D10 to D7 as input pins for row 0 to row 3. D6 to D3 as output for column pins C1 to C3*/
//	  key = Get_Key();
//	  sprintf(hold, "%c", key);
//	  HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100);
//	  SSD1306_GotoXY (0, 30);
//	  SSD1306_UpdateScreen();
//	  SSD1306_Puts (hold, &Font_11x18, 1);
//	  SSD1306_UpdateScreen();
//	  HAL_Delay (500);
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
  hi2c1.Init.ClockSpeed = 400000;
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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KC0_Pin KC3_Pin KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KR1_Pin */
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KR0_Pin */
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartkeypadTask */
/**
* @brief Function implementing the keypadTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartkeypadTask */
void StartkeypadTask(void *argument)
{
  /* USER CODE BEGIN StartkeypadTask */
  /* Infinite loop */
  for(;;)
  {
	key = Get_Key();

//	actionCode = '\0';
	if(key == '#'){
//		actionCode = key;
		if(typedCodeIndex == 4 || typedCodeIndex == 5 || typedCodeIndex == 6){
			if(alarmState == 1){
				if(strcmp(passcode, typedCode) == 0){
					alarmState = 0;
					initialCoolDown = 1;
					motionDetected = 0;
					counter = 60;


					//clearing typed code
					memset(typedCode, 0, MAX_CODE_SIZE);
					typedCodeIndex = 0;
				} else{
//					char errMsg[25];
//					snprintf(errMsg, sizeof(errMsg), "[AUTH] - REQ: %s\n", passcode);
//					snprintf(errMsg, sizeof(errMsg), "[AUTH] - GOT: %s\n", typedCode);
					//clearing typed code
					memset(typedCode, 0, MAX_CODE_SIZE);
					typedCodeIndex = 0;
				}
			}
			else if(alarmState == 0){
				strcpy(passcode, typedCode);
				alarmState = 1;

				//clearing typed code
				memset(typedCode, 0, MAX_CODE_SIZE);
				typedCodeIndex = 0;
			}
		} else{ //invalid code length
			//clearing typed code
			memset(typedCode, 0, MAX_CODE_SIZE);
			typedCodeIndex = 0;
		}
	}else if (key != '*'){ //'*' is not alphanumeric
		appendTypedCode(key); //character is not '#', i.e. new char in typedCode

	}
	LCDUpdateFlag = 1;
	LightsUpdateFlag = 1;

	HAL_Delay(100);
    osDelay(1);
  }
  /* USER CODE END StartkeypadTask */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the LCDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
  /* Infinite loop */
  for(;;)
  {
	  if(LCDUpdateFlag == 1){
		  SSD1306_Clear();
		  SSD1306_UpdateScreen();

		  SSD1306_GotoXY(0,0);

		  sprintf(snum, "%d", counter);

		  if(alarmBuzzing == 1){
			  SSD1306_Puts("DANGER", &Font_11x18,1);
		  }
		  else{
			  if ((alarmState == 1 && initialCoolDown == 1) || (alarmState == 1 && initialCoolDown == 0 && motionDetected == 1)) {
				  SSD1306_Puts("Armed:", &Font_11x18,1);
				  SSD1306_Puts(snum, &Font_11x18, 1);
			  }

			  else if(alarmState == 1 && initialCoolDown == 0 && motionDetected == 0){
				  SSD1306_Puts("Armed", &Font_11x18,1);
			  }
			  else if(alarmState == 0){
				  SSD1306_Puts("Disarmed",&Font_11x18,1);

			  }

			  SSD1306_GotoXY (0, 30);
			  SSD1306_Puts("Code:",&Font_11x18, 1);


			  //This null terminator is necessary for string-handling functions to know where the string ends.
			  char starsToDisplay[MAX_CODE_SIZE+1];
			  for(int i = 0; i < typedCodeIndex; i++){
				  starsToDisplay[i] = '*';
			  }
			  starsToDisplay[typedCodeIndex] = '\0';

			  SSD1306_Puts(starsToDisplay, &Font_11x18, 1);

		  }
		  SSD1306_UpdateScreen();
		  LCDUpdateFlag = 0;


	}
    osDelay(1);
  }
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartlightsTask */
/**
* @brief Function implementing the lightsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartlightsTask */
void StartlightsTask(void *argument)
{
  /* USER CODE BEGIN StartlightsTask */
  /* Infinite loop */
  for(;;)
  {
	if(LightsUpdateFlag == 1){
		if(alarmState == 1){ //armed, red light should be on
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET); //turn off green LED
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);  //turn on red LED
		}else if(alarmState == 0){ //disarmed, green light should be on
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);  //turn on green LED
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET); //turn off red LED
		}
		LightsUpdateFlag = 0;
	}
    osDelay(1);
  }
  /* USER CODE END StartlightsTask */
}
void StartPIRTask(void *argument){
	for(;;){

		if(alarmState == 1){

			if (initialCoolDown == 1) {
				motionDetected = 0;
				counter = counter-1;
				if(counter <= 0){
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					osDelay(2000);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
					initialCoolDown = 0;
				}
				osDelay(1000);
				LCDUpdateFlag = 1;
			}

			else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && initialCoolDown == 0 && motionDetected == 0){
				motionDetected = 1;
				counter = 60;


			}
			else if(motionDetected == 1){
				LCDUpdateFlag = 1;
				counter = counter-1;
				if(counter <= 0){
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					alarmBuzzing = 1;
				}
				osDelay(1000);
				LCDUpdateFlag = 1;

			}

		}
		osDelay(1);

	}
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
