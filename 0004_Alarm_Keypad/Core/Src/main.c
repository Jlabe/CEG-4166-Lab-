
#include "main.h"
#include "cmsis_os.h"
#include "Keypad4X4.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define MAX_KEYPAD_ENTRIES 6

QueueHandle_t xQueue;
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for DisplayOLEDTask */
osThreadId_t DisplayOLEDTaskHandle;
const osThreadAttr_t DisplayOLEDTask_attributes = {
  .name = "DisplayOLEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ReadKeypadTask */
osThreadId_t ReadKeypadTaskHandle;
const osThreadAttr_t ReadKeypadTask_attributes = {
  .name = "ReadKeypadTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for OLEDSemaphore */
osSemaphoreId_t OLEDSemaphoreHandle;
const osSemaphoreAttr_t OLEDSemaphore_attributes = {
  .name = "OLEDSemaphore"
};
/* USER CODE BEGIN PV */
extern char key;
char typedCode[MAX_KEYPAD_ENTRIES];
int typedCodeIndex = 0;

int alarmState = 0;

char passcode[] = "000000";
char actionCode = "\0";
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartDisplayOLEDTask(void *argument);
void StartReadKeypadTask(void *argument);
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

    SSD1306_Init();
  osKernelInitialize();
  OLEDSemaphoreHandle = osSemaphoreNew(1, 0, &OLEDSemaphore_attributes);
  DisplayOLEDTaskHandle = osThreadNew(StartDisplayOLEDTask, NULL, &DisplayOLEDTask_attributes);

  /* creation of ReadKeypadTask */
  ReadKeypadTaskHandle = osThreadNew(StartReadKeypadTask, NULL, &ReadKeypadTask_attributes);
  osKernelStart();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* D10 to D7 as input pins for row 0 to row 3. D6 to D3 as output for column pins C1 to C3*/
	  /*
	  key = Get_Key();
	  sprintf(hold, "%c", key);
	  //HAL_UART_Transmit(&huart2, (uint8_t *)hold, strlen(hold), 100);
	  SSD1306_GotoXY (0, 30);
	  SSD1306_UpdateScreen();
	  SSD1306_Puts (hold, &Font_11x18, 1);
	  SSD1306_UpdateScreen();
	  HAL_Delay (500);
	  */
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
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = KC0_Pin|KC3_Pin|KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);
}
void StartDisplayOLEDTask(void *argument)
{
  for(;;)
  {
	  //if this doesn't work do if(OLEDUpdate == 1){
	  osSemaphoreAcquire(OLEDSemaphoreHandle, osWaitForever);
	  SSD1306_Clear();
	  SSD1306_UpdateScreen();

	  SSD1306_GotoXY(0,0);

	  if(alarmState == 0){
		  SSD1306_Puts("Disarmed", &Font_11x18,1);
		  HAL_GPIO_WritePin(GPIOA,LDR_Pin, RESET);
		  HAL_GPIO_WritePin(GPIOA,LDG_Pin, SET);
	  }
	  else if(alarmState == 1){
		  SSD1306_Puts("Armed",&Font_11x18,1);
		  HAL_GPIO_WrtiePin(GPIOA, LDG_Pin, RESET);
		  HAL_GPIO_WritePin(GPIOA,LDR_Pin, SET);
	  }

	  SSD1306_GotoXY (0, 30);
	  SSD1306_Puts("Code:",&Font_11x18, 1);


	  //This null terminator is necessary for string-handling functions to know where the string ends.
	  char privacyBuffer[MAX_KEYPAD_ENTRIES+1];
	  for(int i = 0; i < typedCodeIndex; i++){
		  privacyBuffer[i] = '?';
	  }
	  privacyBuffer[typedCodeIndex] = '\0';

	  SSD1306_Puts(privacyBuffer, &Font_11x18, 1);
	  SSD1306_UpdateScreen();
	  osSemaphoreRelease(OLEDSemaphoreHandle);

	  //if it doesn't work use OLEDUpdate = 0;

	  osDelay(1);

  }
}
void StartReadKeypadTask(void *argument)
{
  for(;;)
  {
	key = Get_Key();
    if(key != '#' && key != '*'){
    	if(typedCodeIndex < MAX_KEYPAD_ENTRIES){
    		typedCode[typedCodeIndex] = key;
    		typedCodeIndex = typedCodeIndex+1;
    	}
    	else{
    		memset(typedCode,0,MAX_KEYPAD_ENTRIES);
    		typedCodeIndex = 0;
    	}
    }
    else{
		if(key == '#'){
			if(typedCodeIndex == 4 || typedCodeIndex == 5 || typedCodeIndex == 6){
				if(alarmState == 0){
					strcpy(passcode, typedCode);
					alarmState = 1;
					memset(typedCode,0,MAX_KEYPAD_ENTRIES);
					typedCodeIndex = 0;
				}
				else{
					if(strcmp(passcode, typedCode) == 0){
						alarmState = 0;
						memset(typedCode,0,MAX_KEYPAD_ENTRIES);
						typedCodeIndex = 0;
					}
				}
			}
		}
    }
	osSemaphoreRelease(OLEDSemaphoreHandle, osWaitForever);
	//If this doesn't work then do OLEDUpdate = 1;

	HAL_Delay (500);
	osDelay(1);

  }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
}
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
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
