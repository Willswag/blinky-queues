/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct command{
	uint8_t led;
	uint8_t brightness;
	uint32_t delay_ms;
}Command;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_LENGTH 100
#define DELAY_TRIGGER_WORD "delay"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
char rx_buffer[RX_BUFFER_LENGTH];
uint8_t new_message_flag = 0;
uint8_t message_length;
extern UART_HandleTypeDef huart3;
char local_buffer[RX_BUFFER_LENGTH];
/* USER CODE END Variables */
/* Definitions for CommandLine */
osThreadId_t CommandLineHandle;
const osThreadAttr_t CommandLine_attributes = {
  .name = "CommandLine",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Blinker1 */
osThreadId_t Blinker1Handle;
const osThreadAttr_t Blinker1_attributes = {
  .name = "Blinker1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Blinker2 */
osThreadId_t Blinker2Handle;
const osThreadAttr_t Blinker2_attributes = {
  .name = "Blinker2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Blinker3 */
osThreadId_t Blinker3Handle;
const osThreadAttr_t Blinker3_attributes = {
  .name = "Blinker3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CommandsToBlink */
osMessageQueueId_t CommandsToBlinkHandle;
const osMessageQueueAttr_t CommandsToBlink_attributes = {
  .name = "CommandsToBlink"
};
/* Definitions for times_blinked */
osMessageQueueId_t times_blinkedHandle;
const osMessageQueueAttr_t times_blinked_attributes = {
  .name = "times_blinked"
};
/* Definitions for rawCommands */
osMessageQueueId_t rawCommandsHandle;
const osMessageQueueAttr_t rawCommands_attributes = {
  .name = "rawCommands"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCommandLine(void *argument);
void StartBlinker1(void *argument);
void StartBlinker2(void *argument);
void StartBlinker3(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CommandsToBlink */
  CommandsToBlinkHandle = osMessageQueueNew (16, sizeof(uint16_t), &CommandsToBlink_attributes);

  /* creation of times_blinked */
  times_blinkedHandle = osMessageQueueNew (16, sizeof(uint16_t), &times_blinked_attributes);

  /* creation of rawCommands */
  rawCommandsHandle = osMessageQueueNew (16, sizeof(char), &rawCommands_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of CommandLine */
  CommandLineHandle = osThreadNew(StartCommandLine, NULL, &CommandLine_attributes);

  /* creation of Blinker1 */
  Blinker1Handle = osThreadNew(StartBlinker1, NULL, &Blinker1_attributes);

  /* creation of Blinker2 */
  Blinker2Handle = osThreadNew(StartBlinker2, NULL, &Blinker2_attributes);

  /* creation of Blinker3 */
  Blinker3Handle = osThreadNew(StartBlinker3, NULL, &Blinker3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCommandLine */
/**
 * @brief  Function implementing the CommandLine thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCommandLine */
void StartCommandLine(void *argument)
{
  /* USER CODE BEGIN StartCommandLine */
	uint8_t local_index = 0;
	char* delay_ptr = NULL;
	uint16_t delay_len = 0;
	uint16_t total_blinks = 0;
	osStatus_t status;
	char new_char;
	Command loc_com;
	/* Infinite loop */
	for(;;)
	{
		status = osMessageQueueGet(rawCommandsHandle, &new_char, NULL, 0);
		if(status == osOK)
		{
			rx_buffer[local_index] = new_char;
			local_index++;
			if(local_index == RX_BUFFER_LENGTH )
			{
				local_index = 0;
			}

			if(new_char == '\n' || new_char == '\r')
			{
				char* buff_ptr = rx_buffer;
				while(*buff_ptr != '\r'){
					switch (*buff_ptr) {
						case 'l':
							buff_ptr++;
							loc_com.led = atoi(buff_ptr);
							break;
						case 'b':
							buff_ptr++;
							loc_com.brightness = atoi(buff_ptr);
							break;
						case 'd':
							buff_ptr++;
							loc_com.delay_ms = atoi(buff_ptr);
							break;
						default:
							break;
					}
					buff_ptr++;
				}
				local_index = 0;
				memset(rx_buffer,0,RX_BUFFER_LENGTH);
			}
		}
		osDelay(10);
	}
	osThreadDetach(CommandLineHandle);
  /* USER CODE END StartCommandLine */
}

/* USER CODE BEGIN Header_StartBlinker1 */
/**
 * @brief Function implementing the Blinker1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinker1 */
void StartBlinker1(void *argument)
{
  /* USER CODE BEGIN StartBlinker1 */
	uint16_t blink_rate = 1000;
	uint16_t blinks = 0;
	/* Infinite loop */
	for(;;)
	{

		osMessageQueueGet(CommandsToBlinkHandle,&blink_rate , NULL, 0);
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		blinks++;
		if (blinks%100 == 0) {
			osMessageQueuePut(times_blinkedHandle, &blinks , NULL, 0);
		}
		osDelay(blink_rate);
	}
  /* USER CODE END StartBlinker1 */
}

/* USER CODE BEGIN Header_StartBlinker2 */
/**
 * @brief Function implementing the Blinker2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinker2 */
void StartBlinker2(void *argument)
{
  /* USER CODE BEGIN StartBlinker2 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartBlinker2 */
}

/* USER CODE BEGIN Header_StartBlinker3 */
/**
 * @brief Function implementing the Blinker3 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlinker3 */
void StartBlinker3(void *argument)
{
  /* USER CODE BEGIN StartBlinker3 */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
  /* USER CODE END StartBlinker3 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
