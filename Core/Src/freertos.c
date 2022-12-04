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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_LENGTH 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t rx_buffer[2];
uint8_t new_message_flag = 0;
uint8_t message_length;
extern UART_HandleTypeDef huart3;
uint8_t local_buffer[RX_BUFFER_LENGTH];
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
  CommandsToBlinkHandle = osMessageQueueNew (16, sizeof(uint8_t), &CommandsToBlink_attributes);

  /* creation of times_blinked */
  times_blinkedHandle = osMessageQueueNew (16, sizeof(uint8_t), &times_blinked_attributes);

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
	uint8_t local_pointer = 0;
	/* Infinite loop */
  for(;;)
  {
	 HAL_StatusTypeDef ret = HAL_UART_Receive(&huart3,rx_buffer ,1, 100);
	 if(ret == HAL_OK){
		 local_buffer[local_pointer] = rx_buffer[0];
		 if(local_buffer[local_pointer] == '\n' || local_buffer[local_pointer] == '\r'){
			 new_message_flag = 1;
			 local_buffer[local_pointer+1 ] = '\n';
			 message_length = local_pointer;
			 local_pointer = 0;
		 }else{
			 local_pointer++;
			 if(local_pointer > RX_BUFFER_LENGTH){
				 local_pointer = 0;
			 }
		 }
	 }
    osDelay(100);
  }
  osThreadDetach(ListenerHandle);
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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