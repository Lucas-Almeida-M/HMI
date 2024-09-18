/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Display_TASK_ */
osThreadId_t Display_TASK_Handle;
uint32_t Display_TASK_Buffer[ 8128 ];
osStaticThreadDef_t Display_TASK_ControlBlock;
const osThreadAttr_t Display_TASK__attributes = {
  .name = "Display_TASK_",
  .cb_mem = &Display_TASK_ControlBlock,
  .cb_size = sizeof(Display_TASK_ControlBlock),
  .stack_mem = &Display_TASK_Buffer[0],
  .stack_size = sizeof(Display_TASK_Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Button_TASK_ */
osThreadId_t Button_TASK_Handle;
uint32_t Button_TASK_Buffer[ 2048 ];
osStaticThreadDef_t Button_TASK_ControlBlock;
const osThreadAttr_t Button_TASK__attributes = {
  .name = "Button_TASK_",
  .cb_mem = &Button_TASK_ControlBlock,
  .cb_size = sizeof(Button_TASK_ControlBlock),
  .stack_mem = &Button_TASK_Buffer[0],
  .stack_size = sizeof(Button_TASK_Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for queue_buttons */
osMessageQueueId_t queue_buttonsHandle;
uint8_t queue_buttonsBuffer[ 20 * sizeof( uint8_t ) ];
osStaticMessageQDef_t queue_buttonsControlBlock;
const osMessageQueueAttr_t queue_buttons_attributes = {
  .name = "queue_buttons",
  .cb_mem = &queue_buttonsControlBlock,
  .cb_size = sizeof(queue_buttonsControlBlock),
  .mq_mem = &queue_buttonsBuffer,
  .mq_size = sizeof(queue_buttonsBuffer)
};
/* Definitions for sensors_queue */
osMessageQueueId_t sensors_queueHandle;
uint8_t sensors_queueBuffer[ 20 * sizeof( Sensors_Val ) ];
osStaticMessageQDef_t sensors_queueControlBlock;
const osMessageQueueAttr_t sensors_queue_attributes = {
  .name = "sensors_queue",
  .cb_mem = &sensors_queueControlBlock,
  .cb_size = sizeof(sensors_queueControlBlock),
  .mq_mem = &sensors_queueBuffer,
  .mq_size = sizeof(sensors_queueBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Display_TASK(void *argument);
void Button_TASK(void *argument);

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
  /* creation of queue_buttons */
  queue_buttonsHandle = osMessageQueueNew (20, sizeof(uint8_t), &queue_buttons_attributes);

  /* creation of sensors_queue */
  sensors_queueHandle = osMessageQueueNew (20, sizeof(Sensors_Val), &sensors_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Display_TASK_ */
  Display_TASK_Handle = osThreadNew(Display_TASK, NULL, &Display_TASK__attributes);

  /* creation of Button_TASK_ */
  Button_TASK_Handle = osThreadNew(Button_TASK, NULL, &Button_TASK__attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Display_TASK */
/**
* @brief Function implementing the Display_TASK_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_TASK */
__weak void Display_TASK(void *argument)
{
  /* USER CODE BEGIN Display_TASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Display_TASK */
}

/* USER CODE BEGIN Header_Button_TASK */
/**
* @brief Function implementing the Button_TASK_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Button_TASK */
__weak void Button_TASK(void *argument)
{
  /* USER CODE BEGIN Button_TASK */
  /* Infinite loop */

  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Button_TASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

