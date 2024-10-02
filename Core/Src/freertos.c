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
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
  .priority = (osPriority_t) osPriorityNormal1,
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
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for sensor_task_ */
osThreadId_t sensor_task_Handle;
uint32_t sensor_task_Buffer[ 512 ];
osStaticThreadDef_t sensor_task_ControlBlock;
const osThreadAttr_t sensor_task__attributes = {
  .name = "sensor_task_",
  .cb_mem = &sensor_task_ControlBlock,
  .cb_size = sizeof(sensor_task_ControlBlock),
  .stack_mem = &sensor_task_Buffer[0],
  .stack_size = sizeof(sensor_task_Buffer),
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for AlarmsTask_ */
osThreadId_t AlarmsTask_Handle;
uint32_t AlarmsTask_Buffer[ 512 ];
osStaticThreadDef_t AlarmsTask_ControlBlock;
const osThreadAttr_t AlarmsTask__attributes = {
  .name = "AlarmsTask_",
  .cb_mem = &AlarmsTask_ControlBlock,
  .cb_size = sizeof(AlarmsTask_ControlBlock),
  .stack_mem = &AlarmsTask_Buffer[0],
  .stack_size = sizeof(AlarmsTask_Buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for buttons_queue */
osMessageQueueId_t buttons_queueHandle;
uint8_t queue_buttonsBuffer[ 20 * sizeof( uint8_t ) ];
osStaticMessageQDef_t queue_buttonsControlBlock;
const osMessageQueueAttr_t buttons_queue_attributes = {
  .name = "buttons_queue",
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
/* Definitions for buzzerTimer */
osTimerId_t buzzerTimerHandle;
osStaticTimerDef_t buzzerTimerControlBlock;
const osTimerAttr_t buzzerTimer_attributes = {
  .name = "buzzerTimer",
  .cb_mem = &buzzerTimerControlBlock,
  .cb_size = sizeof(buzzerTimerControlBlock),
};
/* Definitions for display_mutex */
osMutexId_t display_mutexHandle;
osStaticMutexDef_t display_mutexControlBlock;
const osMutexAttr_t display_mutex_attributes = {
  .name = "display_mutex",
  .cb_mem = &display_mutexControlBlock,
  .cb_size = sizeof(display_mutexControlBlock),
};
/* Definitions for sensor_mutex */
osMutexId_t sensor_mutexHandle;
osStaticMutexDef_t sensor_mutexControlBlock;
const osMutexAttr_t sensor_mutex_attributes = {
  .name = "sensor_mutex",
  .cb_mem = &sensor_mutexControlBlock,
  .cb_size = sizeof(sensor_mutexControlBlock),
};
/* Definitions for buttonSemph */
osSemaphoreId_t buttonSemphHandle;
osStaticSemaphoreDef_t buttonSemphControlBlock;
const osSemaphoreAttr_t buttonSemph_attributes = {
  .name = "buttonSemph",
  .cb_mem = &buttonSemphControlBlock,
  .cb_size = sizeof(buttonSemphControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Display_TASK(void *argument);
void Button_TASK(void *argument);
void sensor_task(void *argument);
void AlarmsTask(void *argument);
void buzzerTimer_calbk(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of display_mutex */
  display_mutexHandle = osMutexNew(&display_mutex_attributes);

  /* creation of sensor_mutex */
  sensor_mutexHandle = osMutexNew(&sensor_mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of buttonSemph */
  buttonSemphHandle = osSemaphoreNew(1, 1, &buttonSemph_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of buzzerTimer */
  buzzerTimerHandle = osTimerNew(buzzerTimer_calbk, osTimerOnce, NULL, &buzzerTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of buttons_queue */
  buttons_queueHandle = osMessageQueueNew (20, sizeof(uint8_t), &buttons_queue_attributes);

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

  /* creation of sensor_task_ */
  sensor_task_Handle = osThreadNew(sensor_task, NULL, &sensor_task__attributes);

  /* creation of AlarmsTask_ */
  AlarmsTask_Handle = osThreadNew(AlarmsTask, NULL, &AlarmsTask__attributes);

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

/* USER CODE BEGIN Header_sensor_task */
/**
* @brief Function implementing the sensor_task_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensor_task */
__weak void sensor_task(void *argument)
{
  /* USER CODE BEGIN sensor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END sensor_task */
}

/* USER CODE BEGIN Header_AlarmsTask */
/**
* @brief Function implementing the AlarmsTask_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AlarmsTask */
__weak void AlarmsTask(void *argument)
{
  /* USER CODE BEGIN AlarmsTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AlarmsTask */
}

/* buzzerTimer_calbk function */
__weak void buzzerTimer_calbk(void *argument)
{
  /* USER CODE BEGIN buzzerTimer_calbk */

  /* USER CODE END buzzerTimer_calbk */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

