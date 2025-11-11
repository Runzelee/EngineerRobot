/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Chassis_Task */
osThreadId_t Chassis_TaskHandle;
const osThreadAttr_t Chassis_Task_attributes = {
  .name = "Chassis_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 1024
};
/* Definitions for Gimbal_Task */
osThreadId_t Gimbal_TaskHandle;
const osThreadAttr_t Gimbal_Task_attributes = {
  .name = "Gimbal_Task",
  .priority = (osPriority_t) osPriorityNormal1,
  .stack_size = 2048
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ChassisTask(void *argument);
void GimbalTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Chassis_Task */
  Chassis_TaskHandle = osThreadNew(ChassisTask, NULL, &Chassis_Task_attributes);

  /* creation of Gimbal_Task */
  Gimbal_TaskHandle = osThreadNew(GimbalTask, NULL, &Gimbal_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ChassisTask */
#ifdef __DISABLE_CHASSIS_TASK__
/**
  * @brief  Function implementing the Chassis_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisTask */
void ChassisTask(void *argument)
{
  /* USER CODE BEGIN ChassisTask */
  vTaskDelete(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/* USER CODE BEGIN Header_GimbalTask */
#endif /* __DISABLE_CHASSIS_TASK__ */
#ifdef __DISABLE_GIMBAL_TASK__
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalTask */
void GimbalTask(void *argument)
{
  /* USER CODE BEGIN GimbalTask */
  vTaskDelete(NULL);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GimbalTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

#endif /* __DISABLE_GIMBAL_TASK__ */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
