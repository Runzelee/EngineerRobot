/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "gimbal_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#include "bsp_Dbus.h"
#include "motor_can.h"
#include "motor_double_pid.h"
#include "unit_conversation.h"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Stop_All_Motors(void)
{
    Set_Motor_Torque(0, 0x1FF, 5);
    Set_Motor_Torque(0, 0x1FF, 6);
    Set_Motor_Torque(0, 0x1FF, 7);
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  DBus_Control_init();
  Enable_CAN1(0);

  /* 示例：为不同电机分配不同的双环 PID 参数（可选） */
  //DoublePID_SetParams(0, &double_pid_cfg_variant1);
  //DoublePID_SetParams(1, &double_pid_cfg_variant2);

  /* USER CODE END 2 */
  /* Init scheduler */
  //osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  
  /* USER CODE BEGIN WHILE */
  /* 使用新的 gimbal 控制模块，按题主要求在 main 中仅保留两段接口调用：
   * 1) 进入 reset 并完成全部 homing（顺序 lift->pitch->yaw），记录新坐标系零点
   * 2) 进入 ready，并让三台电机逐步到位（lift->pitch->yaw）
   * 具体行为与参数集中在 User/Inc/config.h 和 User/Src/config.c 中，便于调整。
   */
  Gimbal_Init();
/* 阻塞完成一次 reset（第一次 reset 完成后进入下面的检测循环） */
  if (!Gimbal_ResetAndBlock()) {
    /* reset 失败：进入安全态（把三台电机扭矩置零，并停在此处） */
    Stop_All_Motors();
    while (1) {
      HAL_Delay(1000);
    }
  }
  if (!Gimbal_ReadyAndBlock()) {
    /* ready 未到位或失败：保持安全扭矩（不强制死循环），继续检测 */
    Stop_All_Motors();
  }


  /*
   * 第一次 reset 完成后进入无限检测循环：
   * - rc_ctrl.s[1] == 1: 再次进入 reset（阻塞）
   * - rc_ctrl.s[1] == 2: 不动（保持当前状态）
   * - rc_ctrl.s[1] == 3: 进入 ready（阻塞）
   * 检测与动作完成后继续循环检测。
   */
  while (1)
  {
    uint8_t s = rc_ctrl.s[1];

    if (s == 1) {
      /* 要求重新 reset */
      if (!Gimbal_ResetAndBlock()) {
        /* reset 失败：进入安全态 */
        Stop_All_Motors();
        while (1) {
          HAL_Delay(1000);
        }
      }
    } else if (s == 3) {
      /* 要求进入 ready */
      if (!Gimbal_ReadyAndBlock()) {
        /* ready 未到位或失败：保持安全扭矩（不强制死循环），继续检测 */
        Stop_All_Motors();
      }
    } else {
      //Stop_All_Motors();
      /* s == 2 或 其他值：不动，保持当前状态 */
    }

    //Set_Motor_Torque(1000, 0x1FF, 5);

    /* 保留原有的周期性最小主循环工作（例如保持通訊或扭矩） */
    Set_Motor_Torque(1000, 0x1FF, 3);
    // if (rc_ctrl.s[1] == 1) {
    //     // 紧急停止：把电机扭矩置零
    //   Stop_All_Motors();
    // } else if(rc_ctrl.s[1] == 2) {
    //   /* 进入正常运行逻辑 - 这里保持最小化主逻辑，具体控制可在 gimbal_control 或其它模块实现 */
    //   if (!Gimbal_ReadyAndBlock()) {
    //     /* ready 未到位，依然进入主循环但保持安全扭矩 */
    //     Stop_All_Motors();
    //   }
    //   if (!Gimbal_ResetAndBlock()) {
    //     /* reset 失败：进入安全态（把三台电机扭矩置零，并停在此处） */
    //     Stop_All_Motors();
    //     while (1) {
    //       HAL_Delay(1000);
    //     }
    //   }
    // }
    HAL_Delay(10);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
