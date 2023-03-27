/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim_stepper;

/* USER CODE BEGIN Private defines */
typedef struct {
    __IO uint8_t  run_status ;  	// 电机旋转状态 0:停止 1:运行 2:加速 3:减速
    __IO int8_t   dir ;        	// 电机旋转方向 -1:反转 1:正转
    __IO uint32_t decel_start; 	// 启动减速位置
    __IO int32_t  step_delay;  	// 下个脉冲周期（时间间隔），启动时为加速度
    __IO int32_t  decel_val;   	// 减速阶段步数 （负值）
    __IO int32_t  min_delay;   	// 最小脉冲周期(最大速度，即匀速段速度)
    __IO int32_t  accel_count; 	// 加减速阶段计数值
}speed_ramp_data;

/* USER CODE END Private defines */

void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void stepper_move_rel(volatile int32_t step, volatile uint32_t accel, volatile uint32_t decel, volatile int32_t speed);
void stepper_normal_stop();
void stepper_emergency_stop();

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

