/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include <math.h>
#include "stepper.h"

/* USER CODE BEGIN 0 */
#define STEPPER_PUL_PORT GPIOI
#define STEPPER_PUL_PIN (GPIO_PIN_5|GPIO_PIN_6)
#define STEPPER_DIR_PORT GPIOF
#define STEPPER_DIR_PIN (GPIO_PIN_12|GPIO_PIN_14)
#define STEPPER_EN_PORT GPIOF
#define STEPPER_EN_PIN (GPIO_PIN_13|GPIO_PIN_15)

#define STOP                                  0  // 加减速曲线状态：停止
#define RUN                                   1  // 加减速曲线状态：匀速运动
#define ACCEL                                 2  // 加减速曲线状态：匀速运动
#define DECEL                                 3 // 加减速曲线状态：减速阶段



#define STEPMOTOR_DIR_FORWARD()              HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_SET)
#define STEPMOTOR_DIR_REVERSAL()             HAL_GPIO_WritePin(STEPPER_DIR_PORT, STEPPER_DIR_PIN, GPIO_PIN_RESET)
#define STEPMOTOR_ENA_ENABLE()               HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN,  GPIO_PIN_SET)
#define STEPMOTOR_ENA_DISENABLE()            HAL_GPIO_WritePin(STEPPER_EN_PORT, STEPPER_EN_PIN,  GPIO_PIN_RESET)

#define T1_FREQ                               (SystemCoreClock/(7+1)) // 频率ft值
#define T1_FREQ_148                           ((float)((T1_FREQ*0.676)/10))  // 0.676为误差修正值
#define FSPR                                  200// 步进电机单圈步数  步距角:1.8° 360/1.8 = 200 正常情况下需要200步转一圈
#define MICRO_STEP                            32 // 步进电机驱动器细分数
#define SPR                                   (FSPR*MICRO_STEP)   // 旋转一圈需要的脉冲数
#define ALPHA                                 ((float)(2*3.14159/SPR))       // α= 2*pi/spr步距角
#define A_T_x10                               ((float)(10*ALPHA*T1_FREQ))

#define A_SQ                                  ((float)(2*100000*ALPHA))
#define A_x200                                ((float)(200*ALPHA))

/* USER CODE END 0 */

speed_ramp_data ramp_data = {0, 1, 0,0,0,0};
volatile uint32_t step_position = 0; //当前位置
volatile uint8_t motion_status = 0; //当前电机状态 0:停止 1:运行 2:加速 3:减速
volatile uint32_t total_step_count = 0;

TIM_HandleTypeDef htim_stepper;

/* TIM8 init function */
void MX_TIM8_Init(void)
{

    /* USER CODE BEGIN TIM8_Init 0 */

    /* USER CODE END TIM8_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM8_Init 1 */

    /* USER CODE END TIM8_Init 1 */
    htim_stepper.Instance = TIM8;
    htim_stepper.Init.Prescaler = 7;
    htim_stepper.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim_stepper.Init.Period = 65535;
    htim_stepper.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim_stepper.Init.RepetitionCounter = 0;
    htim_stepper.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_OC_Init(&htim_stepper) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim_stepper, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0xffff;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_OC_ConfigChannel(&htim_stepper, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_OC_ConfigChannel(&htim_stepper, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim_stepper, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM8_Interrupt */
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);


    /* USER CODE END TIM8_Interrupt */
    HAL_TIM_MspPostInit(&htim_stepper);

    HAL_TIM_OC_Stop_IT(&htim_stepper, TIM_CHANNEL_1);

    HAL_TIM_Base_Start(&htim_stepper);

}

void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* tim_ocHandle)
{

    if(tim_ocHandle->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
        /* TIM8 clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(timHandle->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

        __HAL_RCC_GPIOI_CLK_ENABLE();
        /**TIM8 GPIO Configuration
        PC6     ------> TIM8_CH1
        PC7     ------> TIM8_CH2
        */
        GPIO_InitStruct.Pin = STEPPER_PUL_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(STEPPER_PUL_PORT, &GPIO_InitStruct);

        /* USER CODE BEGIN Dir and Ena Initialization */
        __HAL_RCC_GPIOF_CLK_ENABLE();
        GPIO_InitStruct.Pin = STEPPER_DIR_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
        HAL_GPIO_Init(STEPPER_DIR_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = STEPPER_EN_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF0_TRACE;
        HAL_GPIO_Init(STEPPER_EN_PORT, &GPIO_InitStruct);
        /* USER CODE END Dir and Ena Initialization */

    }

}

void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef* tim_ocHandle)
{

    if(tim_ocHandle->Instance==TIM8)
    {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();
        /* USER CODE BEGIN TIM8_MspDeInit 1 */

        /* USER CODE END TIM8_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8)
    {
        volatile uint16_t tim_count = 0;

        //保存下一次的延时周期
        uint16_t new_step_delay = 0;
        volatile static uint16_t last_accel_delay = 0;
        //总移动步数计数

        //记录new_step_delay的计数的余数，提高精度
        volatile static uint32_t step_delay_remainder = 0;
        //定时器翻转标记
        volatile static uint8_t tim_flag = 0;

        if(__HAL_TIM_GET_IT_SOURCE(&htim_stepper, TIM_IT_CC1) != RESET) {
            __HAL_TIM_CLEAR_FLAG(&htim_stepper, TIM_FLAG_CC1);
            tim_count = __HAL_TIM_GET_COMPARE(&htim_stepper, TIM_CHANNEL_1);

            //设置比较值
//            __HAL_TIM_SET_COMPARE(&htim_stepper, TIM_CHANNEL_1, tim_count + ramp_data.step_delay);
            htim->Instance->CCR1 = tim_count+ramp_data.step_delay;
            tim_flag++;
            if(tim_flag == 2){
                tim_flag = 0;

                switch (ramp_data.run_status) {
                    case STOP:
                        HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
                        step_delay_remainder = 0;
                        total_step_count = 0;
                        last_accel_delay = 0;
                        ramp_data.accel_count = 0;
                        motion_status = STOP;
                        break;

                    case ACCEL:
                        total_step_count++;
                        if (ramp_data.dir = 1) // 正向运动
                            step_position++;
                        else
                            step_position--;

                        ramp_data.accel_count++;
                        new_step_delay = ramp_data.step_delay - (((2 * ramp_data.step_delay) + step_delay_remainder) /
                                                                 (4 * ramp_data.accel_count + 1));
                        step_delay_remainder =
                                ((2 * ramp_data.step_delay) + step_delay_remainder) % (4 * ramp_data.accel_count + 1);

                        if (total_step_count >= ramp_data.decel_start) {
                            ramp_data.accel_count = ramp_data.decel_val;
                            ramp_data.run_status = DECEL;
                        } else if (new_step_delay <= ramp_data.min_delay) {
                            ramp_data.accel_count = ramp_data.decel_val;
                            last_accel_delay = new_step_delay;
                            new_step_delay = ramp_data.min_delay;
                            step_delay_remainder = 0;
                            ramp_data.run_status = RUN;
                        }

                        last_accel_delay = new_step_delay;
                        break;

                    case RUN:
                        total_step_count++;
                        if (ramp_data.dir = 1) // 正向运动
                            step_position++;
                        else
                            step_position--;

                        new_step_delay = ramp_data.min_delay;
                        if (total_step_count >= ramp_data.decel_start) {
                            ramp_data.accel_count = ramp_data.decel_val;
                            ramp_data.run_status = DECEL;
                            new_step_delay = last_accel_delay;
                        }
                        break;

                    case DECEL:
                        total_step_count++;
                        if (ramp_data.dir = 1) // 正向运动
                            step_position++;
                        else
                            step_position--;

                        ramp_data.accel_count++;
                        new_step_delay = ramp_data.step_delay - (((2 * ramp_data.step_delay) + step_delay_remainder) /
                                                                 (4 * ramp_data.accel_count + 1));
                        step_delay_remainder =
                                ((2 * ramp_data.step_delay) + step_delay_remainder) % (4 * ramp_data.accel_count + 1);

                        if (ramp_data.accel_count >= 0)
                            ramp_data.run_status = STOP;

                        break;

                    default:
                        break;

                }

                ramp_data.step_delay = new_step_delay;
            }

        }
    }
}
/* USER CODE END 1 */

/* USER CODE BEGIN move relatively */
void stepper_move_rel(volatile int32_t step, volatile uint32_t accel, volatile uint32_t decel, volatile int32_t speed){
    if(ramp_data.run_status != STOP){
        return;
    }

    volatile uint16_t tim_count = 0;

    //达到做的速度时的步数
    volatile uint32_t max_s_lim = 0;
    //必须要开始减速的步数
    volatile uint32_t accel_lim = 0;

    if(step < 0){
        ramp_data.dir = -1;
        STEPMOTOR_DIR_REVERSAL();
        step = -step;
    } else{
        ramp_data.dir = 1;
        STEPMOTOR_DIR_FORWARD();
    }

    if (step == 1){
        ramp_data.accel_count = -1;
        ramp_data.run_status = DECEL;
        ramp_data.step_delay = 1000;
    } else if(step != 0){
        ramp_data.min_delay = (int32_t)(A_T_x10/speed);
        ramp_data.step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ/accel)) / 10); //C0, 初始速度的定时器

        // 计算多少步之后达到最大速度的限制
        // max_s_lim = speed^2 / (2*alpha*accel)
        max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel/10));
        if (max_s_lim == 0)
            max_s_lim = 1;

        accel_lim = (uint32_t)(step*decel/(accel+decel));
        if(accel_lim == 0)
            accel_lim = 1;

        if (accel_lim <= max_s_lim)
            ramp_data.decel_val = accel_lim - step;
        else
            ramp_data.decel_val = -(max_s_lim*accel/decel);

        if(ramp_data.decel_val == 0)
            ramp_data.decel_val = -1;

        ramp_data.decel_start = step + ramp_data.decel_val;

        if(ramp_data.step_delay <= ramp_data.min_delay)
        {
            ramp_data.step_delay = ramp_data.min_delay;
            ramp_data.run_status = RUN;
            motion_status = ACCEL;
        } else {
            ramp_data.run_status = ACCEL;
            motion_status = ACCEL;
        }

        ramp_data.accel_count = 0;

    }

    if (ramp_data.step_delay >= 65535)
        ramp_data.step_delay -= 65535;

    STEPMOTOR_ENA_ENABLE();
    tim_count = __HAL_TIM_GET_COUNTER(&htim_stepper);
    htim_stepper.Instance->CCR1 = tim_count + ramp_data.step_delay;
    HAL_TIM_OC_Start_IT(&htim_stepper, TIM_CHANNEL_1);
}

void stepper_normal_stop() {
    switch (ramp_data.run_status) {
        case STOP:
            break;
        case ACCEL:
            ramp_data.decel_start = total_step_count;
            break;
        case RUN:
            ramp_data.decel_start = total_step_count;
            break;
        case DECEL:
            break;

        default:
            break;

    }
}

void stepper_emergency_stop(){
    ramp_data.run_status = STOP;
    STEPMOTOR_ENA_DISENABLE();
    HAL_TIM_OC_Stop_IT(&htim_stepper, TIM_CHANNEL_1);
}

