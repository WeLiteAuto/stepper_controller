//
// Created by Aaron Ge on 2023/3/27.
//

#ifndef STEPPER_CONTROLLER_KEY_H
#define STEPPER_CONTROLLER_KEY_H
#include "stm32f4xx_hal.h"

typedef enum
{
    KEY_UP   = 0,
    KEY_DOWN = 1,
}KEYState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/







void MX_GPIO_Init(void);

#endif //STEPPER_CONTROLLER_KEY_H
