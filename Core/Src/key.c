//
// Created by Aaron Ge on 2023/3/27.
//

#include "key.h"
#include "stepper.h"
/* USER CODE BEGIN 0 */
#define KEYS_RCC_CLK_ENABLE           __HAL_RCC_GPIOE_CLK_ENABLE
#define KEYS_GPIO_PORT                GPIOE
#define FORWARD_GPIO_PIN              GPIO_PIN_2
#define REVERSE_GPIO_PIN              GPIO_PIN_3
#define EMERGENCY_GPIO_PIN            GPIO_PIN_4
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
// Declare the key state variables

GPIO_PinState last_state_forward;
GPIO_PinState last_state_reverse;
//GPIO_PinState last_state_emergency;
uint32_t last_tick;
uint8_t is_pressed;
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pins : PE2 PE3 PE4 */
    GPIO_InitStruct.Pin = FORWARD_GPIO_PIN|REVERSE_GPIO_PIN|EMERGENCY_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEYS_GPIO_PORT, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        GPIO_PinState state;
        uint32_t tick;
        case FORWARD_GPIO_PIN:
            state = HAL_GPIO_ReadPin(KEYS_GPIO_PORT, FORWARD_GPIO_PIN);
            tick = HAL_GetTick();

            if(state != last_state_forward){
                last_state_forward = state;
                last_tick = tick;
                is_pressed = (state == GPIO_PIN_RESET);
            } else if (tick-last_tick>=50 && state == GPIO_PIN_RESET){
                is_pressed = 1;
            }

            if(is_pressed != 1)
                stepper_normal_stop();
            else
                stepper_move_rel(128000, 5000, 5000, 12800);


            break;
        case GPIO_PIN_3:
            state = HAL_GPIO_ReadPin(KEYS_GPIO_PORT, FORWARD_GPIO_PIN);
            tick = HAL_GetTick();

            if(state != last_state_reverse){
                last_state_reverse = state;
                last_tick = tick;
                is_pressed = (state == GPIO_PIN_RESET);
            } else if (tick-last_tick>=50 && state == GPIO_PIN_RESET){
                is_pressed = 1;
            }

            if(is_pressed != 1)
                stepper_normal_stop();
            else
                stepper_move_rel(-128000, 5000, 5000, 12800);
            break;
        case GPIO_PIN_4:
            stepper_emergency_stop();
            HAL_NVIC_DisableIRQ(EXTI4_IRQn);
            HAL_NVIC_DisableIRQ(EXTI3_IRQn);
            HAL_NVIC_DisableIRQ(EXTI2_IRQn);
            stepper_move_rel(-128000, 50000, 50000, 128000);
            HAL_NVIC_EnableIRQ(EXTI4_IRQn);
            HAL_NVIC_EnableIRQ(EXTI3_IRQn);
            HAL_NVIC_EnableIRQ(EXTI2_IRQn);
            break;
        default:
            break;
    }
}