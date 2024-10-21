#include "servo.h"
#include "tim.h"
/**
 * @brief 舵机控制
 * 舵机信号口 ->    PA2     PA3     PA5       PA6     PA7 
 * 对应的label-   steer3  steer4  steer1    steer2   steer5 
 * 对应的时钟       TIM2    TIM2   TIM2      TIM13    TIM14
 * 通道             CH3     CH4    CH1        CH1     CH1
 * 
 * 已经设置了 周期为20ms 方便控制舵机 500->45°
 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
void servo_init(void)
{
    //开启时钟通道 然后控制时间
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

void servo_hands(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);
    HAL_Delay(100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 500);
}

void servo_head(void)
{
    __HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,1500);
    HAL_Delay(100);
    __HAL_TIM_SET_COMPARE(&htim13,TIM_CHANNEL_1,500);
}
