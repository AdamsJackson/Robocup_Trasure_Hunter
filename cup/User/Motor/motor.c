#include "motor.h"
#include "main.h"
#include "math.h"
/**
 * @brief 
 *  说明一下用法，PE5 PE6 PB14 PB15 给PWM信号 
 *  PE2 PE4 PD11 PB13 给使能信号（高有效） 
 *  PE3 PF0  PD10 PD12 给方向信号（高低电平控制正反转）
 *  计数周期为1000
 *  
 */
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
void motor_init(void)
{
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);    //motor_pwm1
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);    //motor_pwm2
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);   //motor_pwm3
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);   //motor_pwm4
    //DISABLE
    HAL_GPIO_WritePin(MTEN1_GPIO_Port, MTEN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN2_GPIO_Port, MTEN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN3_GPIO_Port, MTEN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN4_GPIO_Port, MTEN4_Pin, GPIO_PIN_RESET);
    //direction
    HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

//电机(我们现在可以独立使用四个电机控制)
void motor_set_speed(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4)
{
    HAL_GPIO_WritePin(MTEN1_GPIO_Port, MTEN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTEN2_GPIO_Port, MTEN2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTEN3_GPIO_Port, MTEN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTEN4_GPIO_Port, MTEN4_Pin, GPIO_PIN_SET);
    if(speed1 > 0)
    {
        HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,speed1);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,-speed1);
    }
    if(speed2 > 0)
    {
        HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,speed2);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,-speed2);
    }
    if(speed3 > 0)
    {
        HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,speed3);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,-speed3);
    }
    if(speed4 > 0)
    {
        HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,speed4);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,-speed4);
    }
    
}

void go_forward(int16_t speed)
{
    motor_set_speed(speed,speed,speed,speed);
}
