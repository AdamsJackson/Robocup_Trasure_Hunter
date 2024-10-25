#include "motor.h"
#include "main.h"
#include <math.h>
#include "pid.h"
#include "Color_scales.h"
/**
 * @brief 
 *  说明一下用法，PE5 PE6 PB14 PB15 给PWM信号 
 *  PE2 PE4 PD11 PB13 给使能信号（高有效） 
 *  PE3 PF0  PD10 PD12 给方向信号（高低电平控制正反转）
 *  计数周期为1000
 *  
 */
extern GY_95_t myGY95;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
PID pid_f={0,0,0,0,0,0,0,0};
float error1=0; 
extern flag_ST flag;
extern float Hui_Error_return(void);
float left_pwm=0,right_pwm=0;	
int S1,S2;
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
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,     1000 - speed1);
			
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,     1000 + speed1);
    }
    if(speed2 > 0)
    {
        HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,     1000 - speed2);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,     1000  +  speed2);
    }
    if(speed3 > 0)
    {
        HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,    1000 - speed3);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,    1000 + speed3);
    }
    if(speed4 > 0)
    {
        HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,    1000 - speed4);
    }
    else
    {
        HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_2,    1000 + speed4);
    }
    
}

void go_forward(int16_t speed,PID_TYPE pid_mode)
{
    
	float change = 0;
	flag.pid_flag = pid_mode;
	switch(flag.pid_flag)
	{
		case Straight_Quick:  pid_f.kp = 20;  pid_f.ki = 0;   pid_f.kd = 0;  break;
		case S_Curve			 :	pid_f.kp = 20;  pid_f.ki = 0;pid_f.kd = 0;  break;
		case Straight_Slow :	pid_f.kp = 10 * speed;  pid_f.ki = 0;pid_f.kd = 0;  break;
		default: break;
	}
	if(flag.pid_flag!=0)
	{
		error1 = Hui_Error_return();
		pid_f.Now_err = error1;
		pid_f.Sum_err += pid_f.Now_err;
		if(pid_f.Sum_err >= MAX_sumERR) {pid_f.Sum_err = MAX_sumERR;}//»ý·ÖÏÞ·ù
		pid_f.Diff_err = pid_f.Now_err - pid_f.Last_err; 
		pid_f.Last_err = pid_f.Now_err;
		//PID结果计算
		change = pid_f.kp * pid_f.Now_err
					 + pid_f.ki * pid_f.Sum_err 
					 + pid_f.kd * pid_f.Diff_err;
	}
	else if(flag.pid_flag==0)
	{
		error1=0;
		pid_f.Now_err = 0;
		pid_f.Last_err = 0;
		pid_f.Sum_err = 0;
		pid_f.Diff_err = 0;
		change = 0;
	}
	left_pwm  = speed + change;
    right_pwm = speed - change;
	motor_set_speed(left_pwm,left_pwm,right_pwm,right_pwm);
}
/**
*@轮 前 1 4
*@序 后 2 3
*/
void turn_left(int speed_l, int speed_r)
{	
		HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_SET);
		motor_set_speed(speed_l,speed_l,speed_r,speed_r);
}	
//转动固定角度
void turn_left_angle(int speed, float angle,GY_95_t *GY95)
{
	float ang_last;   //上一次角度
	float ang_now;		//当前角度
	float sum;				
	read_GY95T(GY95);
	ang_last = abs(GY95->yaw);
	while(sum < angle)
	{
		read_GY95T(GY95);
		ang_now = abs(GY95->yaw);
		sum = sum + (ang_now- ang_last);
		turn_left(500,-500);
		HAL_Delay(10);
	}
}

void turn_right(int speed_l, int speed_r)
{	
		HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_SET);
		motor_set_speed(speed_l,speed_l,speed_r,speed_r);
}	
void back(int speed_all)
{	
		HAL_GPIO_WritePin(MTDIR1_GPIO_Port, MTDIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTDIR2_GPIO_Port, MTDIR2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTDIR3_GPIO_Port, MTDIR3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTDIR4_GPIO_Port, MTDIR4_Pin, GPIO_PIN_RESET);
		motor_set_speed(speed_all,speed_all,speed_all,speed_all);
}	
//转动固定角度
void turn_right_angle(int speed, float angle,GY_95_t *GY95)
{
	float ang_last;   //上一次角度
	float ang_now;		//当前角度
	float sum;				
	read_GY95T(GY95);
	ang_last = abs(GY95->yaw);
	while(sum < angle)
	{
		read_GY95T(GY95);
		ang_now = abs(GY95->yaw);
		sum = sum + (ang_now- ang_last);
		turn_left(-500,500);
		HAL_Delay(10);
	}
}
void go_adjust(int initial_speed,AD_TYPE ad_mode)//¿¿É«±ê´«¸ÐÆ÷µ÷Õû
{
    S1=read_L_sensor();//left
    S2=read_R_sensor();//right
    flag.adjust_flag = ad_mode;
    
    if(flag.adjust_flag == AD_UpBridge)//
    {
        if(S1 == 1&&S2 == 1)              
        {
            motor_set_speed(initial_speed,initial_speed,initial_speed,initial_speed);
        }
        else if(S1 == 0)//right turn
        {
            motor_set_speed(initial_speed,initial_speed,-initial_speed,-initial_speed);
            HAL_Delay(5);//10
        }
        else if(S2 == 0)//left turn
        {
            motor_set_speed(-initial_speed,-initial_speed,initial_speed,initial_speed);
            HAL_Delay(5);//10
        }
    }
    
    else if(flag.adjust_flag == AD_OnBridge)//###############ÇÅÉÏ
    {
        if(S1 == 1&&S2 == 1)              
        {
            motor_set_speed(initial_speed,initial_speed,initial_speed,initial_speed);
        }
        else if(S1 == 0)//right turn
        {
            motor_set_speed(initial_speed,initial_speed,-initial_speed,-initial_speed);
            HAL_Delay(5);//10
        }
        else if(S2 == 0)//left turn
        {
            motor_set_speed(-initial_speed,-initial_speed,initial_speed,initial_speed);
            HAL_Delay(5);//10
        }
    }
    
    else if(flag.adjust_flag == AD_DownBridge)//###############ÏÂÇÅ
    {
        if(S1 == 1&&S2 == 1)              
        {
            motor_set_speed(initial_speed,initial_speed,initial_speed,initial_speed);
        }
        else if(S1 == 0)//right turn
        {
            motor_set_speed(initial_speed,initial_speed,-initial_speed,-initial_speed);
            HAL_Delay(5);//10
        }
        else if(S2 == 0)//left turn
        {
            motor_set_speed(-initial_speed,-initial_speed,initial_speed,initial_speed);
            HAL_Delay(5);//10
        }
    }
    
    else if(flag.adjust_flag == AD_Seesaw)//###############ÉÏõÎõÎ°å
    {
        if(S1 == 1&&S2 == 1)              
        {
            motor_set_speed(initial_speed,initial_speed,initial_speed,initial_speed);
        }
        else if(S1 == 0)//right turn
        {
            motor_set_speed(initial_speed,initial_speed,-initial_speed,-initial_speed);
            HAL_Delay(5);//10
        }
        else if(S2 == 0)//left turn
        {
            motor_set_speed(-initial_speed,-initial_speed,initial_speed,initial_speed);
            HAL_Delay(5);//10
        }
    }
}
void stop()
{
		HAL_GPIO_WritePin(MTEN1_GPIO_Port, MTEN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN2_GPIO_Port, MTEN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN3_GPIO_Port, MTEN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MTEN4_GPIO_Port, MTEN4_Pin, GPIO_PIN_RESET);

}
