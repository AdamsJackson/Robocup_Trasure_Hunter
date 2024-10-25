#include "Grayscale_traces.h"
/**
 * @brief 读取灰度传感器数值
 * 先弄清楚引脚定义：
 *  HY1     HY2       HY3       HY4     HY5     HY6     HY7     HY8
 *  PB6     PG15      PG13      PG11    PG9     PD7     PD4     PD1
 */
int E1,E2,E3,E4,E5,E6,E7,E8,sum_E; // 8个传感器数据储存值


//根据引脚状态 reset = 0； set = 1. 
void Grayscale_Read(void)
{ 
    E1 = HAL_GPIO_ReadPin(E1_GPIO_Port,E1_Pin);
    E2 = HAL_GPIO_ReadPin(E2_GPIO_Port,E2_Pin);
    E3 = HAL_GPIO_ReadPin(E3_GPIO_Port,E3_Pin);
    E4 = HAL_GPIO_ReadPin(E4_GPIO_Port,E4_Pin);
    E5 = HAL_GPIO_ReadPin(E5_GPIO_Port,E5_Pin);
    E6 = HAL_GPIO_ReadPin(E6_GPIO_Port,E6_Pin);
    E7 = HAL_GPIO_ReadPin(E7_GPIO_Port,E7_Pin);
    E8 = HAL_GPIO_ReadPin(E8_GPIO_Port,E8_Pin);
    sum_E = E1+E2+E3+E4+E5+E6+E7+E8;
}
PID_TYPE Grayscale_flag_change(void)
{
	Grayscale_Read();
	if(sum_E==8)
	{
		return Disable;
	}
	else if(sum_E==7)
	{
		return Straight_Quick;
	}
	else if(sum_E==6)
	{
		return Straight_Slow;
	}
	else if(sum_E==5)
	{
		return S_Curve;
	}
	else if(sum_E<5)
	{
		return Disable;
	}
	return Disable;
}