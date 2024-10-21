#include "Grayscale_traces.h"
/**
 * @brief 读取灰度传感器数值
 * 先弄清楚引脚定义：
 *  HY1     HY2       HY3       HY4     HY5     HY6     HY7     HY8
 *  PB6     PG15      PG13      PG11    PG9     PD7     PD4     PD1
 */
int g1,g2,g3,g4,g5,g6,g7,g8,sum_g; // 8个传感器数据储存值
//根据引脚状态 reset = 0； set = 1. 
void Grayscale_Read(void)
{ 
    g1 = HAL_GPIO_ReadPin(HY1_GPIO_Port,HY1_Pin);
    g2 = HAL_GPIO_ReadPin(HY2_GPIO_Port,HY2_Pin);
    g3 = HAL_GPIO_ReadPin(HY3_GPIO_Port,HY3_Pin);
    g4 = HAL_GPIO_ReadPin(HY4_GPIO_Port,HY4_Pin);
    g5 = HAL_GPIO_ReadPin(HY5_GPIO_Port,HY5_Pin);
    g6 = HAL_GPIO_ReadPin(HY6_GPIO_Port,HY6_Pin);
    g7 = HAL_GPIO_ReadPin(HY7_GPIO_Port,HY7_Pin);
    g8 = HAL_GPIO_ReadPin(HY8_GPIO_Port,HY8_Pin);
    sum_g = g1+g2+g3+g4+g5+g6+g7+g8;
}

