#include "Color_scales.h"
//PB8 PB9 PE0 PE1
//GD1 GD2 PG3 PG4 input mode
// only use PB8 PB9 ALWAYS   PE0 STAR_STATE
int middle,left,right;

int read_middle_sensor(void)
{
    return HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0);
}
int read_L_sensor(void)
{
    return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
    
}
int read_R_sensor(void)
{
    return HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9);
    
}