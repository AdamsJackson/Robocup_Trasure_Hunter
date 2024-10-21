#ifndef __MPU_H__
#define __MPU_H__

#include <stdint.h>
#include "stdio.h"
#include "string.h"
#include "main.h"



__packed typedef struct
{
    int16_t acc[3];
		int16_t gyro[3];
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t leve;
    int16_t temp;

} gy;

void mpu_init(void);
void mpu_read(gy *gyro);
void mpu_calibration(void);
void send_command(uint8_t addr,uint8_t ft_coade,uint8_t start_reg,uint8_t reg_number);
void USART3_send_byte(uint8_t byte);
void USART_Send(uint8_t *Buffer, uint8_t Length);






#endif
