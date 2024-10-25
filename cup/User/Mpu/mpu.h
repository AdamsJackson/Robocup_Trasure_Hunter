#ifndef __MPU_H__
#define __MPU_H__

#include <stdint.h>
#include "stdio.h"
#include "string.h"
#include "main.h"




typedef struct {
    int16_t acc[3];
	  int16_t gyro[3]; 
    float roll;
    float pitch;
    float yaw;
    uint8_t leve;
    float temp;
} GY_95_t;

extern void read_GY95T(GY_95_t *GY95);
void pre_read_GY95T(void);









#endif
