#ifndef __motor_h__
#define __motor_h__

#include "main.h"
#include "mpu.h"
#include "pid.h"
#define MAX_sumERR 2000
extern void motor_init(void);
extern void motor_set_speed(int16_t speed1,int16_t speed2,int16_t speed3,int16_t speed4);
extern void go_forward(int16_t speed,PID_TYPE pid_mode);
extern void turn_right(int speed_l, int speed_r);
extern void turn_left(int speed_l, int speed_r);
extern void turn_left_angle(int speed, float angle,GY_95_t *GY95);
extern void stop(void);
extern void back(int speed_all);
extern void turn_right_angle(int speed, float angle,GY_95_t *GY95);
extern void turn_left_angle(int speed, float angle,GY_95_t *GY95);


#endif
