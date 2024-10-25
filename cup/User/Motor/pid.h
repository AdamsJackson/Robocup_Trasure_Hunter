#ifndef __PID__H
#define __PID__H

#include "mpu.h"

typedef struct {
	float setpoint;  //目标设定值
	float Now_err;   //定义当前误差
	float Sum_err;   //总误差
	float Diff_err;  //微误差
	float Last_err;  //上一次误差
	float kp,ki,kd;  //定义比列，微分，积分常数
}PID;




/* Door_test模式的枚举 */
typedef enum  
{
	DT_Disable  = 0,
	DT_1        = 1,
	DT_2        = 2,
	DT_3      	= 3,
	DT_4        = 4,
}DT_TYPE;

extern float Hui_Error_return(void);

#endif
