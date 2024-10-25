#include "chassis_task.h"
#include "mpu.h"
#include "motor.h"
#include "Grayscale_traces.h"
/* 速度常量的宏定义 */
void turn_to_white_line()
{
	//亮灯为0，所以我们讨论当亮灯个数大于三个时，则该情况下出现了交汇的
	if( sum_E < 5 )
	{
		if(E1 == 0 || E2 == 0  )
		{
			turn_right(-800,800);
			HAL_Delay(50);
		}
		else if(E8 == 0 || E7 == 0 
			)
		{
			turn_left(-800,800);
			HAL_Delay(50);
		}
		else if(sum_E >= 7)
		{
			back(100);
			HAL_Delay(50);
		}
		else if(sum_E >5 && E4 == 0 && E5==0)
		{
			go_forward(200,Straight_Slow); //���Բ��ü���������
			return;
		}
	}
}
