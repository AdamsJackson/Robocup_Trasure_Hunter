#include "pid.h"
#include "Grayscale_traces.h"
extern int E1,E2,E3,E4,E5,E6,E7,E8,sum_E;
extern flag_ST flag;
float error=0;

extern void Grayscale_Read(void);

float Hui_Error_return(void)
{
	Grayscale_Read();
	
	if(flag.pid_flag==Straight_Quick)//#############��������ֱ��Ѱ��#############
	{
		//��ƫ��
		if(E4==0&&E5==0) {error=0;}//1110 0111
		//0.5��ƫ��
		else if(E4==1&&E5==0){error=-0.5;}//1111 0111 ��ͷ��ƫΪ����  Change=0.5*Kp*speed
		else if(E4==0&&E5==1){error=0.5;} //1110 1111 ��ƫΪ����      Change>speedʱ���ӲŻᷴת
		
		//1��ƫ��
		else if(E5==0&&E6==0){error=-1;}//1111 0011 ��ͷ��ƫΪ����  Change=1*Kp*speed
		else if(E3==0&&E4==0){error=1;} //1100 1111 ��ƫΪ����
		
		//1.5��ƫ��
		else if(E5==1&&E6==0){error=-1.5;}//1111 1011 ��ͷ��ƫΪ����  Change=1.5*Kp*speed
		else if(E3==0&&E4==1){error=1.5;} //1101 1111 ��ƫΪ����
		
		//2��ƫ��
		else if(E6==0&&E7==0){error=-2;}//1111 1001 ��ͷ��ƫΪ����  Change=2*Kp*speed
		else if(E2==0&&E3==0){error=2;} //1001 1111 ��ƫΪ����
		
		//2.5��ƫ��
		else if(E6==1&&E7==0){error=-2.5;}//1111 1101 ��ͷ��ƫΪ����  Change=2.5*Kp*speed
		else if(E2==0&&E3==1){error=2.5;} //1011 1111 ��ƫΪ����
		
		//3��ƫ��
		else if(E7==0&&E8==0){error=-3;}//1111 1100 ��ͷ��ƫΪ����  Change=3*Kp*speed
		else if(E1==0&&E2==0){error=3;} //0011 1111 ��ƫΪ����
		
		//3.5��ƫ��
		else if(E7==1&&E8==0){error=-3.5;}//1111 1110 ��ͷ��ƫΪ����  Change=3.5*Kp*speed
		else if(E1==0&&E2==1){error=3.5;} //0111 1111 ��ƫΪ����
		
		else
		error=0;	
	}

	else if(flag.pid_flag==S_Curve)//#############����������#############
	{
		//��ƫ��
		if(E4==0&&E5==0) {error=0;}//1110 0111
		//0.5��ƫ��
		else if(E4==1&&E5==0){error=-0.5;}//1111 0111 ��ͷ��ƫΪ����  Change=0.5*Kp*speed
		else if(E4==0&&E5==1){error=0.5;} //1110 1111 ��ƫΪ����      Change>speedʱ���ӲŻᷴת
		
		//1��ƫ��
		else if(E5==0&&E6==0){error=-1;}//1111 0011 ��ͷ��ƫΪ����  Change=1*Kp*speed
		else if(E3==0&&E4==0){error=1;} //1100 1111 ��ƫΪ����
		
		//1.5��ƫ��
		else if(E5==1&&E6==0){error=-1.5;}//1111 1011 ��ͷ��ƫΪ����  Change=1.5*Kp*speed
		else if(E3==0&&E4==1){error=1.5;} //1101 1111 ��ƫΪ����
		
		//2��ƫ��
		else if(E6==0&&E7==0){error=-2;}//1111 1001 ��ͷ��ƫΪ����  Change=2*Kp*speed
		else if(E2==0&&E3==0){error=2;} //1001 1111 ��ƫΪ����
		
		//2.5��ƫ��
		else if(E6==1&&E7==0){error=-3;}//1111 1101 ��ͷ��ƫΪ����  Change=3*Kp*speed
		else if(E2==0&&E3==1){error=3;} //1011 1111 ��ƫΪ����
		
		//3��ƫ��
		else if(E7==0&&E8==0){error=-3.5;}//1111 1100 ��ͷ��ƫΪ����  Change=3.5*Kp*speed
		else if(E1==0&&E2==0){error=3.5;} //0011 1111 ��ƫΪ����
		
		//3.5��ƫ��
		else if(E7==1&&E8==0){error=-4.5;}//1111 1110 ��ͷ��ƫΪ����  Change=4.5*Kp*speed
		else if(E1==0&&E2==1){error=4.5;} //0111 1111 ��ƫΪ����
		
		else
		error=0;
	}
	
	else if(flag.pid_flag==Straight_Slow)//#############����������ֱ��#############
	{
		//��ƫ��
		if(E4==0&&E5==0) {error=0;}//1110 0111
		//0.5��ƫ��
		else if(E4==1&&E5==0){error=-0.5;}//1111 0111 ��ͷ��ƫΪ����  Change=0.5*Kp*speed
		else if(E4==0&&E5==1){error=0.5;} //1110 1111 ��ƫΪ����      Change>speedʱ���ӲŻᷴת
		
		//1��ƫ��
		else if(E5==0&&E6==0){error=-1;}//1111 0011 ��ͷ��ƫΪ����  Change=1*Kp*speed
		else if(E3==0&&E4==0){error=1;} //1100 1111 ��ƫΪ����
			
		else
		error=0;	
	}
	
	return error;
}

