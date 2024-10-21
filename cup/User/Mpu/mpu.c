#include "mpu.h"
#include "main.h"
#include <stdint.h>

/**硬件层 - > PD8 PD9 作为发送 usart3
 * 得到的数据储存在my_95Q
 * 储存值分别为 roll pitch yaw level
 */
extern USART_HandleTypeDef husart3;
uint8_t usart_rx_data[30]={0},Receive_ok=0;

gy my_95Q = {0,0,0,0};
void get_data(gy *gyro)
{
	uint8_t sum=0,i=0;
    HAL_Delay(300);//

    while(1)
    {
		if(Receive_ok==0)
		{		
			send_command(0xa4,0x06,0x03,0x01);//查询模式
		  HAL_Delay(3);
			send_command(0xa4,0x03,0x08,0x1b);//发送读取寄存器 ，起始0x08，数量0x1b， ->0x22
			HAL_Delay(10);
		}
        if(Receive_ok)  //接收完毕
        {

            for(sum=0,i=0; i<(usart_rx_data[3]+4); i++) //rgb_data[3]=3
                sum+=usart_rx_data[i];
            if(sum==usart_rx_data[i])//校验和判断
            {
                memcpy(&my_95Q,&usart_rx_data[4],sizeof(my_95Q));
                printf("roll:%.2f,",(float)my_95Q.roll/100.0f);
                printf("pitch:%.2f,",(float)my_95Q.pitch/100.0f);
                printf("yaw:%.2f,",(float)my_95Q.yaw/100);
                printf("temp:%.2f,",(float)my_95Q.temp/100.0f);
                printf("leve:%.2f\r\n ",(float)my_95Q.leve);
            }
            else
            {
                printf(" sum %d\r\n ",sum);
                printf(" count %d\r\n ",usart_rx_data[3]+4);
            }
            Receive_ok=0;//处理完成标志
        }
    }
	
}
void send_command(uint8_t addr,uint8_t ft_coade,uint8_t start_reg,uint8_t reg_number)
{
    uint8_t send_data[5]={addr,ft_coade,start_reg,reg_number,0};
    USART_Send(send_data,5);
}
void USART_Send(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		if(i<(Length-1))
		Buffer[Length-1]+=Buffer[i];  //累加数据
		USART3_send_byte(Buffer[i++]);
	}
}
void USART3_send_byte(uint8_t byte)
{
	while(__HAL_USART_GET_FLAG(&husart3,USART_FLAG_TC)==RESET);
	USART1->DR=byte;	
}

void USART_Send_bytes(uint8_t *Buffer, uint8_t Length)
{
	uint8_t i=0;
	while(i<Length)
	{
		USART3_send_byte(Buffer[i++]);
	}
}

void send_3out(uint8_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[30]={0},i=0,k=0;
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=send;//功能字节
	TX_DATA[i++]=length;//数据个数
	for(k=0;k<length;k++)//存储数据->TX_DATA
	{
		TX_DATA[i++]=(uint8_t)data[k];
	}
	USART_Send(TX_DATA,length+5);	
}
void send_out(int16_t *data,uint8_t length,uint8_t send)
{
	uint8_t TX_DATA[30],i=0,k=0;
	memset(TX_DATA,0,(2*length+5));//先清空数据
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=0X5A;//帧头
	TX_DATA[i++]=send;//功能字节
	TX_DATA[i++]=2*length;//数据个数
	for(k=0;k<length;k++)//存储数据->TX_DATA
	{
		TX_DATA[i++]=(uint16_t)data[k]>>8;
		TX_DATA[i++]=(uint16_t)data[k];
	}
	USART_Send(TX_DATA,2*length+5);	
}
void display(int16_t *num,uint8_t send,uint8_t count)
{
	  uint8_t i=0;
		USART3_send_byte(0X0d);
		USART3_send_byte(0X0a);
		USART3_send_byte(send);
	  USART3_send_byte(':');
		for(i=0;i<count;i++)
		{
			if(num[i]<0)
			{
				num[i]=-num[i];
				USART3_send_byte('-');
			}
			else
				USART3_send_byte('+');
		
			USART3_send_byte(0x30|(num[i]/10000));	
			USART3_send_byte(0x30|(num[i]%10000/1000));
			USART3_send_byte(0x30|(num[i]%1000/100));
			//USART_send_byte(0x2e);
			USART3_send_byte(0x30|(num[i]%100/10));
			USART3_send_byte(0x30|(num[i]%10));
			USART3_send_byte(',');	
	}
}

void CHeck(uint8_t *re_data)
{
	
}
