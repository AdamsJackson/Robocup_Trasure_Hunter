#include "mpu.h"
#include "main.h"
#include <stdint.h>
#include <math.h>
/**硬件层 - > PD8 PD9 作为发送 usart3
 * 得到的数据储存在my_95Q
 * 储存值分别为 roll pitch yaw level
 */
#define ROLL_L 0x14
#define ROLL_H 0X15
#define PITCH_L 0x16
#define PITCH_H 0X17
#define YAW_L 0x18
#define YAW_H 0X19

#define TEMP_L 0x1B
#define TEMP_H 0x1C

extern USART_HandleTypeDef husart3;
GY_95_t myGY95;


uint8_t uart_rx_buffer[30];

float angle[3];
uint8_t read_pre_1[4] = {0xA4, 0x06, 0x02, 0x03}; 
uint8_t read_pre_2[4] = {0xA4, 0x06, 0x01, 0x06}; 
uint8_t read_pre_3[4] = {0xA4, 0x06, 0x03, 0x01}; 
uint8_t read_pre_4[4] = {0xA4, 0x06, 0x04, 0x01};
uint8_t read_pre_5[4] = {0xA4, 0x06, 0x05, 0x57};
uint8_t read_wri[4] = {0xA4, 0x03, 0x08, 0x1B};  


void write_GY95T()
{
	HAL_USART_Transmit(&husart3,read_pre_1,sizeof(read_pre_1),HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_USART_Transmit(&husart3,read_pre_2,sizeof(read_pre_2),HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_USART_Transmit(&husart3,read_pre_3,sizeof(read_pre_3),HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_USART_Transmit(&husart3,read_pre_4,sizeof(read_pre_4),HAL_MAX_DELAY);
	HAL_Delay(10);
	HAL_USART_Transmit(&husart3,read_pre_5,sizeof(read_pre_5),HAL_MAX_DELAY);
	HAL_Delay(10);
}

void read_GY95T(GY_95_t *GY95)
{
		HAL_USART_Transmit(&husart3,read_wri,sizeof(read_wri),HAL_MAX_DELAY);
		//发送指令读取欧拉角
		//开始接受并解析数据
    HAL_USART_Receive(&husart3,uart_rx_buffer,sizeof(uart_rx_buffer),100);
		GY95->roll =     ((uart_rx_buffer[12] << 8) | uart_rx_buffer[13]) / 100.0f;
	  GY95->pitch =   ((uart_rx_buffer[14] << 8) | uart_rx_buffer[15]) / 100.0f;
	  GY95->yaw =       ((uart_rx_buffer[16] << 8) | uart_rx_buffer[17]) / 100.0f;
    GY95->leve = uart_rx_buffer[18];
		GY95->temp = ((uart_rx_buffer[20] << 8) | uart_rx_buffer[19]) / 100.0f;
//		GY95->roll = ((uart_rx_buffer[16] << 8) | uart_rx_buffer[17]) / 100.0f;
//    GY95->pitch = ((uart_rx_buffer[14] << 8) | uart_rx_buffer[15]) / 100.0f;
//    GY95->yaw = ((uart_rx_buffer[12] << 8) | uart_rx_buffer[13]) / 100.0f;
//    GY95->leve = uart_rx_buffer[11];
//    GY95->temp = ((uart_rx_buffer[9] << 8) | uart_rx_buffer[10]) / 100.0f;
    
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		read_GY95T(&myGY95);
}


