/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MTEN1_Pin GPIO_PIN_2
#define MTEN1_GPIO_Port GPIOE
#define MTDIR1_Pin GPIO_PIN_3
#define MTDIR1_GPIO_Port GPIOE
#define MTEN2_Pin GPIO_PIN_4
#define MTEN2_GPIO_Port GPIOE
#define MOTOR_PWM1_Pin GPIO_PIN_5
#define MOTOR_PWM1_GPIO_Port GPIOE
#define MOTOR_PWM2_Pin GPIO_PIN_6
#define MOTOR_PWM2_GPIO_Port GPIOE
#define MTDIR2_Pin GPIO_PIN_0
#define MTDIR2_GPIO_Port GPIOF
#define PZ_Pin GPIO_PIN_4
#define PZ_GPIO_Port GPIOF
#define SW3_Pin GPIO_PIN_7
#define SW3_GPIO_Port GPIOF
#define SW4_Pin GPIO_PIN_9
#define SW4_GPIO_Port GPIOF
#define SW1_Pin GPIO_PIN_0
#define SW1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_2
#define SW2_GPIO_Port GPIOC
#define CODE_4A_Pin GPIO_PIN_0
#define CODE_4A_GPIO_Port GPIOA
#define CODE_4B_Pin GPIO_PIN_1
#define CODE_4B_GPIO_Port GPIOA
#define steer3_Pin GPIO_PIN_2
#define steer3_GPIO_Port GPIOA
#define steer4_Pin GPIO_PIN_3
#define steer4_GPIO_Port GPIOA
#define steer1_Pin GPIO_PIN_5
#define steer1_GPIO_Port GPIOA
#define steer2_Pin GPIO_PIN_6
#define steer2_GPIO_Port GPIOA
#define steer5_Pin GPIO_PIN_7
#define steer5_GPIO_Port GPIOA
#define sound1_Pin GPIO_PIN_0
#define sound1_GPIO_Port GPIOG
#define sound2_Pin GPIO_PIN_1
#define sound2_GPIO_Port GPIOG
#define CODE_2A_Pin GPIO_PIN_9
#define CODE_2A_GPIO_Port GPIOE
#define CODE_2B_Pin GPIO_PIN_11
#define CODE_2B_GPIO_Port GPIOE
#define MPU_SCL_Pin GPIO_PIN_10
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_11
#define MPU_SDA_GPIO_Port GPIOB
#define MTDIR4_Pin GPIO_PIN_12
#define MTDIR4_GPIO_Port GPIOB
#define MTEN4_Pin GPIO_PIN_13
#define MTEN4_GPIO_Port GPIOB
#define MOTOR_PWM4_Pin GPIO_PIN_14
#define MOTOR_PWM4_GPIO_Port GPIOB
#define MOTOR_PWM3_Pin GPIO_PIN_15
#define MOTOR_PWM3_GPIO_Port GPIOB
#define IMU_TX_Pin GPIO_PIN_8
#define IMU_TX_GPIO_Port GPIOD
#define IMU_RX_Pin GPIO_PIN_9
#define IMU_RX_GPIO_Port GPIOD
#define MTDIR3_Pin GPIO_PIN_10
#define MTDIR3_GPIO_Port GPIOD
#define MTEN3_Pin GPIO_PIN_11
#define MTEN3_GPIO_Port GPIOD
#define CODE_1B_Pin GPIO_PIN_12
#define CODE_1B_GPIO_Port GPIOD
#define CODE__1A_Pin GPIO_PIN_13
#define CODE__1A_GPIO_Port GPIOD
#define sound3_Pin GPIO_PIN_2
#define sound3_GPIO_Port GPIOG
#define sound4_Pin GPIO_PIN_3
#define sound4_GPIO_Port GPIOG
#define sound5_Pin GPIO_PIN_4
#define sound5_GPIO_Port GPIOG
#define sound6_Pin GPIO_PIN_5
#define sound6_GPIO_Port GPIOG
#define sound7_Pin GPIO_PIN_6
#define sound7_GPIO_Port GPIOG
#define sound8_Pin GPIO_PIN_7
#define sound8_GPIO_Port GPIOG
#define OMV_TX_Pin GPIO_PIN_6
#define OMV_TX_GPIO_Port GPIOC
#define OMV_RX_Pin GPIO_PIN_7
#define OMV_RX_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOA
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define HY8_Pin GPIO_PIN_1
#define HY8_GPIO_Port GPIOD
#define HY7_Pin GPIO_PIN_4
#define HY7_GPIO_Port GPIOD
#define HY6_Pin GPIO_PIN_7
#define HY6_GPIO_Port GPIOD
#define HY5_Pin GPIO_PIN_9
#define HY5_GPIO_Port GPIOG
#define SW5_Pin GPIO_PIN_10
#define SW5_GPIO_Port GPIOG
#define HY4_Pin GPIO_PIN_11
#define HY4_GPIO_Port GPIOG
#define SW6_Pin GPIO_PIN_12
#define SW6_GPIO_Port GPIOG
#define HY3_Pin GPIO_PIN_13
#define HY3_GPIO_Port GPIOG
#define SW7_Pin GPIO_PIN_14
#define SW7_GPIO_Port GPIOG
#define HY2_Pin GPIO_PIN_15
#define HY2_GPIO_Port GPIOG
#define SW8_Pin GPIO_PIN_3
#define SW8_GPIO_Port GPIOB
#define HY1_Pin GPIO_PIN_6
#define HY1_GPIO_Port GPIOB
#define GD1_Pin GPIO_PIN_8
#define GD1_GPIO_Port GPIOB
#define GD2_Pin GPIO_PIN_9
#define GD2_GPIO_Port GPIOB
#define PG3_Pin GPIO_PIN_0
#define PG3_GPIO_Port GPIOE
#define PG4_Pin GPIO_PIN_1
#define PG4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
extern uint16_t USART_ReceiveData(USART_TypeDef* USARTx);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
