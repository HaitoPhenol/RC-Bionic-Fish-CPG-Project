/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define Infrared_L_Pin GPIO_PIN_1
#define Infrared_L_GPIO_Port GPIOA
#define Infrared_M_Pin GPIO_PIN_2
#define Infrared_M_GPIO_Port GPIOA
#define Infrared_R_Pin GPIO_PIN_3
#define Infrared_R_GPIO_Port GPIOA
#define Botton_1_Pin GPIO_PIN_4
#define Botton_1_GPIO_Port GPIOA
#define Botton_2_Pin GPIO_PIN_5
#define Botton_2_GPIO_Port GPIOA
#define Servo_1_Pin GPIO_PIN_6
#define Servo_1_GPIO_Port GPIOA
#define Servo_2_Pin GPIO_PIN_7
#define Servo_2_GPIO_Port GPIOA
#define Servo_3_Pin GPIO_PIN_0
#define Servo_3_GPIO_Port GPIOB
#define Servo_4_Pin GPIO_PIN_1
#define Servo_4_GPIO_Port GPIOB
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOB
#define RC_TX_Pin GPIO_PIN_9
#define RC_TX_GPIO_Port GPIOA
#define RC_RX_Pin GPIO_PIN_10
#define RC_RX_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
