/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This codes writed by fstorm (Fikri Samet Güngör).
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
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define IDLE "idle"
#define CHARGE "charge"
#define DISCHARGE_M1 "discharge_m1"
#define DISCHARGE_M2 "discharge_m2"
#define NOS 100	// Number Of Sample
#define relayStatusChange(status) (relayStatus = status)
#define VREFIN_CAL ((uint16_t*)((uint32_t)0x1FFF7A2A))

typedef struct {
	float voltage;
	float current;
	double temp;
} batValue_t;

typedef enum {
	relayStatus_Idle = 0, relayStatus_Charge, relayStatus_Discharge_M1, relayStatus_Discharge_M2
} relayStatus_e;

typedef struct {
	uint8_t tx_data[25];
	char rx_data[20];
	bool received;
	uint16_t transtick;
} usart_t;
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
#define BAT_C_Pin GPIO_PIN_0
#define BAT_C_GPIO_Port GPIOA
#define BAT_TP_Pin GPIO_PIN_1
#define BAT_TP_GPIO_Port GPIOA
#define BAT_V_Pin GPIO_PIN_2
#define BAT_V_GPIO_Port GPIOA
#define RELAY_1_Pin GPIO_PIN_4
#define RELAY_1_GPIO_Port GPIOA
#define RELAY_2_Pin GPIO_PIN_5
#define RELAY_2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
