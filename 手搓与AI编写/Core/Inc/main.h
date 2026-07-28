/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define ADC_NTC1_Pin GPIO_PIN_2
#define ADC_NTC1_GPIO_Port GPIOA
#define ADC_NTC2_Pin GPIO_PIN_3
#define ADC_NTC2_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define ADC_IW_Pin GPIO_PIN_0
#define ADC_IW_GPIO_Port GPIOB
#define ADC_IV_Pin GPIO_PIN_1
#define ADC_IV_GPIO_Port GPIOB
#define ADC_IBUS_Pin GPIO_PIN_2
#define ADC_IBUS_GPIO_Port GPIOB
#define ADC_VBUS_Pin GPIO_PIN_11
#define ADC_VBUS_GPIO_Port GPIOB
#define LIN_W_Pin GPIO_PIN_13
#define LIN_W_GPIO_Port GPIOB
#define LIN_V_Pin GPIO_PIN_14
#define LIN_V_GPIO_Port GPIOB
#define LIN_U_Pin GPIO_PIN_15
#define LIN_U_GPIO_Port GPIOB
#define HIN_W_Pin GPIO_PIN_8
#define HIN_W_GPIO_Port GPIOA
#define HIN_V_Pin GPIO_PIN_9
#define HIN_V_GPIO_Port GPIOA
#define HIN_U_Pin GPIO_PIN_10
#define HIN_U_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_11
#define KEY1_GPIO_Port GPIOA
#define RGB_Pin GPIO_PIN_12
#define RGB_GPIO_Port GPIOA
#define MT6816_CSN_Pin GPIO_PIN_15
#define MT6816_CSN_GPIO_Port GPIOA
#define MT6816_SCK_Pin GPIO_PIN_3
#define MT6816_SCK_GPIO_Port GPIOB
#define MT6816_MISO_Pin GPIO_PIN_4
#define MT6816_MISO_GPIO_Port GPIOB
#define MT6816_MOSI_Pin GPIO_PIN_5
#define MT6816_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
