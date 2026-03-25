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
#include "stm32f7xx_hal.h"

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
void	MX_CAN1_Init(void);
void	MX_CAN2_Init(void);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define gpio_TrackBoard2_8_Pin GPIO_PIN_2
#define gpio_TrackBoard2_8_GPIO_Port GPIOE
#define gpio_TrackBoard2_7_Pin GPIO_PIN_3
#define gpio_TrackBoard2_7_GPIO_Port GPIOE
#define gpio_TrackBoard2_6_Pin GPIO_PIN_4
#define gpio_TrackBoard2_6_GPIO_Port GPIOE
#define gpio_TrackBoard2_5_Pin GPIO_PIN_5
#define gpio_TrackBoard2_5_GPIO_Port GPIOE
#define gpio_TrackBoard2_4_Pin GPIO_PIN_6
#define gpio_TrackBoard2_4_GPIO_Port GPIOE
#define gpio_TrackBoard2_3_Pin GPIO_PIN_13
#define gpio_TrackBoard2_3_GPIO_Port GPIOC
#define gpio_TrackBoard2_2_Pin GPIO_PIN_14
#define gpio_TrackBoard2_2_GPIO_Port GPIOC
#define gpio_TrackBoard2_1_Pin GPIO_PIN_15
#define gpio_TrackBoard2_1_GPIO_Port GPIOC
#define gpio_TrackBoard1_8_Pin GPIO_PIN_0
#define gpio_TrackBoard1_8_GPIO_Port GPIOC
#define gpio_TrackBoard1_7_Pin GPIO_PIN_1
#define gpio_TrackBoard1_7_GPIO_Port GPIOC
#define gpio_TrackBoard1_6_Pin GPIO_PIN_2
#define gpio_TrackBoard1_6_GPIO_Port GPIOC
#define gpio_TrackBoard1_5_Pin GPIO_PIN_3
#define gpio_TrackBoard1_5_GPIO_Port GPIOC
#define gpio_TrackBoard1_4_Pin GPIO_PIN_4
#define gpio_TrackBoard1_4_GPIO_Port GPIOA
#define gpio_TrackBoard1_3_Pin GPIO_PIN_5
#define gpio_TrackBoard1_3_GPIO_Port GPIOA
#define gpio_TrackBoard1_2_Pin GPIO_PIN_6
#define gpio_TrackBoard1_2_GPIO_Port GPIOA
#define gpio_TrackBoard1_1_Pin GPIO_PIN_7
#define gpio_TrackBoard1_1_GPIO_Port GPIOA
#define gpio_TrackBoard3_1_Pin GPIO_PIN_0
#define gpio_TrackBoard3_1_GPIO_Port GPIOB
#define gpio_TrackBoard3_2_Pin GPIO_PIN_1
#define gpio_TrackBoard3_2_GPIO_Port GPIOB
#define gpio_TrackBoard3_3_Pin GPIO_PIN_9
#define gpio_TrackBoard3_3_GPIO_Port GPIOE
#define gpio_TrackBoard3_4_Pin GPIO_PIN_10
#define gpio_TrackBoard3_4_GPIO_Port GPIOE
#define gpio_TrackBoard3_5_Pin GPIO_PIN_11
#define gpio_TrackBoard3_5_GPIO_Port GPIOE
#define gpio_TrackBoard3_6_Pin GPIO_PIN_12
#define gpio_TrackBoard3_6_GPIO_Port GPIOE
#define gpio_TrackBoard3_7_Pin GPIO_PIN_13
#define gpio_TrackBoard3_7_GPIO_Port GPIOE
#define gpio_TrackBoard3_8_Pin GPIO_PIN_14
#define gpio_TrackBoard3_8_GPIO_Port GPIOE
#define gpio_TrackBoard_Home_L_Pin GPIO_PIN_15
#define gpio_TrackBoard_Home_L_GPIO_Port GPIOB
#define gpio_TrackBoard_Home_R_Pin GPIO_PIN_8
#define gpio_TrackBoard_Home_R_GPIO_Port GPIOD
#define Button1_Pin GPIO_PIN_10
#define Button1_GPIO_Port GPIOD
#define Button2_Pin GPIO_PIN_11
#define Button2_GPIO_Port GPIOD
#define Button3_Pin GPIO_PIN_12
#define Button3_GPIO_Port GPIOD
#define LEDR_Pin GPIO_PIN_11
#define LEDR_GPIO_Port GPIOC
#define LEDG_Pin GPIO_PIN_0
#define LEDG_GPIO_Port GPIOD
#define LEDB_Pin GPIO_PIN_1
#define LEDB_GPIO_Port GPIOD
#define gpio_HW_1_Pin GPIO_PIN_3
#define gpio_HW_1_GPIO_Port GPIOD
#define gpio_HW_2_Pin GPIO_PIN_4
#define gpio_HW_2_GPIO_Port GPIOD
#define gpio_HW_3_Pin GPIO_PIN_5
#define gpio_HW_3_GPIO_Port GPIOD
#define gpio_HW_4_Pin GPIO_PIN_6
#define gpio_HW_4_GPIO_Port GPIOD
#define gpio_HW_5_Pin GPIO_PIN_7
#define gpio_HW_5_GPIO_Port GPIOD
#define beebee_Pin GPIO_PIN_9
#define beebee_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
