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
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
//#include "heartbeat.h"
#include "processIMU.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// Global system state for ACS / TVC / Engine
typedef enum {
  SystemDisabled = 0,
  SystemManual,
  SystemAutomatic
} SubsystemState;

typedef float float32_t;


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define debug_TX_Pin GPIO_PIN_2
#define debug_TX_GPIO_Port GPIOA
#define debug_RX_Pin GPIO_PIN_3
#define debug_RX_GPIO_Port GPIOA
#define ADC_1_VC_Position_1_Pin GPIO_PIN_4
#define ADC_1_VC_Position_1_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define ADC_2_TVC_Position_2_Pin GPIO_PIN_6
#define ADC_2_TVC_Position_2_GPIO_Port GPIOA
#define ENG_TIM8_CH1N_Pin GPIO_PIN_7
#define ENG_TIM8_CH1N_GPIO_Port GPIOA
#define IMU_Reset_Pin GPIO_PIN_4
#define IMU_Reset_GPIO_Port GPIOC
#define IMU3_Rx_Pin GPIO_PIN_5
#define IMU3_Rx_GPIO_Port GPIOC
#define PR3_TIM3_CH3_Pin GPIO_PIN_0
#define PR3_TIM3_CH3_GPIO_Port GPIOB
#define PR4_TIM3_CH4_Pin GPIO_PIN_1
#define PR4_TIM3_CH4_GPIO_Port GPIOB
#define IMU3_Tx_Pin GPIO_PIN_10
#define IMU3_Tx_GPIO_Port GPIOB
#define TVC2_REV_Pin GPIO_PIN_12
#define TVC2_REV_GPIO_Port GPIOB
#define TVC2_FWD_Pin GPIO_PIN_13
#define TVC2_FWD_GPIO_Port GPIOB
#define TVC1_REV_Pin GPIO_PIN_14
#define TVC1_REV_GPIO_Port GPIOB
#define TVC1_FWD_Pin GPIO_PIN_15
#define TVC1_FWD_GPIO_Port GPIOB
#define IMU1_Tx_Pin GPIO_PIN_6
#define IMU1_Tx_GPIO_Port GPIOC
#define IMU1_Rx_Pin GPIO_PIN_7
#define IMU1_Rx_GPIO_Port GPIOC
#define heatbeat_TX_Pin GPIO_PIN_9
#define heatbeat_TX_GPIO_Port GPIOA
#define heatbeat_RX_Pin GPIO_PIN_10
#define heatbeat_RX_GPIO_Port GPIOA
#define IMU2_Tx_Pin GPIO_PIN_10
#define IMU2_Tx_GPIO_Port GPIOC
#define IMU2_Rx_Pin GPIO_PIN_11
#define IMU2_Rx_GPIO_Port GPIOC
#define ENG_TX_Pin GPIO_PIN_12
#define ENG_TX_GPIO_Port GPIOC
#define ENG_RX_Pin GPIO_PIN_2
#define ENG_RX_GPIO_Port GPIOD
#define PR1_TIM3_CH1_Pin GPIO_PIN_4
#define PR1_TIM3_CH1_GPIO_Port GPIOB
#define PR2_TIM3_CH2_Pin GPIO_PIN_5
#define PR2_TIM3_CH2_GPIO_Port GPIOB
#define YAW1_TIM2_CH1_Pin GPIO_PIN_6
#define YAW1_TIM2_CH1_GPIO_Port GPIOB
#define YAW2_TIM4_CH2_Pin GPIO_PIN_7
#define YAW2_TIM4_CH2_GPIO_Port GPIOB
#define LIDAR_SCL_Pin GPIO_PIN_8
#define LIDAR_SCL_GPIO_Port GPIOB
#define LIDAR_SDA_Pin GPIO_PIN_9
#define LIDAR_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
