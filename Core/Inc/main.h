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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_OFF_Pin GPIO_PIN_13
#define PWR_OFF_GPIO_Port GPIOC
#define TOUCH_MISO_Pin GPIO_PIN_2
#define TOUCH_MISO_GPIO_Port GPIOC
#define TOUCH_MOSI_Pin GPIO_PIN_3
#define TOUCH_MOSI_GPIO_Port GPIOC
#define JOY_X_Pin GPIO_PIN_0
#define JOY_X_GPIO_Port GPIOA
#define JOY_Y_Pin GPIO_PIN_1
#define JOY_Y_GPIO_Port GPIOA
#define V_BAT_Pin GPIO_PIN_2
#define V_BAT_GPIO_Port GPIOA
#define DISP_DC_Pin GPIO_PIN_3
#define DISP_DC_GPIO_Port GPIOA
#define SOUND_WS_Pin GPIO_PIN_4
#define SOUND_WS_GPIO_Port GPIOA
#define DISP_SCK_Pin GPIO_PIN_5
#define DISP_SCK_GPIO_Port GPIOA
#define DISP_MISO_Pin GPIO_PIN_6
#define DISP_MISO_GPIO_Port GPIOA
#define DISP_MOSI_Pin GPIO_PIN_7
#define DISP_MOSI_GPIO_Port GPIOA
#define DISP_RST_Pin GPIO_PIN_4
#define DISP_RST_GPIO_Port GPIOC
#define IU_RX_Pin GPIO_PIN_5
#define IU_RX_GPIO_Port GPIOC
#define IU_TX_Pin GPIO_PIN_10
#define IU_TX_GPIO_Port GPIOB
#define TOUCH_CS_Pin GPIO_PIN_12
#define TOUCH_CS_GPIO_Port GPIOB
#define TOUCH_SCK_Pin GPIO_PIN_13
#define TOUCH_SCK_GPIO_Port GPIOB
#define TOUCH_IRQ_Pin GPIO_PIN_14
#define TOUCH_IRQ_GPIO_Port GPIOB
#define SOUND_MCK_Pin GPIO_PIN_7
#define SOUND_MCK_GPIO_Port GPIOC
#define EU_TX_Pin GPIO_PIN_9
#define EU_TX_GPIO_Port GPIOA
#define EU_RX_Pin GPIO_PIN_10
#define EU_RX_GPIO_Port GPIOA
#define SOUND_CK_Pin GPIO_PIN_10
#define SOUND_CK_GPIO_Port GPIOC
#define SOUND_SD_Pin GPIO_PIN_12
#define SOUND_SD_GPIO_Port GPIOC
#define BTN_1_Pin GPIO_PIN_4
#define BTN_1_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_5
#define BTN_2_GPIO_Port GPIOB
#define BTN_3_Pin GPIO_PIN_6
#define BTN_3_GPIO_Port GPIOB
#define BTN_4_Pin GPIO_PIN_7
#define BTN_4_GPIO_Port GPIOB
#define DISP_CS_Pin GPIO_PIN_8
#define DISP_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
