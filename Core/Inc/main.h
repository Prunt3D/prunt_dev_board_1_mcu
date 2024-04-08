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
#include "stm32g0xx_hal.h"

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
#define Stepper_2_Fault_Pin GPIO_PIN_13
#define Stepper_2_Fault_GPIO_Port GPIOC
#define Stepper_3_Fault_Pin GPIO_PIN_14
#define Stepper_3_Fault_GPIO_Port GPIOC
#define Stepper_1_Fault_Pin GPIO_PIN_15
#define Stepper_1_Fault_GPIO_Port GPIOC
#define Stepper_6_Fault_Pin GPIO_PIN_0
#define Stepper_6_Fault_GPIO_Port GPIOF
#define Stepper_6_En_Pin GPIO_PIN_1
#define Stepper_6_En_GPIO_Port GPIOF
#define ADC_Temp_1_Pin GPIO_PIN_0
#define ADC_Temp_1_GPIO_Port GPIOA
#define ADC_Temp_2_Pin GPIO_PIN_1
#define ADC_Temp_2_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_2
#define SPI1_MOSI_GPIO_Port GPIOA
#define Unused_PA3_Pin GPIO_PIN_3
#define Unused_PA3_GPIO_Port GPIOA
#define ADC_Temp_3_Pin GPIO_PIN_4
#define ADC_Temp_3_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define Fan_1_PWM_Pin GPIO_PIN_7
#define Fan_1_PWM_GPIO_Port GPIOA
#define Fan_2_PWM_Pin GPIO_PIN_0
#define Fan_2_PWM_GPIO_Port GPIOB
#define Fan_4_PWM_Pin GPIO_PIN_1
#define Fan_4_PWM_GPIO_Port GPIOB
#define ADC_Temp_4_Pin GPIO_PIN_2
#define ADC_Temp_4_GPIO_Port GPIOB
#define SPI1_CSN_Pin GPIO_PIN_10
#define SPI1_CSN_GPIO_Port GPIOB
#define Unused_PB11_Pin GPIO_PIN_11
#define Unused_PB11_GPIO_Port GPIOB
#define Stepper_3_En_Pin GPIO_PIN_12
#define Stepper_3_En_GPIO_Port GPIOB
#define Fan_PWM_OE_Pin GPIO_PIN_13
#define Fan_PWM_OE_GPIO_Port GPIOB
#define Fan_3_Tach_Pin GPIO_PIN_14
#define Fan_3_Tach_GPIO_Port GPIOB
#define Fan_3_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan_4_Tach_Pin GPIO_PIN_15
#define Fan_4_Tach_GPIO_Port GPIOB
#define Fan_4_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan_3_PWM_Pin GPIO_PIN_8
#define Fan_3_PWM_GPIO_Port GPIOA
#define Stepper_3_OE_Pin GPIO_PIN_9
#define Stepper_3_OE_GPIO_Port GPIOA
#define Fan_2_Tach_Pin GPIO_PIN_6
#define Fan_2_Tach_GPIO_Port GPIOC
#define Fan_2_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Fan_1_Tach_Pin GPIO_PIN_7
#define Fan_1_Tach_GPIO_Port GPIOC
#define Fan_1_Tach_EXTI_IRQn EXTI4_15_IRQn
#define Unused_PA10_Pin GPIO_PIN_10
#define Unused_PA10_GPIO_Port GPIOA
#define Stepper_1_En_Pin GPIO_PIN_11
#define Stepper_1_En_GPIO_Port GPIOA
#define Stepper_1_OE_Pin GPIO_PIN_12
#define Stepper_1_OE_GPIO_Port GPIOA
#define Unused_PA13_Pin GPIO_PIN_13
#define Unused_PA13_GPIO_Port GPIOA
#define Unused_PA14_Boot0_Pin GPIO_PIN_14
#define Unused_PA14_Boot0_GPIO_Port GPIOA
#define Stepper_2_OE_Pin GPIO_PIN_15
#define Stepper_2_OE_GPIO_Port GPIOA
#define Stepper_2_En_Pin GPIO_PIN_0
#define Stepper_2_En_GPIO_Port GPIOD
#define Stepper_4_En_Pin GPIO_PIN_1
#define Stepper_4_En_GPIO_Port GPIOD
#define Stepper_4_OE_Pin GPIO_PIN_2
#define Stepper_4_OE_GPIO_Port GPIOD
#define Stepper_5_En_Pin GPIO_PIN_3
#define Stepper_5_En_GPIO_Port GPIOD
#define Stepper_6_OE_Pin GPIO_PIN_3
#define Stepper_6_OE_GPIO_Port GPIOB
#define Stepper_5_OE_Pin GPIO_PIN_4
#define Stepper_5_OE_GPIO_Port GPIOB
#define Heater_En_Pin GPIO_PIN_5
#define Heater_En_GPIO_Port GPIOB
#define Stepper_5_Fault_Pin GPIO_PIN_8
#define Stepper_5_Fault_GPIO_Port GPIOB
#define Stepper_4_Fault_Pin GPIO_PIN_9
#define Stepper_4_Fault_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
