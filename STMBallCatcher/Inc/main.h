/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stdarg.h>
void modprintf(const char* format, ...);
//#define DEBUG
//#define MOTOR_DEBUG
//#define BASE_MOTOR_ONLY
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MOTOR_MAX_PERIOD_COUNTS 0x1570
#define MOTOR_MIN_PERIOD_COUNTS_X 0x2300
#define MOTOR_MIN_PERIOD_COUNTS_Y 0x4300
#define MOTOR_CUTSPEED_X 0x6300
#define MOTOR_CUTSPEED_Y 0x6300
#define MOTOR_KP_X 1
#define MOTOR_KI_X 0
#define MOTOR_KD_X 0
#define MOTOR_KP_Y 1
#define MOTOR_KI_Y 0
#define MOTOR_KD_Y 0
#define DUMM 0
#define DUMM2 0
#define DUMM3 0
#define SERVO_CLOSED 1800
#define SERVO_OPEN 3600

#define BLUE_BUTTON_Pin GPIO_PIN_13
#define BLUE_BUTTON_GPIO_Port GPIOC
#define M0_1_Pin GPIO_PIN_0
#define M0_1_GPIO_Port GPIOA
#define M1_1_Pin GPIO_PIN_1
#define M1_1_GPIO_Port GPIOA
#define M2_1_Pin GPIO_PIN_4
#define M2_1_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOA
#define MOTOR_EN_1_Pin GPIO_PIN_0
#define MOTOR_EN_1_GPIO_Port GPIOB
#define DIR_2_Pin GPIO_PIN_10
#define DIR_2_GPIO_Port GPIOB
#define M1_2_Pin GPIO_PIN_7
#define M1_2_GPIO_Port GPIOC
#define MOTOR_EN_2_Pin GPIO_PIN_8
#define MOTOR_EN_2_GPIO_Port GPIOA
#define M2_2_Pin GPIO_PIN_9
#define M2_2_GPIO_Port GPIOA
#define DIR_1_Pin GPIO_PIN_10
#define DIR_1_GPIO_Port GPIOA
#define STEP_1_Pin GPIO_PIN_3
#define STEP_1_GPIO_Port GPIOB
#define STEP_2_Pin GPIO_PIN_4
#define STEP_2_GPIO_Port GPIOB
#define CALIBR_BUTTON_Pin GPIO_PIN_5
#define CALIBR_BUTTON_GPIO_Port GPIOB
#define M0_2_Pin GPIO_PIN_6
#define M0_2_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define STEP_CHANNEL_CENTRAL 	TIM_CHANNEL_1
#define STEP_CHANNEL_BASAL 		TIM_CHANNEL_2
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
