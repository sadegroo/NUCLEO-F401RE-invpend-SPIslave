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
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// #include <stdbool.h>
#include "l6474.h"
#include "x_nucleo_ihmxx.h"
#include "x_nucleo_ihm01a1_stm32f4xx.h"
#include "l6474_acceleration_control.h"
#include <chrono.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    uint32_t cnt3;                            // Counter 3
    int range_error;                          // Range error indicator
    float position;                   // Current encoder position
    int position_steps;               // Encoder position in steps
    int position_init;                // Initial encoder position
    int previous_position;            // Previous encoder position
    int max_position;                 // Maximum encoder position
    int global_max_position;          // Global maximum encoder position
    int prev_global_max_position;     // Previous global maximum encoder position
    int position_down;                // Downward encoder position
    int position_curr;                // Current encoder position in integer
    int position_prev;                // Previous encoder position in integer
    bool peaked;
    bool handled_peak;
    bool zero_crossed;
    const int counts_per_turn;		//
} Quadrature_Encoder_TypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define delayUS_ASM(us) do {\
		asm volatile (	"MOV R0,%[loops]\n\t"\
				"1: \n\t"\
				"SUB R0, #1\n\t"\
				"CMP R0, #0\n\t"\
				"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		);\
} while(0)

// program flow
#define START_STATE_MAIN 0

#define T_SAMPLE 0.001 // should match the sample time in MATLAB model

#define SPI_BUFFER_SIZE  12
#define UART_BUFFER_SIZE  100
#define UART_DECIMATION  2000

// encoder
#define COUNTS_PER_TURN 2400

// stepper motor
#define ACCELERATION_CONTROL
//#define VELOCITY_CONTROL // NOT IMPLEMENTED
//#define POSITION_CONTROL
#define MAX_SPEED 10000
#define MIN_SPEED 30
#define MAX_ACCEL 32767 //uint16 max
#define MAX_DECEL 32767 //uint16 max
#define MAX_TORQUE_CONFIG 800 					// 400 Selected Value for normal control operation
#define OVERCURRENT_THRESHOLD 2000				// 2000 Selected Value for Integrated Rotary Inverted Pendulum System
#define STEPS_PER_TURN 3200
#define ROTOR_ANGLE_READ_DECIMATION  1000000			// only read rotor every X cycles

#define MAX_DEFLECTION_REV 10000.0	// maximum deflection of the rotor before hard stop


#define __HAS_OPPOSITE_SIGNS(a, b) (((a) < 0) != ((b) < 0))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
//void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Error_Handler(uint16_t error);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
