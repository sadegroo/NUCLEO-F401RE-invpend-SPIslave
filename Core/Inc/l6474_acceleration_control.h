

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __L6474_ACCELERATION_CONTROL_H
#define __L6474_ACCELERATION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

// includes
#include <math.h>
#include "motor.h"
#include "l6474.h"
#include "x_nucleo_ihm01a1_stm32f4xx.h"
#include "x_nucleo_ihmxx.h"
#include <chrono.h>

// defines
#define ARM_MATH_CM4
#define ACCEL_CONTROL_DATA 0		// Set to 1 for display of timing data
#define PWM_COUNT_SAFETY_MARGIN 2

// Prescale a frequency in preparation for calculating counter period for PWM1
#define __L6474_Board_Pwm1PrescaleFreq(freq) (TIMER_PRESCALER * BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER * freq)

//typedefs
typedef struct {
	uint32_t desired_pwm_period;
	float desired_pwm_period_float;
	uint32_t current_pwm_period;
	float target_velocity; // can be negative
	float acceleration; // in microsteps/s^2
	float velocity;	// in microsteps/s
	uint32_t min_speed; // in microsteps/s
	uint32_t max_speed; // in microsteps/s
	uint32_t max_accel; // in microsteps/s^2
	motorDir_t old_dir;
	motorDir_t new_dir;
	float t_sample; // in seconds
	float speed; // always positive
	float speed_prescaled;
	uint8_t state;
	bool update_isr_flag;

} L6474_Acceleration_Control_TypeDef;

typedef struct {
	uint32_t min_speed;
	uint32_t max_speed;
	uint32_t max_accel;
	float t_sample;
} L6474_Acceleration_Control_Init_TypeDef;

// local function prototypes
void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams);
void SetAccel_L6472_Acceleration_Control(float acc);
uint8_t Run_L6472_Acceleration_Control(float acc);
void StepClockHandler_L6472_Acceleration_Control(void);
float GetSampleTime_L6472_Acceleration_Control(void);

// extern function prototypes
extern void L6474_StartMovement_InstantSteady(uint8_t deviceId);

#ifdef __cplusplus
}
#endif

#endif /* __L6474_ACCELERATION_CONTROL_H */
