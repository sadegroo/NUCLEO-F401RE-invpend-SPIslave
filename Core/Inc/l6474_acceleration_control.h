

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

// defines
#define RCC_SYS_CLOCK_FREQ 84000000 // should equal HAL_RCC_GetSysClockFreq()
#define RCC_HCLK_FREQ 84000000 // should equal HAL_RCC_GetHCLKFreq()
// acceleration control
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
	uint32_t apply_acc_start_time;
	uint32_t clock_int_time;
	uint32_t clock_int_tick;
	float target_velocity_prescaled;
	float acceleration; // in microsteps/s^2
	float velocity;	// in microsteps/s
	uint32_t max_speed; // in microsteps/s
	uint32_t max_accel; // in microsteps/s^2
	uint32_t max_decel; // in microsteps/s^2
	float t_sample; // in seconds
	motorDir_t old_dir;
	motorDir_t new_dir;
	float speed_prescaled;
	uint8_t firstcall_ok;

} L6474_Acceleration_Control_TypeDef;

typedef struct {
	uint32_t max_speed;
	uint32_t max_accel;
	uint32_t max_decel;
	float t_sample;
} L6474_Acceleration_Control_Init_TypeDef;

// local function prototypes
void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams);
void Update_L6472_Acceleration_Control(float acc);
void Check_L6472_Acceleration_Control(void);

// extern function prototypes


#ifdef __cplusplus
}
#endif

#endif /* __L6474_ACCELERATION_CONTROL_H */
