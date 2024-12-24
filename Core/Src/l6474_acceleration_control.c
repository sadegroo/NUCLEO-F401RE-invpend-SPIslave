#include "l6474_acceleration_control.h"

static L6474_Acceleration_Control_TypeDef hAccelCtrl;

void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams) {
	  // Initialize and enable cycle counter
	  ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	  DWT->CTRL |= 1; 		// at address 0xE0001000
	  DWT->CYCCNT = 0; 		// at address 0xE0001004

	  // Initialize PWM period variables used by step interrupt
	  hAccelCtrl.desired_pwm_period = 0;
	  hAccelCtrl.current_pwm_period = 0;
	  hAccelCtrl.target_velocity_prescaled = 0.0;

	  hAccelCtrl.t_sample = gInitParams->t_sample;
	  hAccelCtrl.max_speed = gInitParams->max_speed;
	  hAccelCtrl.max_accel = gInitParams->max_accel;
	  hAccelCtrl.max_decel = gInitParams->max_decel;
}


void Update_L6472_Acceleration_Control(float acc) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */
	float speed_prescaled;
	float desired_pwm_period_float;
	uint32_t pwm_count ;
	uint32_t pwm_time_left;
	uint32_t new_pwm_time_left;
	uint32_t current_pwm_period_local = hAccelCtrl.current_pwm_period;
	uint32_t desired_pwm_period_local = hAccelCtrl.desired_pwm_period;
	uint32_t effective_pwm_period = desired_pwm_period_local;

	/*
	 * Add time reporting
	 */


	hAccelCtrl.apply_acc_start_time = DWT->CYCCNT;
	hAccelCtrl.acceleration = acc; // new acceleration

	motorDir_t old_dir = hAccelCtrl.target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (old_dir == FORWARD) {
		if (hAccelCtrl.acceleration > hAccelCtrl.max_accel) {
			hAccelCtrl.acceleration = hAccelCtrl.max_accel;
		} else if (hAccelCtrl.acceleration < -hAccelCtrl.max_decel) {
			hAccelCtrl.acceleration = -hAccelCtrl.max_decel;
		}
	} else {
		if (hAccelCtrl.acceleration < -hAccelCtrl.max_accel) {
			hAccelCtrl.acceleration = -hAccelCtrl.max_accel;
		} else if (hAccelCtrl.acceleration > hAccelCtrl.max_decel) {
			hAccelCtrl.acceleration = hAccelCtrl.max_decel;
		}
	}

	hAccelCtrl.target_velocity_prescaled += __L6474_Board_Pwm1PrescaleFreq(acc) * hAccelCtrl.t_sample;
	motorDir_t new_dir = hAccelCtrl.target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

	if (hAccelCtrl.target_velocity_prescaled > __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
		hAccelCtrl.target_velocity_prescaled = __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
	} else if (hAccelCtrl.target_velocity_prescaled < -1*__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
		hAccelCtrl.target_velocity_prescaled = (float) -1*__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
	}


	if (new_dir == FORWARD) {
		speed_prescaled = hAccelCtrl.target_velocity_prescaled;
	} else {
		speed_prescaled = hAccelCtrl.target_velocity_prescaled * -1.0;
		if (speed_prescaled == 0) speed_prescaled = 0; // convert negative 0 to positive 0
	}

	desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / speed_prescaled);
		if (!(desired_pwm_period_float < 4294967296.0f)) {
			desired_pwm_period_local = UINT32_MAX;
		} else {
			desired_pwm_period_local = (uint32_t)(desired_pwm_period_float);
		}

		if (old_dir != new_dir) {
			L6474_Board_SetDirectionGpio(0, new_dir);
		}

		if (current_pwm_period_local != 0) {
			pwm_count = L6474_Board_Pwm1GetCounter();
			pwm_time_left = current_pwm_period_local - pwm_count;
			if (pwm_time_left > PWM_COUNT_SAFETY_MARGIN) {
				if (old_dir != new_dir) {
					// pwm_time_left = effective_pwm_period - pwm_time_left; // One method for assignment of PWM period during switching directions. This has the effect of additional discrete step noise.
					pwm_time_left = effective_pwm_period; // Second method for assignment of PWM period during switching directions. This shows reduced discrete step noise.
				}

				new_pwm_time_left = ((uint64_t) pwm_time_left * desired_pwm_period_local) / effective_pwm_period;
				if (new_pwm_time_left != pwm_time_left) {
					if (new_pwm_time_left < PWM_COUNT_SAFETY_MARGIN) {
						new_pwm_time_left = PWM_COUNT_SAFETY_MARGIN;
					}
					current_pwm_period_local = pwm_count + new_pwm_time_left;
					if (current_pwm_period_local < pwm_count) {
						current_pwm_period_local = UINT32_MAX;
					}

					L6474_Board_Pwm1SetPeriod(current_pwm_period_local);
					hAccelCtrl.current_pwm_period = current_pwm_period_local;
				}
			}
		} else {
			L6474_Board_Pwm1SetPeriod(desired_pwm_period_local);
			hAccelCtrl.current_pwm_period = desired_pwm_period_local;
		}

		hAccelCtrl.desired_pwm_period = desired_pwm_period_local;

}
