#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;

void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams) {
	  // Initialize and enable cycle counter
	  ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	  DWT->CTRL |= 1; 		// at address 0xE0001000
	  DWT->CYCCNT = 0; 		// at address 0xE0001004

	  // Initialize PWM period variables used by step interrupt
	  hAccelCtrl.desired_pwm_period = UINT32_MAX;
	  hAccelCtrl.current_pwm_period = UINT32_MAX;
	  hAccelCtrl.target_velocity_prescaled = 0.0;

	  hAccelCtrl.t_sample = gInitParams->t_sample;
	  hAccelCtrl.max_speed = gInitParams->max_speed;
	  hAccelCtrl.max_accel = gInitParams->max_accel;
	  hAccelCtrl.update_flag = 0;
	  hAccelCtrl.zero_received_flag = 0;


}


void Integrate_L6472_Acceleration_Control(float acc) {
	/*
	 *  Stepper motor acceleration, speed, direction and position control developed by Ryan Nemiroff
	 */
	if (acc == 0 && hAccelCtrl.zero_received_flag == 0) {

		hAccelCtrl.zero_received_flag = 1;
	    //BSP_MotorControl_CmdEnable(0);
	    BSP_MotorControl_Run(0,FORWARD);
	}

	if ( hAccelCtrl.zero_received_flag == 1) {
		hAccelCtrl.update_flag = 1;
		hAccelCtrl.apply_acc_start_time = DWT->CYCCNT;

		hAccelCtrl.old_dir = hAccelCtrl.target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

		// clamp acceleration
		if (hAccelCtrl.old_dir == FORWARD) {
			if (acc > (float) hAccelCtrl.max_accel) {
				hAccelCtrl.acceleration = (float) hAccelCtrl.max_accel;
			} else if (acc < -1.0*(float) hAccelCtrl.max_accel) {
				hAccelCtrl.acceleration = -1.0* (float) hAccelCtrl.max_accel;
			}
		} else {
			if (acc < -1.0* (float) hAccelCtrl.max_accel) {
				hAccelCtrl.acceleration = -1.0*(float) hAccelCtrl.max_accel;
			} else if (acc > (float) hAccelCtrl.max_accel) {
				hAccelCtrl.acceleration = (float) hAccelCtrl.max_accel;
			}
		}
		// integrate
		hAccelCtrl.target_velocity_prescaled += __L6474_Board_Pwm1PrescaleFreq(acc) * hAccelCtrl.t_sample;
		hAccelCtrl.new_dir = hAccelCtrl.target_velocity_prescaled > 0 ? FORWARD : BACKWARD;

		// clamp velocity
		if (hAccelCtrl.target_velocity_prescaled > (float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
			hAccelCtrl.target_velocity_prescaled = (float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
		} else if (hAccelCtrl.target_velocity_prescaled < -1.0*(float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
			hAccelCtrl.target_velocity_prescaled = -1.0*(float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
		}


		if (hAccelCtrl.new_dir == FORWARD) {
			hAccelCtrl.speed_prescaled = hAccelCtrl.target_velocity_prescaled;
		} else {
			hAccelCtrl.speed_prescaled = hAccelCtrl.target_velocity_prescaled * -1.0;
			if (hAccelCtrl.speed_prescaled == 0) hAccelCtrl.speed_prescaled = 0; // convert negative 0 to positive 0
		}

		hAccelCtrl.desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / hAccelCtrl.speed_prescaled);

		if (!(hAccelCtrl.desired_pwm_period_float < 4294967296.0f)) {
			hAccelCtrl.desired_pwm_period = UINT32_MAX;
		} else {
			hAccelCtrl.desired_pwm_period = (uint32_t)(hAccelCtrl.desired_pwm_period_float);
		}
	}

}


void Update_L6472_Acceleration_Control(void) {

	if (hAccelCtrl.update_flag == 1) {
		if (hAccelCtrl.target_velocity_prescaled < 0){
			L6474_Board_SetDirectionGpio(0, BACKWARD);
		} else {
			L6474_Board_SetDirectionGpio(0, FORWARD);
		}

		L6474_Board_Pwm1SetPeriod(hAccelCtrl.desired_pwm_period);
		hAccelCtrl.current_pwm_period = hAccelCtrl.desired_pwm_period;
		hAccelCtrl.update_flag =0;

		hAccelCtrl.velocity = (float) RCC_SYS_CLOCK_FREQ / (float) hAccelCtrl.current_pwm_period / (float) TIMER_PRESCALER / (float) BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER;
	}

	/*

					if (fabs(recv_number1) > FLT_EPSILON) {
						// update setpoint
						prev_velocity_setpoint = velocity_setpoint;
						velocity_setpoint += accel_multiplier *  recv_number1 * T_SAMPLE;
						// clamp setpoint
						if (velocity_setpoint > MAX_SPEED) {
							velocity_setpoint = MAX_SPEED;
						} else if (velocity_setpoint < -1* MAX_SPEED) {
							velocity_setpoint = -1 * MAX_SPEED;
						}

						BSP_MotorControl_SetAcceleration(0,(uint16_t) fabs(accel_multiplier *  recv_number1));
						BSP_MotorControl_SetDeceleration(0,(uint16_t) fabs(accel_multiplier *  recv_number1));

						BSP_MotorControl_SetMaxSpeed(0,(uint16_t) fmax(fabs(velocity_setpoint), MIN_SPEED));

						if (__HAS_OPPOSITE_SIGNS(prev_velocity_setpoint, velocity_setpoint)) {
							if (velocity_setpoint < 0){
								L6474_Board_SetDirectionGpio(0, BACKWARD);
								//BSP_MotorControl_SoftStop(0);
								//BSP_MotorControl_Run(0,BACKWARD);
							} else {
								L6474_Board_SetDirectionGpio(0, FORWARD);
								//BSP_MotorControl_Run(0,FORWARD);
							}

						}

						if (fabs(velocity_setpoint) > MIN_SPEED) {
							L6474_Board_Pwm1SetPeriod((uint32_t) roundf((float)RCC_SYS_CLOCK_FREQ / (float)__L6474_Board_Pwm1PrescaleFreq(fabs(velocity_setpoint))));
						}

						if (BSP_MotorControl_GetDeviceState(0) >= 8){
							// handle inactive drive and direction change
							if (velocity_setpoint < 0){
								BSP_MotorControl_Run(0,BACKWARD);
							} else if (velocity_setpoint > 0){
								BSP_MotorControl_Run(0,FORWARD);
							}
						}


					}
					*/
}
