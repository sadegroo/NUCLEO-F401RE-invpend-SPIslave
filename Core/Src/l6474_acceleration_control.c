#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;

void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams) {

		/* This causes timing issues, or maybe not, retry!!!
	  // Initialize and enable cycle counter
	  ITM->LAR = 0xC5ACCE55; 	// at address 0xE0001FB0
	  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // at address 0xE000EDFC, CoreDebug_DEMCR_TRCENA_Msk = 0x01000000
	  DWT->CTRL |= 1; 		// at address 0xE0001000
	  DWT->CYCCNT = 0; 		// at address 0xE0001004
	  */

	  // Initialize PWM period variables used by step interrupt
	  hAccelCtrl.desired_pwm_period = UINT32_MAX;
	  hAccelCtrl.current_pwm_period = UINT32_MAX;
	  hAccelCtrl.target_velocity_prescaled = 0.0;

	  hAccelCtrl.t_sample = gInitParams->t_sample;
	  hAccelCtrl.min_speed = gInitParams->min_speed;
	  hAccelCtrl.max_speed = gInitParams->max_speed;
	  hAccelCtrl.max_accel = gInitParams->max_accel;
	  hAccelCtrl.update_flag = 0;
	  hAccelCtrl.state = 0;


}

uint8_t Run_L6472_Acceleration_Control(float acc) {

	motorDir_t new_dir;
	bool status;

	switch (hAccelCtrl.state) {
	case 0:
		if (acc == 0) {
			// after 0 command received
			//L6474_CmdEnable(0);
			//L6474_ApplySpeed(0, hAccelCtrl.min_speed);
			//L6474_Board_Pwm1SetFreq(hAccelCtrl.min_speed);
			//L6474_Board_Pwm1SetPeriod(__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed));
			status=BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.min_speed);
		    BSP_MotorControl_Run(0,FORWARD);
		    hAccelCtrl.state = 1;
		}
		break;
	case 1:
		//wait for steady running at min speed
		if (BSP_MotorControl_GetDeviceState(0) == STEADY) {
			status=BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.max_speed);
			L6474_Board_Pwm1SetPeriod(__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed));
			hAccelCtrl.state = 10;
			//hAccelCtrl.integrate_start_time = DWT->CYCCNT; // timemark for integration
		}
		break;

	case 10:
		// normal operation
		hAccelCtrl.update_flag = 1;


		// clamp acceleration
		if (acc > (float) hAccelCtrl.max_accel) {
			hAccelCtrl.acceleration = (float) hAccelCtrl.max_accel;
		} else if (acc < -1.0*(float) hAccelCtrl.max_accel) {
			hAccelCtrl.acceleration = -1.0* (float) hAccelCtrl.max_accel;
		} else {
			hAccelCtrl.acceleration = acc;
		}

		/* Timing issues !!!
		//measure time difference
		uint32_t integrate_end_time = DWT->CYCCNT; // Read the current CYCCNT value
		uint32_t difference = integrate_end_time - hAccelCtrl.integrate_start_time;

		// Update the previous value for the next measurement
		hAccelCtrl.integrate_start_time = integrate_end_time;

		// Convert difference to time (in seconds)
		float integrate_time_diff = (float)difference / RCC_SYS_CLOCK_FREQ;
		*/

		// integrate
		hAccelCtrl.target_velocity_prescaled += __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.acceleration) * hAccelCtrl.t_sample;

		// clamp velocity
		if (hAccelCtrl.target_velocity_prescaled > (float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
			hAccelCtrl.target_velocity_prescaled = (float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
		} else if (hAccelCtrl.target_velocity_prescaled < -1.0*(float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed)) {
			hAccelCtrl.target_velocity_prescaled = -1.0*(float) __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.max_speed);
		}

		new_dir = hAccelCtrl.target_velocity_prescaled > 0 ? FORWARD : BACKWARD;
		if (new_dir == FORWARD) {
			hAccelCtrl.speed_prescaled = hAccelCtrl.target_velocity_prescaled;
		} else {
			hAccelCtrl.speed_prescaled = hAccelCtrl.target_velocity_prescaled * -1.0;
			if (hAccelCtrl.speed_prescaled == 0) hAccelCtrl.speed_prescaled = 0; // convert negative 0 to positive 0
		}

		if (hAccelCtrl.speed_prescaled < __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed)) {
			hAccelCtrl.speed_prescaled = __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed);
		}

		hAccelCtrl.desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / hAccelCtrl.speed_prescaled);

		if (!(hAccelCtrl.desired_pwm_period_float < 4294967296.0f)) {
			hAccelCtrl.desired_pwm_period = UINT32_MAX;
		} else {
			hAccelCtrl.desired_pwm_period = (uint32_t)(hAccelCtrl.desired_pwm_period_float);
		}
		hAccelCtrl.velocity = (float) RCC_SYS_CLOCK_FREQ / (float) hAccelCtrl.current_pwm_period / (float) TIMER_PRESCALER / (float) BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER;
		break;
	default:
		break;
	}

	return hAccelCtrl.state;
}


void Update_L6472_Acceleration_Control(void) {

	//motorDir_t current_dir = BSP_MotorControl_GetDirection(0);

	if (hAccelCtrl.update_flag == 1) {
		if (hAccelCtrl.target_velocity_prescaled < 0){
			L6474_Board_SetDirectionGpio(0, BACKWARD);
		} else if (hAccelCtrl.target_velocity_prescaled > 0){
			L6474_Board_SetDirectionGpio(0, FORWARD);
		}

		L6474_Board_Pwm1SetPeriod(hAccelCtrl.desired_pwm_period);
		hAccelCtrl.current_pwm_period = hAccelCtrl.desired_pwm_period;
		hAccelCtrl.update_flag =0;
	}
}
