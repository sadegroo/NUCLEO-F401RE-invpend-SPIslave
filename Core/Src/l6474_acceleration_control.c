#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;
static Chrono_TypeDef cycletimer= {0,0,0.0};

void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams) {

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

	  cycletimer.t_diff_s = hAccelCtrl.t_sample;

}

uint8_t Run_L6472_Acceleration_Control(float acc) {

	motorDir_t new_dir;
	bool status;

	switch (hAccelCtrl.state) {
	case 0: // first call
		//L6474_CmdEnable(0);
		//L6474_ApplySpeed(0, hAccelCtrl.min_speed);
		//L6474_Board_Pwm1SetFreq(hAccelCtrl.min_speed);
		//L6474_Board_Pwm1SetPeriod(__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed));
		//status=BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.min_speed);
		BSP_MotorControl_Run(0,FORWARD);
		hAccelCtrl.state = 1;

		break;
	case 1:
		//wait for steady running at min speed
		if (BSP_MotorControl_GetDeviceState(0) == ACCELERATING) {
//			status=BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.max_speed);
			L6474_Board_Pwm1SetPeriod(__L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.min_speed));
			hAccelCtrl.state = 9;
		}
		break;
	case 9:
		Chrono_Mark(&cycletimer);
		hAccelCtrl.state++;

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

		//get sample time
		Chrono_Mark(&cycletimer);
		hAccelCtrl.t_sample = cycletimer.t_diff_s;

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

float GetSampleTime_L6472_Acceleration_Control(void) {
	return hAccelCtrl.t_sample;
}
