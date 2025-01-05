#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;
static Chrono_TypeDef cycletimer= {0,0,0.0};
static volatile bool ISRFlag = FALSE;
static volatile uint8_t ISR_cmd = 0;

extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); // standard stepclockhandler
extern void L6474_StepClockHandler_alt(uint8_t deviceId, float acceleration); // alternative stepclockhandler for acceleration control


void Init_L6472_Acceleration_Control(L6474_Acceleration_Control_Init_TypeDef *gInitParams) {

	  // Initialize PWM period variables used by step interrupt
	  hAccelCtrl.desired_pwm_period = UINT32_MAX;
	  hAccelCtrl.current_pwm_period = UINT32_MAX;
	  hAccelCtrl.target_velocity = 0.0;
	  hAccelCtrl.speed = gInitParams->min_speed;

	  hAccelCtrl.t_sample = gInitParams->t_sample;
	  hAccelCtrl.min_speed = gInitParams->min_speed;
	  hAccelCtrl.max_speed = gInitParams->max_speed;
	  hAccelCtrl.max_accel = gInitParams->max_accel;

	  hAccelCtrl.old_dir = UNKNOW_DIR;
	  hAccelCtrl.new_dir = UNKNOW_DIR;
	  hAccelCtrl.state = 0;
	  hAccelCtrl.update_isr_flag = FALSE;

	  cycletimer.t_diff_s = hAccelCtrl.t_sample;

}

void SetAccel_L6472_Acceleration_Control(float acc) {
	// clamp acceleration
	if (acc > (float) hAccelCtrl.max_accel) {
		hAccelCtrl.acceleration = (float) hAccelCtrl.max_accel;
	} else if (acc < -1.0*(float) hAccelCtrl.max_accel) {
		hAccelCtrl.acceleration = -1.0* (float) hAccelCtrl.max_accel;
	} else {
		hAccelCtrl.acceleration = acc;
	}
}

uint8_t Run_L6472_Acceleration_Control(float acc) {

	switch (hAccelCtrl.state) {
	case 0: // first call
		hAccelCtrl.state = 1;
		//BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.min_speed+1);
		//L6474_Run_InstantSteady(0,FORWARD);
		L6474_SetMaxSpeed(0,hAccelCtrl.min_speed+1);
		//BSP_MotorControl_Run(0,FORWARD);
		L6474_Run(0,FORWARD);
		//ISRFlag = FALSE;
		break;
	case 1:
		if (BSP_MotorControl_GetCurrentSpeed(0) >= hAccelCtrl.min_speed) {
			hAccelCtrl.state = 2;
			ISR_cmd = 1;
			//__disable_irq();
			L6474_SetMaxSpeed(0,hAccelCtrl.max_speed);
			//__enable_irq();
			ISRFlag = FALSE;
		}
		break;
	case 2:
		if (ISRFlag){
			hAccelCtrl.state = 9;
			//L6474_SpoofMaxSpeed(0);
			ISRFlag = FALSE;
			ISR_cmd = 2;
			//hAccelCtrl.update_isr_flag = TRUE;
		}
		break;
	case 9:
		Chrono_Mark(&cycletimer);
		hAccelCtrl.state = 10;
		// no break
	case 10:
		// normal operation

		/*

		//get sample time
		hAccelCtrl.t_sample = Chrono_GetDiffMark(&cycletimer);

		// integrate
		hAccelCtrl.target_velocity += hAccelCtrl.acceleration * hAccelCtrl.t_sample;

		// clamp velocity
		if (hAccelCtrl.target_velocity > (float) hAccelCtrl.max_speed) {
			hAccelCtrl.target_velocity = (float) hAccelCtrl.max_speed;
		} else if (hAccelCtrl.target_velocity < -1.0*(float) hAccelCtrl.max_speed) {
			hAccelCtrl.target_velocity = -1.0*(float) hAccelCtrl.max_speed;
		}

		// update old and new direction
		hAccelCtrl.old_dir = hAccelCtrl.new_dir;
		hAccelCtrl.new_dir = hAccelCtrl.target_velocity > 0 ? FORWARD : BACKWARD;

		if (hAccelCtrl.new_dir == FORWARD) {
			hAccelCtrl.speed = hAccelCtrl.target_velocity;
		} else {
			hAccelCtrl.speed = hAccelCtrl.target_velocity * -1.0;
			if (hAccelCtrl.speed == 0) hAccelCtrl.speed = 0; // convert negative 0 to positive 0
		}

		if (hAccelCtrl.speed < hAccelCtrl.min_speed) {
			hAccelCtrl.speed = hAccelCtrl.min_speed;
		}

		hAccelCtrl.speed_prescaled = __L6474_Board_Pwm1PrescaleFreq(hAccelCtrl.speed);

		hAccelCtrl.desired_pwm_period_float = roundf(RCC_SYS_CLOCK_FREQ / hAccelCtrl.speed_prescaled);

		if (!(hAccelCtrl.desired_pwm_period_float < 4294967296.0f)) {
			hAccelCtrl.desired_pwm_period = UINT32_MAX;
		} else {
			hAccelCtrl.desired_pwm_period = (uint32_t)(hAccelCtrl.desired_pwm_period_float);
		}
		hAccelCtrl.velocity = (float) RCC_SYS_CLOCK_FREQ / (float) hAccelCtrl.current_pwm_period / (float) TIMER_PRESCALER / (float) BSP_MOTOR_CONTROL_BOARD_PWM1_FREQ_RESCALER;
		*/
		break;
	default:
		break;
	}

	return hAccelCtrl.state;
}

void StepClockHandler_L6472_Acceleration_Control(void) {

	ISRFlag = TRUE;

	switch (ISR_cmd){
	case 0: // default stepclockhandler that does motion planning or wants to go to max speed
		BSP_MotorControl_StepClockHandler(0);
		break;
	case 1: // acceleration mode
		L6474_StepClockHandler_alt(0, hAccelCtrl.acceleration);
		break;
		/*
	case 2:
		//if (hAccelCtrl.new_dir != hAccelCtrl.old_dir){
			if (hAccelCtrl.new_dir == FORWARD){
				L6474_Board_SetDirectionGpio(0, FORWARD);
			} else {
				L6474_Board_SetDirectionGpio(0, BACKWARD);
			}
		//}
		//L6474_Board_Pwm1SetPeriod(hAccelCtrl.desired_pwm_period);
		//hAccelCtrl.current_pwm_period = hAccelCtrl.desired_pwm_period;

		*/
	default:
		break;
	}

}

float GetSampleTime_L6472_Acceleration_Control(void) {
	return hAccelCtrl.t_sample;
}
