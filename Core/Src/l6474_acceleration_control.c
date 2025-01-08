#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;
static Chrono_TypeDef cycletimer= {0,0,0.0};
static volatile bool ISRFlag = FALSE;
static volatile uint8_t ISR_cmd = 0;

extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); // standard stepclockhandler
extern void L6474_StepClockHandler_alt(uint8_t deviceId, float acceleration); // alternative stepclockhandler for acceleration control


void Init_L6472_Acceleration_Control(L6474_Init_t *gInitParams, float t_sample) {

	/// Acceleration rate in step/s2. Range: (0..+inf).
	hAccelCtrl.max_accel = gInitParams->acceleration_step_s2;
	/// DECELERATION is ignored for acceleration control

	/// Maximum speed in step/s. Range: (30..10000].
	hAccelCtrl.max_speed = gInitParams->maximum_speed_step_s;
	 ///Minimum speed in step/s. Range: [30..10000).
	hAccelCtrl.min_speed = gInitParams->minimum_speed_step_s;


	  hAccelCtrl.speed = 0;

	  hAccelCtrl.t_sample = t_sample;
	  cycletimer.t_diff_s = hAccelCtrl.t_sample;

	  hAccelCtrl.state = 0;

	  // old inits
	  //hAccelCtrl.min_speed = gInitParams->min_speed;
	  //hAccelCtrl.max_speed = gInitParams->max_speed;
	  //hAccelCtrl.max_accel = gInitParams->max_accel;

	  //hAccelCtrl.old_dir = UNKNOW_DIR;
	  //hAccelCtrl.new_dir = UNKNOW_DIR;
	  //hAccelCtrl.update_isr_flag = FALSE;
	  // Initialize PWM period variables used by step interrupt
	  //hAccelCtrl.desired_pwm_period = UINT32_MAX;
	  //hAccelCtrl.current_pwm_period = UINT32_MAX;
	  //hAccelCtrl.target_velocity = 0.0;


}

void Stop__L6472_Acceleration_Control(void){
	BSP_MotorControl_HardStop(0);
	ISR_cmd = 0;
	hAccelCtrl.state = 0;

}

void Run_L6472_Acceleration_Control(float acceleration_input) {

	switch (hAccelCtrl.state) {
	case 0: // first call
		hAccelCtrl.state = 1;
		ISR_cmd = 2;
		BSP_MotorControl_Run(0,FORWARD);
		ISRFlag = FALSE;
		break;
	case 1:
		if (ISRFlag) {
			Chrono_Mark(&cycletimer);
			ISR_cmd = 1;
			ISRFlag = FALSE;
			hAccelCtrl.state = 10;
		}
	case 10:
		if (ISRFlag) {
			// normal operation
			hAccelCtrl.t_sample = Chrono_GetDiffMark(&cycletimer);

		  // clamp acceleration
		  if (acceleration_input > (float) hAccelCtrl.max_accel) {
			  hAccelCtrl.acceleration = (float) hAccelCtrl.max_accel;
		  } else if (acceleration_input < -1.0*(float) hAccelCtrl.max_accel) {
			  hAccelCtrl.acceleration = -1.0* (float) hAccelCtrl.max_accel;
		  } else {
			  hAccelCtrl.acceleration = acceleration_input;
		  }
		}

		/* old way below

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

		// functions used before
		//BSP_MotorControl_SetMaxSpeed(0,hAccelCtrl.min_speed+1);
		//L6474_Run_InstantSteady(0,FORWARD);
		//L6474_SetMaxSpeed(0,hAccelCtrl.min_speed+1);
		//BSP_MotorControl_Run(0,FORWARD);
		//L6474_Run(0,FORWARD);
		//ISRFlag = FALSE;
		//L6474_SpoofMaxSpeed(0);

		break;
	default:
		break;
	}
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
	case 2: // run at min speed
		L6474_StepClockHandler_alt(0, hAccelCtrl.acceleration);
		L6474_ApplySpeed(0, hAccelCtrl.min_speed);
		break;
		/* old way below
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
