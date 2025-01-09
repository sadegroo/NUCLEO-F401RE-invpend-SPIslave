#include "l6474_acceleration_control.h"

static volatile L6474_Acceleration_Control_TypeDef hAccelCtrl;
static Chrono_TypeDef cycletimer= {0,0,0.0};
static volatile bool ISRFlag = FALSE;
static volatile uint8_t ISR_cmd = 0;

extern void BSP_MotorControl_StepClockHandler(uint8_t deviceId); // standard stepclockhandler
extern void L6474_StepClockHandler_alt(uint8_t deviceId, int32_t acceleration); // alternative stepclockhandler for acceleration control


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

	  hAccelCtrl.acceleration = 0;
	  hAccelCtrl.velocity = 0;
	  hAccelCtrl.position = 0;
}

void Stop_L6472_Acceleration_Control(void){
	BSP_MotorControl_HardStop(0);
	ISR_cmd = 0;
	hAccelCtrl.state = 0;
}

void Run_L6472_Acceleration_Control(int32_t acceleration_input) {

	switch (hAccelCtrl.state) {
	case 0: // first call
		hAccelCtrl.state = 10;
		ISR_cmd = 1;
		BSP_MotorControl_Run(0,FORWARD);
		ISRFlag = FALSE;
		Chrono_Mark(&cycletimer);
		break;
	case 10: // normal operation
		if (ISRFlag) {
			hAccelCtrl.t_sample = Chrono_GetDiffMark(&cycletimer);

		  // clamp acceleration
		  if (acceleration_input > (int32_t) hAccelCtrl.max_accel) {
			  hAccelCtrl.acceleration = (int32_t) hAccelCtrl.max_accel;
		  } else if (acceleration_input < -1.0*(int32_t) hAccelCtrl.max_accel) {
			  hAccelCtrl.acceleration = -1.0* (int32_t) hAccelCtrl.max_accel;
		  } else {
			  hAccelCtrl.acceleration = (int32_t) acceleration_input;
		  }
		}

		hAccelCtrl.speed = BSP_MotorControl_GetCurrentSpeed(0);
		if (BSP_MotorControl_GetDirection(0) == FORWARD) {
			hAccelCtrl.velocity = (int32_t) hAccelCtrl.speed;
		} else {
			hAccelCtrl.velocity = (int32_t) -1 * hAccelCtrl.speed;
		}

		break;
	default:
		break;
	}
}

void StepClockHandler_L6472_Acceleration_Control(void) {

	ISRFlag = TRUE;
	uint32_t period = 84000000 / ( 1024 * (uint32_t)BSP_MotorControl_GetCurrentSpeed(0));

	switch (ISR_cmd){
	case 0: // default stepclockhandler that does motion planning or wants to go to max speed
		BSP_MotorControl_StepClockHandler(0);
		break;
	case 1: // acceleration mode
		L6474_StepClockHandler_alt(0, hAccelCtrl.acceleration);
		L6474_Board_Pwm1SetPeriod(period);
		break;
	default:
		break;
	}
}

float GetSampleTime_L6472_Acceleration_Control(void) {
	return hAccelCtrl.t_sample;
}

int32_t GetPosition_L6472_Acceleration_Control(void){
 // warning, involves SPI, do not call every cycle
	return BSP_MotorControl_GetPosition(0);
}

int32_t GetVelocity_L6472_Acceleration_Control(void){
	return hAccelCtrl.velocity;
}

int32_t GetAcceleration_L6472_Acceleration_Control(void){
	return hAccelCtrl.acceleration;
}
