#include "DTI.h"
#include "config.h" 

#include "rtt.h"
#include "inverter_dbc.h"
#include "can.h"
#include "timeout.h"
#include "gpio.h"
#include "driver_GPIO.h"
#include "driver_can.h"
#include "FaultManager.h"
#include "driverless.h"
#include "VehicleState.h"

static DTI_s dtiRR = {0};
static DTI_s dtiRL = {0};
static DTI_s dtiFR = {0};
static DTI_s dtiFL = {0};

static DTI_s *dtiArr[4] = {&dtiRR, &dtiRL, &dtiFR, &dtiFL};

static core_timeout_t rrTimeout;
static core_timeout_t rlTimeout;
static core_timeout_t frTimeout;
static core_timeout_t flimeout;


static int pwm_msg_divider = 0;
static uint8_t inverter_pwm = 0;
static uint8_t motor_pwm = 0;


static void timeout_callback(core_timeout_t *timeout)
{
    if (timeout == &rr_timeout) mainBus.inverter_status.vc_rr_lost = 1;
    else if (timeout == &rl_timeout) mainBus.inverter_status.vc_rl_lost = 1;
    else if (timeout == &fr_timeout) mainBus.inverter_status.vc_fr_lost = 1;
    else if (timeout == &fl_timeout) mainBus.inverter_status.vc_fl_lost = 1;
}


void DTI_init(){
	//RR timeout init
	rrTimeout.callback = timeout_callback; 
	rrTimeout.timeout = INV_CAN_TIMEOUT_MS;
	rrTimeout.module = CAN_INV;
	rrTimeout.ref = DTI_DBC_RR_INFO_1_FRAME_ID;
	core_timeout_insert(&rrTimeout);

	//RL timeout init
	rlTimeout.callback = timeout_callback; 
	rlTimeout.timeout = INV_CAN_TIMEOUT_MS;
	rlTimeout.module = CAN_INV;
	rlTimeout.ref = DTI_DBC_RL_INFO_1_FRAME_ID;
	core_timeout_insert(&rlTimeout);

	//FR timeout init
	frTimeout.callback = timeout_callback; 
	frTimeout.timeout = INV_CAN_TIMEOUT_MS;
	frTimeout.module = CAN_INV;
	frTimeout.ref = DTI_DBC_FR_INFO_1_FRAME_ID;
	core_timeout_insert(&frTimeout);

	//FL timeout init
	flTimeout.callback = timeout_callback; 
	flTimeout.timeout = INV_CAN_TIMEOUT_MS;
	flTimeout.module = CAN_INV;
	flTimeout.ref = DTI_DBC_FL_INFO_1_FRAME_ID;
	core_timeout_insert(&flTimeout);
}


void Inverters_Task_Update(){
	float debug[2];
	VehicleState_e vs = VehicleState_get_state();

	check_errors();
	state_machine();

	if (vs != VehicleState_RTD) {
		for (int inv = 0; inv < 4; inv++) { set_zero(inv); }
	}

    // Check if we're double pedaling
	if (FaultManager_read(FAULT_DOUBLE_PEDAL | FAULT_SOFT_DOUBLE_PEDAL)) {
		for (int inv = 0; inv < 4; inv++) { set_zero(inv); }
	}

	// Check if the motorspeeds are too low for regen
	check_regen();

	// Fan PWM control
	if ((++pwm_msg_divider) == 100) {
		int16_t max_inv = 0, max_mot = 0;
		for (int i=0; i < 4; i++) {
			if (invArr[i]->actual2.temp_motor > max_mot) max_mot = invArr[i]->actual2.temp_motor;
			if (invArr[i]->actual2.temp_inverter > max_inv) max_inv = invArr[i]->actual2.temp_inverter;
		}
		if ((inverter_pwm > 0) && (max_inv < 250)) inverter_pwm = 0;
		else if ((inverter_pwm == 0) && (max_inv >= 300)) inverter_pwm = 25;
		if ((motor_pwm > 0) && (max_mot < 750)) motor_pwm = 0;
		else if ((motor_pwm == 0) && (max_mot > 800)) motor_pwm = 25;
		uint64_t msg = inverter_pwm | (motor_pwm << 5);
		core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_PDU_CONTROL_FRAME_ID, 2, msg);
		pwm_msg_divider = 0;
    }
}

bool DTI_get_precharged_all(){
	for(int i = 0; i<4; i++){
		if(dtiArr[i] -> )
	}

}

DTIState_e DTI_get_state(uint8_t dtiNum){
	return dtiArr[dtiNum->state];
}

bool DTI_get_enable_all(){
	for(int dti = 0; dti < 4; dti++){
		if(dtiArr->) return fasle;
	}
	return true;
}


/*
void DTI_CAN_rx(){


}*/
