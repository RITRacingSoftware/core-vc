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

static int pwm_msg_divider = 0;
static uint8_t inverter_pwm = 0;
static uint8_t motor_pwm = 0;


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
			if (dtiARR[i]->actual2.temp_motor > max_mot) max_mot = dtiArr[i]->actual2.temp_motor;
			if (dtiArr[i]->actual2.temp_inverter > max_inv) max_inv = dtiArr[i]->actual2.temp_inverter;
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
		if(dtiArr[i]->info.input_voltage < (main.bms_status_pack_voltage * 0.9f) || dtiArr[i]->info.input_voltage < MIN_PRECHARGE_VOL) return false;
	}

}

DTIState_e DTI_get_state(uint8_t dtiNum){
	return dtiArr[dtiNum]->state];
}



