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


void DTI_Task_Update(){
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
			if (dtiARR[i]->tempFault.tempmotor > max_mot) max_mot = dtiArr[i]->tempFault.tempmotor;
			if (dtiArr[i]->tempFault.tempcontoller > max_inv) max_inv = dtiArr[i]->tempFault.tempcontroller;
		}
		if ((inverter_pwm > 0) && (max_inv < 250)) inverter_pwm = 0;
		else if ((inverter_pwm == 0) && (max_inv >= 300)) inverter_pwm = 25;
		if ((motor_pwm > 0) && (max_mot < 750)) motor_pwm = 0;
		else if ((motor_pwm == 0) && (max_mot > 800)) motor_pwm = 25;
		uint64_t msg = inverter_pwm | (motor_pwm << 5);
		core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_PDU_CONTROL_FRAME_ID): 2, msg);
		pwm_msg_divider = 0;
    }
}

bool DTI_get_precharged_all(){
	for(int i=0; i<4; i++){
		if(dtiArr[i]->erpm_duty_voltage.input_voltage < (main.bms_status_pack_voltage * 0.9f) || dtiArr[i]->erpm_duty_voltage.input_voltage < MIN_PRECHARGE_VOL) return false;
	}

}

DTIState_e DTI_get_state(uint8_t dtiNum){
	return dtiArr[dtiNum]->state];
}

bool DTI_get_drive_enable_all(void){
	for(int i = 0; i<4; i++){
		if(dtiArr[i]->misc.drive_enable != 1) return false;
	}
}

void DTI_set_drive_enable_all(uint8_t val){
	for(int i=0; i<4; i++){
		dtiArr[i]->enable.drive_enable = val;
	}
}



void DTI_CAN_rx(){
	Can_Message_s canMessage;

	if(core_CAN_receive_from_queue(CAN_INV, &canMessage)){
		int id = canMessage.id;

		switch(id){
			//RR
			case(DTI_DBC_RR_ERPM_DUTY_VOLTAGE_FRAME_ID):
				inverter_dbc_rr_erpm_duty_voltage_full_decode(&dtiRR.erpm_duty_voltage, (uint8_t *) &canMessage.data, 8); break;
			case(DTI_DBC_RR_AC_DC_CURRENT_FRAME_ID):
				inverter_dbc_rr_ac_dc_current_full_decode(&dtiRR.current), (uint_8 *) &canMessage.data, 8); break;

			case(DTI_DBC_RR_TEMP_FAULT_FRAME_ID):
				inverter_dbc_rr_temp_fault_full_decode(&dtiRR.tempFault), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_RR_FOC_FRAME_ID):
				inverter_dbc_rr_foc_full_decode(&dtiRR.foc), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_RR_MISC_FRAME_ID):
				inverter_dbc_rr_misc_full_decode(&dtiRR.misc), (uint_8 *) &canMessage.data, 8); break;

			
			//RL
			case(DTI_DBC_RL_ERPM_DUTY_VOLTAGE_FRAME_ID):
				inverter_dbc_rl_erpm_duty_voltage_full_decode(&dtiRL.erpm_duty_voltage, (uint8_t *) &canMessage.data, 8); break;
			case(DTI_DBC_RL_AC_DC_CURRENT_FRAME_ID):
				inverter_dbc_rl_ac_dc_current_full_decode(&dtiRL.current), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_RL_TEMP_FAULT_FRAME_ID):
				inverter_dbc_rl_temp_fault_full_decode(&dtiRL.tempFault), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_RL_FOC_FRAME_ID):
				inverter_dbc_rl_foc_full_decode(&dtiRL.foc), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_RL_MISC_FRAME_ID):
				inverter_dbc_fr_misc_full_decode(&dtiFR.misc), (uint_8 *) &canMessage.data, 8); break;

			//FR
			case(DTI_DBC_FR_ERPM_DUTY_VOLTAGE_FRAME_ID):
				inverter_dbc_fr_erpm_duty_voltage(&dtiFR.erpm_duty_voltage, (uint8_t *) &canMesssage.data, 8); break;
			case(DTI_DBC_FR_AC_DC_CURRENT_FRAME_ID):
				inverter_dbc_fr_ac_dc_current_full_decode(&dtiFR.current), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FR_TEMP_FAULT_FRAME_ID):
				inverter_dbc_fr_temp_fault_full_decode(&dtiFR.tempFault), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FR_FOC_FRAME_ID):
				inverter_dbc_fr_foc_full_decode(&dtiFR.foc), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FR_MISC_FRAME_ID):
				inverter_dbc_fr_misc_full_decode(&dtiFR.misc), (uint_8 *) &canMessage.data, 8); break;
			
			//FL
			case(DTI_DBC_FL_ERPM_DUTY_VOLTAGE_FRAME_ID):
				inverter_dbc_fl_erpm_duty_voltage_full_decode(&dtiFL.erpm_duty_voltage, (uint8_t *) &canMessage.data, 8); break;
			case(DTI_DBC_FL_AC_DC_CURRENT_FRAME_ID):
				inverter_dbc_fl_ac_dc_current_full_decode(&dtiFL.current), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FL_TEMP_FAULT_FRAME_ID):
				inverter_dbc_fl_temp_fault_full_decode(&dtiFL.tempFault), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FL_FOC_FRAME_ID):
				inverter_dbc_fl_foc_full_decode(&dtiFL.foc), (uint_8 *) &canMessage.data, 8); break;
			case(DTI_DBC_FL_MISC_FRAME_ID):
				inverter_dbc_fl_misc_full_decode(&dtiFL.misc), (uint_8 *) &canMessage.data, 8); break;
		}
		
	}

}
