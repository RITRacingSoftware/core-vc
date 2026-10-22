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


static void check_errors();
static void state_machine();
static void check_regen();
static bool check_state_change(DTI_s *dti, DTIState_e state);

void DTI_Task_Update(){

	float debug[2];
	VehicleState_e vs = VehicleState_get_state();

	check_errors();
	state_machine();

	if (vs != VehicleState_RTD) {

		for (int inv = 0; inv < 4; inv++) { set_zero(inv); }
		DTI_set_drive_enable_all(DRIVE_ENABLE_OFF);
	}

    // Check if we're double pedaling
	if (FaultManager_read(FAULT_DOUBLE_PEDAL | FAULT_SOFT_DOUBLE_PEDAL)) {
	// Check if the motorspeeds are too low for regen
	check_regen();

	// Fan PWM control
	if ((++pwm_msg_divider) == 100) {
		int16_t max_inv = 0, max_mot = 0;
		for (int i=0; i < 4; i++) {
			if (dtiArr[i]->tempFault.actual_temp_motor > max_mot) max_mot = dtiArr[i]->tempFault.actual_temp_motor;
			if (dtiArr[i]->tempFault.actual_temp_controller > max_inv) max_inv = dtiArr[i]->tempFault.actual_temp_controller;
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
	for(int dti = 0; dti < 4; dti++){
		if(dtiArr[dti]->erpm_duty_voltage.input_voltage < (mainBus.bms_status.bms_status_pack_voltage * 0.9f) || dtiArr[dti]->erpm_duty_voltage.input_voltage < MIN_PRECHARGE_VOL) return false;
	}

}

DTIState_e DTI_get_state(uint8_t dtiNum){
	return dtiArr[dtiNum]->state];
}

void DTI_set_state(uint8_t dtiNum, DTIState_e state){

	if(check_state_change(dtiArr[dtiNum], state)){
		DTI_s *target = dtiArr[dtiNum];
		DTI_s *paired;


		switch(dtiNum){
			case DTI_RR:
				paired = &dtiRR; break;
			case DTI_RL:
				paired = &dtiRL; break;
			case DTI_FR:
				paired = &dtiFR; break;
			case DTI_FL:
				paired = &dtiFL; break;
		}

		target->state;

		switch(state){
			case DTIState_NORMAL:
				if(paired->state == DTIState_SOFT_PAIRED) target->state = DTIState_NORMAL;
				break;
			case DTIState_HARD_FAULT:
				for(int dti = 0; dti < 4; dti++)if(check_state_change(dtiArr[dti], DTIState_HARD_PAIRED)) dtiArr[dti]->state = DTIState_HARD_PAIRED;
				target->state = DTIState_HARD_PAIRED;
				break;
		}
	}
}



bool DTI_get_drive_enable_all(void){
	for(int i=0; i<4; i++){
		if(dtiArr[i]->misc.drive_enable != 1) return false;
	}
}

void DTI_set_drive_enable_all(uint8_t val){
	for(int i=0; i<4; i++){
		dtiArr[i]->enable.drive_enable = val;
	}
}

void DTI_CAN_rx(){
	CanMessage_s canMessage;

	if(core_CAN_receive_from_queue(CAN_INV, &canMessage)){
		int id = canMessage.id;

		switch(id){
			//RR
			case INVERTER_DBC_RR_ERPM_DUTY_VOLTAGE_FRAME_ID:
				inverter_dbc_rr_erpm_duty_voltage_full_decode(&dtiRR.erpm_duty_voltage, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RR_AC_DC_CURRENT_FRAME_ID:
				inverter_dbc_rr_ac_dc_current_full_decode(&dtiRR.current, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RR_TEMP_FAULT_FRAME_ID:
				inverter_dbc_rr_temp_fault_full_decode(&dtiRR.tempFault, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RR_FOC_FRAME_ID:
				inverter_dbc_rr_foc_full_decode(&dtiRR.foc, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RR_MISC_FRAME_ID:
				inverter_dbc_rr_misc_full_decode(&dtiRR.misc, (uint8_t *) canMessage.data, 8); break;
			
			//RL
			case INVERTER_DBC_RL_ERPM_DUTY_VOLTAGE_FRAME_ID:
				inverter_dbc_rl_erpm_duty_voltage_full_decode(&dtiRL.erpm_duty_voltage, (uint8_t) canMessage.data, 8); break;
			case INVERTER_DBC_RL_AC_DC_CURRENT_FRAME_ID:
				inverter_dbc_rl_ac_dc_current_full_decode(&dtiRL.current, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RL_TEMP_FAULT_FRAME_ID:
				inverter_dbc_rl_temp_fault_full_decode(&dtiRL.tempFault, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RL_FOC_FRAME_ID:
				inverter_dbc_rl_foc_full_decode(&dtiRL.foc, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_RL_MISC_FRAME_ID:
				inverter_dbc_fr_misc_full_decode(&dtiFR.misc, (uint8_t *) canMessage.data, 8); break;

			//FR
			case INVERTER_DBC_FR_ERPM_DUTY_VOLTAGE_FRAME_ID:
				inverter_dbc_fr_erpm_duty_voltage_full_decode(&dtiFR.erpm_duty_voltage, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FR_AC_DC_CURRENT_FRAME_ID:
				inverter_dbc_fr_ac_dc_current_full_decode(&dtiFR.current, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FR_TEMP_FAULT_FRAME_ID:
				inverter_dbc_fr_temp_fault_full_decode(&dtiFR.tempFault, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FR_FOC_FRAME_ID:
				inverter_dbc_fr_foc_full_decode(&dtiFR.foc, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FR_MISC_FRAME_ID:
				inverter_dbc_fr_misc_full_decode(&dtiFR.misc, (uint8_t *) canMessage.data, 8); break;
			
			//FL
			case INVERTER_DBC_FL_ERPM_DUTY_VOLTAGE_FRAME_ID:
				inverter_dbc_fl_erpm_duty_voltage_full_decode(&dtiFL.erpm_duty_voltage, (uint8_t) canMessage.data, 8); break;
			case INVERTER_DBC_FL_AC_DC_CURRENT_FRAME_ID:
				inverter_dbc_fl_ac_dc_current_full_decode(&dtiFL.current, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FL_TEMP_FAULT_FRAME_ID:
				inverter_dbc_fl_temp_fault_full_decode(&dtiFL.tempFault, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FL_FOC_FRAME_ID:
				inverter_dbc_fl_foc_full_decode(&dtiFL.foc, (uint8_t *) canMessage.data, 8); break;
			case INVERTER_DBC_FL_MISC_FRAME_ID:
				inverter_dbc_fl_misc_full_decode(&dtiFL.misc, (uint8_t *) canMessage.data, 8); break;
		}
	
	}

}

static void state_machine(){

}

void DTI_set_torque_request(uint8_t dtiNum, uint16_t torque){
	int16_t currentValue = (torque/TORQUE_CONSTANT);
	dtiArr[dtiNum]->current.target_ac_current = inverter_dbc_set_ac_current_target_ac_current_encode(currentValue);
}

static void check_errors(){
	for(int dti=0; dti<4; dti++){
		uint8_t errorCode = dtiArr[dti]->tempFault.actual_fault_code;
		for(int i=0; i<13;i++){
			if(errorCode == DTI_ERR_OVER_VOLT ||
			errorCode == DTI_ERR_UNDER_VOLT ||
			errorCode == DTI_ERR_DRV ||
			errorCode == DTI_ERR_ABS_OVER_CURRENT ||
			errorCode == DTI_ERR_CONTROLLER_OVER_TEMP ||
			errorCode == DTI_ERR_MOTOR_OVER_TEMP ||
			errorCode == DTI_ERR_SENSOR_WIRE ||
			errorCode == DTI_ERR_SENSOR_GEN ||
			errorCode == DTI_ERR_CAN ||
			errorCode == DTI_ERR_ANALOG_INPUT ||
			errorCode == DTI_ERR_INTERNAL ||
			errorCode == DTI_ERR_MCU_OVER_TEMP ||
			errorCode == DTI_ERR_INDEX_LOST){ DTI_set_state(dti, DTIState_SOFT_FAULT);}
			else{
				DTI_set_state(dti, DTIState_NORMAL);
			}
		}
	}

}

void set_zero(uint8_t dtiNum){
	dtiArr[dtiNum]->current.target_ac_current = 0;
}

static bool check_state_change(DTI_s *dti, DTIState_e state){
	
	switch(dti->state){
		case DTIState_NORMAL:
			return true;
		case DTIState_SOFT_PAIRED:
			return (state != DTIState_SOFT_FAULT) && (state != DTIState_NORMAL);
		case DTIState_SOFT_FAULT:
			return (state != DTIState_NORMAL) && (state != DTIState_SOFT_PAIRED);
		case DTIState_HARD_PAIRED:
			return (state == DTIState_HARD_FAULT);
		case DTIState_HARD_FAULT:
			return false;
	
	}

}

void DTI_get_velocities_codegen(float *velArr){
	velArr[0] = dtiFL.erpm_duty_voltage.erpm;
	velArr[1] = dtiRL.erpm_duty_voltage.erpm;
	velArr[2] = dtiFR.erpm_duty_voltage.erpm;
	velArr[3] = dtiRR.erpm_duty_voltage.erpm;

}


static void check_regen(){
	for(int dti = 0; dti < 4; dti++){
		if(dtiArr[dti]->erpm_duty_voltage.erpm < REGEN_MOTORSPEED_RPM_LOW) dtiArr[dti]->hyst = true;
		else if(dtiArr[dti]->erpm_duty_voltage.erpm > REGEN_MOTORSPEED_RPM_HIGH) dtiArr[dti]->hyst = false;
		else if(dtiArr[dti]->hyst) set_zero(dti);
	}
}




