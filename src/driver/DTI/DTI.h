#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "inverter_dbc.h"

#define DTI_RR 1
#define DTI_RL 2
#define DTI_FR 3
#define DTI_FL 4

#define DRIVE_ENABLE_OFF 0
#define DRIVE_ENABLE_ON 1

#define TORQUE_CONSTANT 0.492f

//Fault Codes
#define DTI_ERR_OVER_VOLT 1 //input voltage was higher then set max 
#define DTI_ERR_UNDER_VOLT 2 //input voltage was lower then set min
#define DTI_ERR_DRV 3 // transistor or transistor drive error
#define DTI_ERR_ABS_OVER_CURRENT 4 //AC current is higher than absolute max
#define DTI_ERR_CONTROLLER_OVER_TEMP 5 //controller temp higher then set max
#define DTI_ERR_MOTOR_OVER_TEMP 6 //motor temp higher then set max
#define DTI_ERR_SENSOR_WIRE 7 //issue with sensor differential signals
#define DTI_ERR_SENSOR_GEN 8 //sensor processing error
#define DTI_ERR_CAN 9 //invalid CAN command received 
#define DTI_ERR_ANALOG_INPUT 10 //excessive deviation in redundant config in HV550/HV850
#define DTI_ERR_INTERNAL 11 //internal hardware fault
#define DTI_ERR_MCU_OVER_TEMP 12 //MCU temp too high
#define DTI_ERR_INDEX_LOST 13 //absolute position index lost 

typedef enum{
	DTIState_NORMAL,
	DTIState_SOFT_PAIRED,
	DTIState_SOFT_FAULT,
	DTIState_HARD_PAIRED,
	DTIState_HARD_FAULT
} DTIState_e;


typedef enum{
	ResetState_0,
	ResetState_1,
	ResetState_2
} ResetState_e;

typedef struct{

	struct inverter_dbc_erpm_duty_voltage_t erpm_duty_voltage;
	struct inverter_dbc_ac_dc_current_t receivedCurrent;
	struct inverter_dbc_set_ac_current_t current;
	struct inverter_dbc_temp_fault_t tempFault;
	struct inverter_dbc_foc_t foc;
	struct inverter_dbc_misc_t misc;
	struct inverter_dbc_enable_t enable;
	ResetState_e reset_state;
	bool hyst;
	DTIState_e state;
} DTI_s;

void DTI_init();
void DTI_Task_Update();
void DTI_update();


//Getters
DTIState_e DTI_get_state(uint8_t dtiNum);
bool DTI_get_precharged_all(void);
bool DTI_get_drive_enable_all(void);
void DTI_get_velocities_codegen(float *velArr);
void DTI_get_voltages(float *volArr);
void DTI_get_torques(float *trqArr);
bool DTI_get_dc_on_all();


//Setters
void DTI_set_drive_enable_all(uint8_t val);
void DTI_set_state(uint8_t dtiNum, DTIState_e state);
void DTI_set_torque_request(uint8_t dtiNum, uint16_t torque);
void DTI_set_state(uint8_t dtiNum, DTIState_e state);
void DTI_set_can_states();


void DTI_set_state(uint8_t dtiNum, DTIState_e state);
void DTI_set_can_states();
void DTI_CAN_rx();
void check_errors();
