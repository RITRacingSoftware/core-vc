#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "inverter_dbc.h"


//Fault Codes
#define DTI_OVER_VOLT_ERR 1 //input voltage was higher then set max 
#define DTI_UNDER_VOLT_ERR 2 //input voltage was lower then set min
#define DTI_DRV_ERR 3 // transistor or transistor drive error
#define DTI_ABS_OVER_CURRENT_ERR 4 //AC current is higher than absolute max
#define DTI_CONTROLLER_OVER_TEMP_ERR 5 //controller temp higher then set max
#define DTI_MOTOR_OVER_TEMP_ERR 6 //motor temp higher then set max
#define DTI_SENSOR_WIRE_ERR 7 //issue with sensor differential signals
#define DTI_SENSOR_GEN_ERR 8 //sensor processing error
#define DTI_CAN_ERR 9 //invalid CAN command received 
#define DTI_ANALOG_INPUT_ERR 10 //excessive deviation in redundant config in HV550/HV850
#define DTI_INTERNAL_ERR 11 //internal hardware fault
#define DTI_MCU_OVER_TEMP_ERR 12 //MCU temp too high
#define DTI_INDEX_LOST_ERR 13 //absolute position index lost 

typedef enum{
	DTI_SOFT_FAULT,
	DTI_HARD_FAULT,
	DTI_STANDBY,

} DTIState_e


typedef struct{
	struct dti_dbc_info_1_t info;
	struct dti_dbc_current_1_t current;
	struct dti_dbc_temp_fault_1_t tempFault;
	struct dti_dbc_foc_values_1_t foc;
	DTIState_e state;
} DTI_s;

void DTI_init();
void DTI_Task_Update();
void DTI_update();


//Getters
DTIState_e DTI_get_state(uint8_t dtiNum);
bool DTI_get_enable_all();
bool DTI_get_enable_any();
bool DTI_get_precharged_all();

//Setters for CAN messages
void DTI_set_current();
void DTI_set_input_voltage();
void DTI_set_brake_current();
void DTI_set_rpm();
void DTI_set_postion();
void DTI_set_temps();
void DTI_set_torque_request();

void DTI_set_state(uint8_t dtiNum, DTIState_e state);
void DTI_set_can_states();
void DTI_CAN_rx();

