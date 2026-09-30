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
	DTI_NORMAL,
	DTI_RESETTING,
	DTI_HARD_PAIRED,
	DTI_HARD_FAULT
} DTIState_e


typedef struct{
	struct dti_dbc__1_t info_1;
	struct dti_dbc_current_t current;
	struct dti_dbc_temp_fault_t tempFault;
	struct dti_dbc_foc_values_t foc;
	struct dti_dbc_misc_t
	DTIState_e state;
} DTI_s;

void DTI_init();
void DTI_Task_Update();
void DTI_update();


//Getters
DTIState_e DTI_get_state(uint8_t dtiNum);
bool DTI_get_RPM(void);
bool DTI_get_duty_cycle(void);
bool DTI_get_input_voltage(void);
bool DTI_get_current(void);
bool DTI_get_temps(void);
bool DTI_get_faults(void);
bool DTI_get_FOC(void);
bool DTI_get_enable(void);

//Setters for CAN messages
void DTI_set_current(int_16 val);
void DTI_set_brake_current(uint16_t val);
void DTI_set_RPM(int32_t val);
void DTI_set_motor_postion(uint16_t val);
void DTI_set_relative_current(int16_t val);
void DTI_set_relative_brake_current(uint16_t val);
void DTI_set_digital_output(uint8_t val);
void DTI_set_max_AC_current(uint16_t val);
void DTI_set_max_AC_brake_current(int16_t val);
void DTI_set_max_DC_current(uint16_t val);
void DTI_set_max_DC_brake_current(int16_t val);
void DTI_set_drive_enable(uint8_t val);

void DTI_set_state(uint8_t dtiNum, DTIState_e state);
void DTI_set_can_states();
void DTI_CAN_rx();

