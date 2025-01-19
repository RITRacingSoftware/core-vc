#pragma once

#define VC_NOT_READY 1
#define INVERTERS_POWERED 2
#define PRECHARGING 3
#define WAIT 4
#define STANDBY 5
#define RTD 6
#define SHUTDOWN 7

#define RTD_HOLD_TIME 1000


typedef enum
{
    VehicleState_VC_NOT_READY, // Doing nothing, default state
    VehicleState_INVERTERS_POWERED, // Reads for inverters saying they're ready
    VehicleState_PRECHARGING, // When precharge done button is pressed, send it out,
    // receive echo and confirmation, flip interlock relay
    VehicleState_WAIT, // Wait for enable button to be pressed
    VehicleState_STANDBY, // Send inverter enable and on, receive inverter on echos and confirmation, set BE2
    VehicleState_READY_TO_DRIVE, // Actively running, send torque request every 200ms. Wait for stop button
    VehicleState_SHUTDOWN // Turn off everything in sequence
} VehicleState_e;

void VehicleState_init();
void VehicleState_Task_Update();