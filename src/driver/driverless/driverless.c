#include <stdint.h>
#include <stdbool.h>
#include "timeout.h"

#include "config.h"
#include "FaultManager.h"
#include "VehicleState.h"
#include "Inverters.h"
#include "driver_can.h"

static core_timeout_t ds_timeout;

static timeout_callback(core_timeout_t *to) {
    FaultManager_set(FAULT_DS_TIMEOUT);
}

void driverless_init() {

#ifdef DRIVERLESS_ENABLED
    ds_timeout.callback = timeout_callback;
    ds_timeout.timeout = 50;
    core_timeout_insert(&ds_timeout);
#endif

}

void driverless_state_update() {
    VehicleState_e vs = VehicleState_get_state();
    // Update DS enable bits
    mainBus.vc_status.vc_ds_rear_enable = (vs == VehicleState_RTD_AS) && (Inverters_get_state(0) == InvState_NORMAL) && (Inverters_get_state(1) == InvState_NORMAL);
    mainBus.vc_status.vc_ds_front_enable = (vs == VehicleState_RTD_AS) && (Inverters_get_state(2) == InvState_NORMAL) && (Inverters_get_state(3) == InvState_NORMAL);
    // Check that the datalogger releases the inverters. If not, turn off the car
    if ((mainBus.vc_status.vc_ds_rear_enable || !(mainBus.ds_status.ds_rear_active)) && (mainBus.vc_status.vc_ds_front_enable || !(mainBus.ds_status.ds_front_active))) {
        core_timeout_reset(&ds_timeout);
    }
}

// For torque sentpoints from the VC to be enabled, the VC must both not
// request setpoints from the DS AND the DS must not be currently claiming
// the inverters. The case where the VC is not requesting setpoints from the
// DS but the DS continues to claim the inverters is handled by the ds_timeout.
bool driverless_rear_enabled() {
    return !(mainBus.vc_status.vc_ds_rear_enable) && !(mainBus.ds_status.ds_rear_active);
}

bool driverless_front_enabled() {
    return !(mainBus.vc_status.vc_ds_front_enable) && !(mainBus.ds_status.ds_front_active);
}
