#include "driver_can.h"
#include "config.h"
#include "Inverters.h"
#include "inverter_dbc.h"
#include "main_dbc.h"
#include "VC.h"
#include "DriverInputs.h"
#include "FaultManager.h"
#include "vectornav.h"
#include "DRS.h"
#include "core.h"
#include <math.h>
#include <string.h>

MAIN_BUS mainBus = {0};
// SENSE_BUS senseBus = {0};

static bool CAN_add_filters();
static void send_CAN_errors();
static void send_controls_params();

static int dash_msg_divider = 0;

core_timeout_t fssdb_lost_timeout;          // Brake pressure sensor not on CAN timeout

int main_id_arr[NUM_IDS_MAIN] = {
        MAIN_DBC_BMS_FAULT_VECTOR_FRAME_ID,
        MAIN_DBC_BMS_STATUS_FRAME_ID,
        MAIN_DBC_SSDB_FRONT_FRAME_ID,
        MAIN_DBC_VECTOR_NAV0_FRAME_ID,
        MAIN_DBC_VECTOR_NAV2_FRAME_ID,
        MAIN_DBC_VECTOR_NAV6_FRAME_ID,
        MAIN_DBC_VECTOR_NAV7_FRAME_ID,
        MAIN_DBC_BMS_CURRENT_FRAME_ID,
        MAIN_DBC_BMS_CELL_OVERVIEW_FRAME_ID 
};

int inv_id_arr[NUM_IDS_INV] = {
        INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_RR_AMK_ACTUAL_2_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET3_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET4_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET5_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET6_FRAME_ID,
        INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_RL_AMK_ACTUAL_2_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET3_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET4_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET5_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET6_FRAME_ID,
        INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_FR_AMK_ACTUAL_2_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET3_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET4_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET5_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET6_FRAME_ID,
        INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_FL_AMK_ACTUAL_2_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET3_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET4_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET5_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET6_FRAME_ID
};


bool CAN_init()
{
    if (!core_CAN_init(CAN_INV, 1000000)) return false;
    if (!core_CAN_init(CAN_MAIN, 1000000)) return false;
    if (!core_CAN_init(CAN_SENSE, 1000000)) return false;
    if (!CAN_add_filters()) return false;
    
    /*** FSSDB ***/
    fssdb_lost_timeout.module = NULL;
    fssdb_lost_timeout.ref = FAULT_FSSDB_LOST;
    fssdb_lost_timeout.timeout = DI_TIMEOUT_MS;
    //fssdb_lost_timeout.callback = brake_timeout_callback;
    fssdb_lost_timeout.latching = 0;
    fssdb_lost_timeout.single_shot = 0;
    //core_timeout_insert(&fssdb_lost_timeout);

    return true;
}

bool CAN_tx_main()
{
    core_CAN_send_from_tx_queue_task(CAN_MAIN);
    return false;
}

bool CAN_tx_sense()
{
    rprintf("Starting CAN task\n");
    core_CAN_send_from_tx_queue_task(CAN_SENSE);
    return false;
}

bool CAN_tx_inv()
{
    core_CAN_send_from_tx_queue_task(CAN_INV);
    return false;
}

void CAN_rx_main()
{
    CanMessage_s canMessage;

    if (core_CAN_receive_from_queue(CAN_MAIN, &canMessage))
    {
        int id = canMessage.id;

        switch (id)
        {
            case MAIN_DBC_BMS_FAULT_VECTOR_FRAME_ID:
                main_dbc_bms_fault_vector_unpack(&mainBus.bms_fault_vector, (uint8_t *) &canMessage.data, canMessage.dlc);
                if (canMessage.data & (~(1<<12))) FaultManager_set(FAULT_BMS);
                break;

            case MAIN_DBC_BMS_STATUS_FRAME_ID:
                main_dbc_bms_status_unpack(&mainBus.bms_status, (uint8_t *) &canMessage.data, canMessage.dlc);
                mainBus.bms_status.bms_status_pack_voltage *= PACK_VOLTAGE_SCALE; break;

            // case MAIN_DBC_BMS_CURRENT_LIMIT_FRAME_ID:
                // main_dbc_bms_current_limit_unpack(&mainBus.bms_current_limit, (uint8_t *) &canMessage.data, canMessage.dlc); break;

            case MAIN_DBC_BMS_CURRENT_FRAME_ID:
                main_dbc_bms_current_unpack(&mainBus.bms_current, (uint8_t *) &canMessage.data, canMessage.dlc);
                core_CAN_add_message_to_tx_queue(CAN_MAIN, 7, 8, mainBus.bms_current.bms_inst_current_filt);
                break;

            case MAIN_DBC_SSDB_FRONT_FRAME_ID:
                //core_timeout_reset(&fssdb_lost_timeout);
                main_dbc_ssdb_front_unpack(&mainBus.ssdb_front, (uint8_t *) &canMessage.data, 8); break;

            case MAIN_DBC_VECTOR_NAV0_FRAME_ID:
                main_dbc_vector_nav0_unpack(&mainBus.vn0, (uint8_t *) &canMessage.data, canMessage.dlc); break;

            case MAIN_DBC_VECTOR_NAV2_FRAME_ID:
                main_dbc_vector_nav2_unpack(&mainBus.vn2, (uint8_t *) &canMessage.data, canMessage.dlc); break;

            case MAIN_DBC_VECTOR_NAV6_FRAME_ID:
                main_dbc_vector_nav6_unpack(&mainBus.vn6, (uint8_t *) &canMessage.data, canMessage.dlc); break;

            case MAIN_DBC_VECTOR_NAV7_FRAME_ID:
                main_dbc_vector_nav7_unpack(&mainBus.vn7, (uint8_t *) &canMessage.data, canMessage.dlc); break;

            case MAIN_DBC_BMS_CELL_OVERVIEW_FRAME_ID:
                main_dbc_bms_cell_overview_unpack(&mainBus.bms_cells, (uint8_t *) &canMessage.data, canMessage.dlc);
                break;

            case MAIN_DBC_DS_STATUS_FRAME_ID:
                main_dbc_ds_status_unpack(&mainBus.ds_status, (uint8_t*)&canMessage.data, canMessage.dlc);
                break;

            case MAIN_DBC_RSS_PDO_FRAME_ID:
                main_dbc_rss_pdo_unpack(&mainBus.rss_pdo, (uint8_t*)(&canMessage.data), canMessage.dlc);
                break;
            case 510:
                //DRS_set_position(canMessage.data >> 32);
                break;
        }
    }
}

void CAN_rx_secondary(void *arg) {
    (void) arg;
    CanExtendedMessage_s msg;
    while (1) {
        if (core_CAN_receive_extended_from_queue(CAN_SENSE, &msg)) {
            switch (msg.id) {
                case SENSOR_DBC_DASH_INPUTS_FRAME_ID:
                    rprintf("DASH %x\n", msg.data[0]);
                    mainBus.dash_buttons = msg.data[0] ^ 0x01;
                    break;
                 default:
                    break;
            }
        }
    }
}

void CAN_Task_Update()
{ 
    uint64_t msg;

    Inverters_set_can_states();
    main_dbc_vc_inverter_status_pack((uint8_t *)&msg, &mainBus.inverter_status, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_INVERTER_STATUS_FRAME_ID, 8, msg);

    main_dbc_vc_processed_inputs_pack((uint8_t *)&msg, &mainBus.processed_inputs, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_PROCESSED_INPUTS_FRAME_ID, 8, msg);

    main_dbc_vc_pedal_inputs_raw_pack((uint8_t *)&msg, &mainBus.pedal_inputs_raw, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_PEDAL_INPUTS_RAW_FRAME_ID, 8, msg);
    
    // Dash temperature messages
    if ((++dash_msg_divider) == 100) {
        memcpy(&msg, mainBus.motor_temps, 8);
        core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_MOTOR_TEMPS_FRAME_ID, 8, msg);
        memcpy(&msg, mainBus.inverter_temps, 8);
        core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_INVERTER_TEMPS_FRAME_ID, 8, msg);
        dash_msg_divider = 0;
    }

    // Inverters_send_timeout_times();
    //Inverters_echo_on_main();
    vectornav_send_errors();
    send_CAN_errors();
    send_controls_params();
}

static void send_CAN_errors() {
#ifndef VC_TEST
    uint64_t msg = 0;
    ((uint16_t *)&msg)[0] = core_CAN_errors.arbitration_error;
    ((uint16_t *)&msg)[1] = core_CAN_errors.data_error;

    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CAN_ERRORS_FRAME_ID, 8, msg);
#endif
}

static bool CAN_add_filters()
{
    int minFilter;
    int maxFilter;
    bool status = true;

    minFilter = main_id_arr[0];
    maxFilter = main_id_arr[0];
    for (int i = 1; i < NUM_IDS_MAIN; i++)
    {
        if (main_id_arr[i] < minFilter) minFilter = main_id_arr[i];
        if (main_id_arr[i] > maxFilter) maxFilter = main_id_arr[i];
    }
    status = (status && core_CAN_add_filter(CAN_MAIN, false, minFilter, maxFilter));

    minFilter = inv_id_arr[0];
    maxFilter = inv_id_arr[0];
    for (int i = 1; i < NUM_IDS_INV; i++)
    {
        if (inv_id_arr[i] < minFilter) minFilter = inv_id_arr[i];
        if (inv_id_arr[i] > maxFilter) maxFilter = inv_id_arr[i];
    }
    status = (status && core_CAN_add_filter(CAN_INV, false, minFilter, maxFilter));

    status = (status && core_CAN_add_filter(CAN_SENSE, false, SENSOR_DBC_DASH_INPUTS_FRAME_ID, SENSOR_DBC_DASH_INPUTS_FRAME_ID));
   
    return status;
}

static void send_controls_params()
{
    mainBus.controls_const1.vc_long_factor = main_dbc_vc_controls_constants1_vc_long_factor_encode(CG_LONG_FACTOR);
    //mainBus.controls_const1.vc_target_slip_ratio = main_dbc_vc_controls_constants1_vc_target_slip_ratio_encode(CG_TARGET_SR_NOMINAL);
    mainBus.controls_const1.vc_k_p_slip_ratio = main_dbc_vc_controls_constants1_vc_k_p_slip_ratio_encode(fabsf(CG_KP_SLIP_RATIO));
    mainBus.controls_const1.vc_k_i_slip_ratio = main_dbc_vc_controls_constants1_vc_k_i_slip_ratio_encode(fabsf(CG_KI_SLIP_RATIO));
    mainBus.controls_const1.vc_tc_activation_threshold = main_dbc_vc_controls_constants1_vc_tc_activation_threshold_encode(CG_TC_ACTIVATION_THRESHOLD);
    mainBus.controls_const1.vc_k_p_yaw_rate = main_dbc_vc_controls_constants1_vc_k_p_yaw_rate_encode(CG_KP_YAW_RATE);
    mainBus.controls_const1.vc_k_i_yaw_rate = main_dbc_vc_controls_constants1_vc_k_i_yaw_rate_encode(CG_KI_YAW_RATE);
    mainBus.controls_const2.vc_max_desired_yaw_rate = main_dbc_vc_controls_constants2_vc_max_desired_yaw_rate_encode(CG_MAX_DESIRED_YAW_RATE);
    mainBus.controls_const2.vc_understeer_gradient = main_dbc_vc_controls_constants2_vc_understeer_gradient_encode(CG_UNDERSTEER_GRADIENT);
    mainBus.controls_const2.vc_static_long_split = main_dbc_vc_controls_constants2_vc_static_long_split_encode(CG_STATIC_LONG_SPLIT);
    mainBus.controls_const2.vc_k_f_yaw_rate = main_dbc_vc_controls_constants2_vc_k_f_yaw_rate_encode(CG_KF_YAW_RATE);

    uint64_t msg;
    main_dbc_vc_controls_constants1_pack((uint8_t *)&msg, &mainBus.controls_const1, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_CONSTANTS1_FRAME_ID, 8, msg);

    msg = 0;
    main_dbc_vc_controls_constants2_pack((uint8_t *)&msg, &mainBus.controls_const2, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_CONTROLS_CONSTANTS2_FRAME_ID, 8, msg);


    /*
    uint64_t msg = 0;
    ((uint16_t *) &msg)[0] = ((uint16_t) (CS_LAT_FACTOR_ACC * 100));
    ((uint16_t *) &msg)[1] = ((uint16_t) (PL_MAX_POWER_W * 100));
    ((uint16_t *) &msg)[2] = ((uint16_t) (CS_TOTAL_GAIN * 100));
    ((uint16_t *) &msg)[3] = ((uint16_t) (ENDURANCE_CURRENT_LIMIT));
    core_CAN_add_message_to_tx_queue(CAN_MAIN, 3, 8, msg);
    */
}
