#include "driver_can.h"
#include "config.h"
#include "can.h"
#include "usart.h"
#include "Inverters.h"
#include "inverter_dbc.h"
#include "main_dbc.h"
#include "rtt.h"
#include "VC.h"
#include "DriverInputs.h"
#include "FaultManager.h"

MAIN_BUS mainBus;
INV_BUS invBus;

static uint8_t inc;

static bool CAN_add_filters();
static void pack_and_send_main_echoes(int id);

int main_id_arr[NUM_IDS_MAIN] = {
        MAIN_DBC_SSDB_FRONT_FRAME_ID,
        MAIN_DBC_SSDB_VECTOR_NAV6_FRAME_ID
};

int inv_id_arr[NUM_IDS_INV] = {
        INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_RR_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_RL_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_FR_AMK_RIT_SET2_FRAME_ID,
        INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET1_FRAME_ID,
        INVERTER_DBC_FL_AMK_RIT_SET2_FRAME_ID
};


bool CAN_init()
{
    inc = 0;
    if (!core_CAN_init(CAN_INV)) return false;
    if (!core_CAN_init(CAN_MAIN)) return false;
    if (!core_CAN_init(CAN_SENSE)) return false;
    if (!CAN_add_filters()) return false;
    return true;
}

bool CAN_tx_main()
{
    core_CAN_send_from_tx_queue_task(CAN_MAIN);
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
                if ((uint8_t *) &canMessage.data) FaultManager_set(FAULT_BMS);
                break;

            case MAIN_DBC_SSDB_FRONT_FRAME_ID:
                main_dbc_ssdb_front_unpack(&mainBus.ssdb_front, (uint8_t *) &canMessage.data, canMessage.dlc);
                break;

            case MAIN_DBC_SSDB_VECTOR_NAV6_FRAME_ID:
                main_dbc_ssdb_vector_nav6_unpack(&mainBus.vn_vel, (uint8_t *) &canMessage.data, canMessage.dlc); break;
        }
    }
}

void CAN_rx_inv()
{
    CanMessage_s canMessage;

    if (core_CAN_receive_from_queue(CAN_INV, &canMessage))
    {
        int id = canMessage.id;
       // rprintf("Got inv: %d\n", id);

        switch (id)
        {
            // RR
            case INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_rr_amk_actual_1_unpack(&invBus.rr_actual1, (uint8_t *) &canMessage.data, 8);
                if (invBus.rr_actual1.rr_status_error) {FaultManager_set(FAULT_RR_ERROR); rprintf("RR Faulted");}
                core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RR_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_RR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rr_amk_rit_set1_unpack(&invBus.rr_set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rr_amk_rit_set2_unpack(&invBus.rr_set2, (uint8_t *) &canMessage.data, 8); break;

            // RL
            case INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_rl_amk_actual_1_unpack(&invBus.rl_actual1, (uint8_t *) &canMessage.data, 8);
                if (invBus.rl_actual1.rl_status_error) FaultManager_set(FAULT_RL_ERROR);
                core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_RL_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_RL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rl_amk_rit_set1_unpack(&invBus.rl_set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_RL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rl_amk_rit_set2_unpack(&invBus.rl_set2, (uint8_t *) &canMessage.data, 8); break;

            // FR
            case INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_fr_amk_actual_1_unpack(&invBus.fr_actual1, (uint8_t *) &canMessage.data, 8);
                if (invBus.fr_actual1.fr_status_error) FaultManager_set(FAULT_FR_ERROR);
                core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FR_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_FR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_fr_amk_rit_set1_unpack(&invBus.fr_set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_fr_amk_rit_set2_unpack(&invBus.fr_set2, (uint8_t *) &canMessage.data, 8); break;

            // FL
            case INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_fl_amk_actual_1_unpack(&invBus.fl_actual1, (uint8_t *) &canMessage.data, 8);
                if (invBus.fl_actual1.fl_status_error) FaultManager_set(FAULT_FL_ERROR);
                core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_FL_AMK_ACTUAL_1_FRAME_ID, canMessage.dlc, canMessage.data); break;   // Echo over main bus

            case INVERTER_DBC_FL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_fl_amk_rit_set1_unpack(&invBus.fl_set1, (uint8_t *) &canMessage.data, 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_fl_amk_rit_set2_unpack(&invBus.fl_set2, (uint8_t *) &canMessage.data, 8); break;
        }
        Inverters_update();

    }

}

int CAN_pack_message(int id, uint8_t *msg_data)
{
    switch (id)
    {
        // Inverter setpoints
        case INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_rr_amk_setpoints_pack(msg_data, &invBus.rr_setpoints, 8);

        case INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID:
//            uprintf(USART3, "SP: %d\n", invBus.rl_setpoints.rl_amk_torque_setpoint);
            return inverter_dbc_rl_amk_setpoints_pack(msg_data, &invBus.rl_setpoints, 8);

        case INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_fr_amk_setpoints_pack(msg_data, &invBus.fr_setpoints, 8);

        case INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_fl_amk_setpoints_pack(msg_data, &invBus.fl_setpoints, 8);
    }

    return -1;
}

void CAN_send_driver_inputs()
{
    uint64_t message;

    main_dbc_vc_pedal_inputs_pack((uint8_t *)&message, &mainBus.pedal_inputs, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_PEDAL_INPUTS_FRAME_ID, 8, message);

    main_dbc_vc_pedal_inputs_raw_pack((uint8_t *)&message, &mainBus.pedal_inputs_raw, 8);
    core_CAN_add_message_to_tx_queue(CAN_MAIN, MAIN_DBC_VC_PEDAL_INPUTS_RAW_FRAME_ID, 8, message);
}

void CAN_echo_on_main()
{
    // RR
    mainBus.rr_info1.vc_rr_error_list1 = invBus.rr_set1.rr_error_list1;
    mainBus.rr_info1.vc_rr_error_list2 = invBus.rr_set1.rr_error_list2;
    mainBus.rr_info2.vc_rr_error_list3 = invBus.rr_set2.rr_error_list3;
    mainBus.rr_info2.vc_rr_error_info = invBus.rr_actual2.rr_error_info;
    mainBus.rr_info2.vc_rr_temp_inverter = invBus.rr_actual2.rr_temp_inverter;
    mainBus.rr_info3.vc_rr_temp_motor = invBus.rr_actual2.rr_temp_motor;

    //RL
    mainBus.rl_info1.vc_rl_error_list1 = invBus.rl_set1.rl_error_list1;
    mainBus.rl_info1.vc_rl_error_list2 = invBus.rl_set1.rl_error_list2;
    mainBus.rl_info2.vc_rl_error_list3 = invBus.rl_set2.rl_error_list3;
    mainBus.rl_info2.vc_rl_error_info = invBus.rl_actual2.rl_error_info;
    mainBus.rl_info2.vc_rl_temp_inverter = invBus.rl_actual2.rl_temp_inverter;
    mainBus.rl_info3.vc_rl_temp_motor = invBus.rl_actual2.rl_temp_motor;

    //FR
    mainBus.fr_info1.vc_fr_error_list1 = invBus.fr_set1.fr_error_list1;
    mainBus.fr_info1.vc_fr_error_list2 = invBus.fr_set1.fr_error_list2;
    mainBus.fr_info2.vc_fr_error_list3 = invBus.fr_set2.fr_error_list3;
    mainBus.fr_info2.vc_fr_error_info = invBus.fr_actual2.fr_error_info;
    mainBus.fr_info2.vc_fr_temp_inverter = invBus.fr_actual2.fr_temp_inverter;
    mainBus.fr_info3.vc_fr_temp_motor = invBus.fr_actual2.fr_temp_motor;

    //FL
    mainBus.fl_info1.vc_fl_error_list1 = invBus.fl_set1.fl_error_list1;
    mainBus.fl_info1.vc_fl_error_list2 = invBus.fl_set1.fl_error_list2;
    mainBus.fl_info2.vc_fl_error_list3 = invBus.fl_set2.fl_error_list3;
    mainBus.fl_info2.vc_fl_error_info = invBus.fl_actual2.fl_error_info;
    mainBus.fl_info2.vc_fl_temp_inverter = invBus.fl_actual2.fl_temp_inverter;
    mainBus.fl_info3.vc_fl_temp_motor = invBus.fl_actual2.fl_temp_motor;



    pack_and_send_main_echoes(MAIN_DBC_VC_RR_INFO_1_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_RR_INFO_2_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_RR_INFO_3_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_RL_INFO_1_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_RL_INFO_2_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_RL_INFO_3_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FR_INFO_1_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FR_INFO_2_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FR_INFO_3_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FL_INFO_1_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FL_INFO_2_FRAME_ID);
    pack_and_send_main_echoes(MAIN_DBC_VC_FL_INFO_3_FRAME_ID);
}

static void pack_and_send_main_echoes(int id)
{
    uint64_t msg_data;

    switch (id)
    {
        // RR
        case MAIN_DBC_VC_RR_INFO_1_FRAME_ID:
            main_dbc_vc_rr_info_1_pack((uint8_t *)&msg_data, &mainBus.rr_info1, 8); break;

        case MAIN_DBC_VC_RR_INFO_2_FRAME_ID:
            main_dbc_vc_rr_info_2_pack((uint8_t *)&msg_data, &mainBus.rr_info2, 8); break;

        case MAIN_DBC_VC_RR_INFO_3_FRAME_ID:
            main_dbc_vc_rr_info_3_pack((uint8_t *)&msg_data, &mainBus.rr_info3, 8); break;

            //RL
        case MAIN_DBC_VC_RL_INFO_1_FRAME_ID:
            main_dbc_vc_rl_info_1_pack((uint8_t *)&msg_data, &mainBus.rl_info1, 8); break;

        case MAIN_DBC_VC_RL_INFO_2_FRAME_ID:
            main_dbc_vc_rl_info_2_pack((uint8_t *)&msg_data, &mainBus.rl_info2, 8); break;

        case MAIN_DBC_VC_RL_INFO_3_FRAME_ID:
            main_dbc_vc_rl_info_3_pack((uint8_t *)&msg_data, &mainBus.rl_info3, 8); break;

            //FR
        case MAIN_DBC_VC_FR_INFO_1_FRAME_ID:
            main_dbc_vc_fr_info_1_pack((uint8_t *)&msg_data, &mainBus.fr_info1, 8); break;

        case MAIN_DBC_VC_FR_INFO_2_FRAME_ID:
            main_dbc_vc_fr_info_2_pack((uint8_t *)&msg_data, &mainBus.fr_info2, 8); break;

        case MAIN_DBC_VC_FR_INFO_3_FRAME_ID:
            main_dbc_vc_fr_info_3_pack((uint8_t *)&msg_data, &mainBus.fr_info3, 8); break;

            //FL
        case MAIN_DBC_VC_FL_INFO_1_FRAME_ID:
            main_dbc_vc_fl_info_1_pack((uint8_t *)&msg_data, &mainBus.fl_info1, 8); break;

        case MAIN_DBC_VC_FL_INFO_2_FRAME_ID:
            main_dbc_vc_fl_info_2_pack((uint8_t *)&msg_data, &mainBus.fl_info2, 8); break;

        case MAIN_DBC_VC_FL_INFO_3_FRAME_ID:
            main_dbc_vc_fl_info_3_pack((uint8_t *)&msg_data, &mainBus.fl_info3, 8); break;
    }
//    core_CAN_add_message_to_tx_queue(CAN_MAIN, id, 8, msg_data);
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
   
    return status;
}
