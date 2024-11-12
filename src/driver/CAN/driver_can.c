#include "driver_can.h"
#include "can.h"
#include "Inverters/Inverters.h"
#include "inverter_dbc.h"
#include "main_dbc.h"
#include "VC/VC.h"

#define CAN_SENSE FDCAN1
#define CAN_MAIN FDCAN2
#define CAN_INV FDCAN3

SENSE_BUS senseBus;
MAIN_BUS mainBus;
INV_BUS invBus;

static bool CAN_add_filters();

int main_id_arr[NUM_IDS_MAIN] = {
        MAIN_DBC_SSDB_BRAKE_PRESSURE_FRONT_FRAME_ID,
        MAIN_DBC_SSDB_STEERING_ANGLE_FRAME_ID
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
    if (!core_CAN_init(CAN_MAIN)) return false;
    if (!core_CAN_init(CAN_INV)) return false;
    if (!CAN_add_filters()) return false;
}

bool CAN_tx()
{
    core_CAN_send_from_tx_queue_task(CAN_MAIN);
    core_CAN_send_from_tx_queue_task(CAN_INV);
    return false;
}

void CAN_rx()
{
    CanMessage_s canMessage;

    if (core_CAN_receive_from_queue(CAN_MAIN, &canMessage))
    {
        uint8_t data[8];
        for (int i = 0; i < 8; i++)
        {
            data[i] = (canMessage.data >> (i * 8)) & 0xff;
        }
        int id = canMessage.id;

        switch (id)
        {
            case MAIN_DBC_SSDB_BRAKE_PRESSURE_FRONT_FRAME_ID:
                main_dbc_ssdb_brake_pressure_front_unpack(&mainBus.front_bps, &data[0], canMessage.dlc); break;

            case MAIN_DBC_SSDB_STEERING_ANGLE_FRAME_ID:
                main_dbc_ssdb_steering_angle_unpack(&mainBus.steering_angle, &data[0], canMessage.dlc); break;
        }
    }

    if (core_CAN_receive_from_queue(CAN_INV, &canMessage))
    {
        uint8_t data[8];
        for (int i = 0; i < 8; i++)
        {
            data[i] = (canMessage.data >> (i * 8)) & 0xff;
        }
        int id = canMessage.id;

        switch (id)
        {
            // RR
            case INVERTER_DBC_RR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_rr_amk_actual_1_unpack(&invBus.rr_actual1, &data[0], 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rr_amk_rit_set1_unpack(&invBus.rr_set1, &data[0], 8); break;

            case INVERTER_DBC_RR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rr_amk_rit_set2_unpack(&invBus.rr_set2, &data[0], 8); break;

            // RL
            case INVERTER_DBC_RL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_rl_amk_actual_1_unpack(&invBus.rl_actual1, &data[0], 8); break;

            case INVERTER_DBC_RL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_rl_amk_rit_set1_unpack(&invBus.rl_set1, &data[0], 8); break;

            case INVERTER_DBC_RL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_rl_amk_rit_set2_unpack(&invBus.rl_set2, &data[0], 8); break;

            // FR
            case INVERTER_DBC_FR_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_fr_amk_actual_1_unpack(&invBus.fr_actual1, &data[0], 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_fr_amk_rit_set1_unpack(&invBus.fr_set1, &data[0], 8); break;

            case INVERTER_DBC_FR_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_fr_amk_rit_set2_unpack(&invBus.fr_set2, &data[0], 8); break;

            // FL
            case INVERTER_DBC_FL_AMK_ACTUAL_1_FRAME_ID:
                inverter_dbc_fl_amk_actual_1_unpack(&invBus.fl_actual1, &data[0], 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET1_FRAME_ID:
                inverter_dbc_fl_amk_rit_set1_unpack(&invBus.fl_set1, &data[0], 8); break;

            case INVERTER_DBC_FL_AMK_RIT_SET2_FRAME_ID:
                inverter_dbc_fl_amk_rit_set2_unpack(&invBus.fl_set2, &data[0], 8); break;
        }
        Inverters_update();

    }

}

int CAN_pack_message(int id, uint8_t *msg_data)
{
    switch (id)
    {
        case INVERTER_DBC_RR_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_rr_amk_setpoints_pack(msg_data, &invBus.rr_setpoints, 8);

        case INVERTER_DBC_RR_AMK_SETPOINTS2_FRAME_ID:
            return inverter_dbc_rr_amk_setpoints2_pack(msg_data, &invBus.rr_setpoints2, 2);

        case INVERTER_DBC_RL_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_rl_amk_setpoints_pack(msg_data, &invBus.rl_setpoints, 8);

        case INVERTER_DBC_RL_AMK_SETPOINTS2_FRAME_ID:
            return inverter_dbc_rl_amk_setpoints2_pack(msg_data, &invBus.rl_setpoints2, 2);

        case INVERTER_DBC_FR_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_fr_amk_setpoints_pack(msg_data, &invBus.fr_setpoints, 8);

        case INVERTER_DBC_FR_AMK_SETPOINTS2_FRAME_ID:
            return inverter_dbc_fr_amk_setpoints2_pack(msg_data, &invBus.fr_setpoints2, 2);

        case INVERTER_DBC_FL_AMK_SETPOINTS_FRAME_ID:
            return inverter_dbc_fl_amk_setpoints_pack(msg_data, &invBus.fl_setpoints, 8);

        case INVERTER_DBC_FL_AMK_SETPOINTS2_FRAME_ID:
            return inverter_dbc_fl_amk_setpoints2_pack(msg_data, &invBus.fl_setpoints2, 2);

        default:
            break;
    }

    return -1;
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