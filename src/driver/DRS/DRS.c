#include <stdint.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"
#include "core.h"
#include "config.h"

#include "sensor_dbc.h"

static struct sensor_dbc_vc_drs_status_t drs_status;

void DRS_init() {
#ifdef DRS_ENABLED
    __HAL_RCC_TIM1_CLK_ENABLE();
    // Prescaler of 50
    TIM1->PSC = 49;
    // 16MHZ / 64000 / 50 = 50Hz -> 20ms period
    TIM1->ARR = 64000;
    TIM1->CNT = 0;
    // Output 1: 1.5ms (center)
    TIM1->CCR2 = DRS_SERVO1_CLOSED;
    // Output 2: 1.5ms (center)
    TIM1->CCR3 = DRS_SERVO2_CLOSED;
    // Output 1 in normal PWM mode, output 2 in inverted PWM mode
    TIM1->CCMR1 = TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    // Output 3 in normal PWM mode
    TIM1->CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;
    // Output positive and negative outputs for channel 2 and 3
    TIM1->CCER = TIM_CCER_CC2NE | TIM_CCER_CC3NE;
    // Enable center aligned counting and ARR preload
    TIM1->CR1 =  TIM_CR1_ARPE;
    // Generate update event to update auto-reload register and CCR1/CCR2/CCR3
    TIM1->EGR = TIM_EGR_UG;
    // Start the counter
    TIM1->CR1 |= TIM_CR1_CEN;
    
    TIM1->BDTR = TIM_BDTR_MOE;
    
    HAL_GPIO_Init(GPIOB, &((GPIO_InitTypeDef){0x0003, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, 6}));
#else
    // If DRS is disabled configure the pins as outputs and pull them low so
    // that noise can't accidentally trigger the servos.
    core_GPIO_init(GPIOB, GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_init(GPIOB, GPIO_PIN_1, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    core_GPIO_digital_write(GPIOB, GPIO_PIN_0, 0);
    core_GPIO_digital_write(GPIOB, GPIO_PIN_1, 0);
#endif
}

static void DRS_set(uint16_t pwm1, uint16_t pwm2) {
    TIM1->CCR2 = pwm1;
    TIM1->CCR3 = pwm2;
}

void DRS_set_position(bool open) {
    if (open) drs_status.vc_drs_ramp_set = DRS_RAMP_TIME;
    else drs_status.vc_drs_ramp_set = 0;
}

void DRS_task() {
    uint64_t msg;
    int servo1_offset = DRS_SERVO1_CLOSED, servo1_range = DRS_SERVO1_OPEN - DRS_SERVO1_CLOSED;
    int servo2_offset = DRS_SERVO2_CLOSED, servo2_range = DRS_SERVO2_OPEN - DRS_SERVO2_CLOSED;
    if (drs_status.vc_drs_ramp_set > drs_status.vc_drs_ramp_count) {
        // Opening
        drs_status.vc_drs_ramp_count++;
        drs_status.vc_drs_overdrive_count = DRS_OVERDRIVE_TIME;
    }
    else if (drs_status.vc_drs_ramp_set < drs_status.vc_drs_ramp_count) {
        // Closing
        drs_status.vc_drs_ramp_count--;
        drs_status.vc_drs_overdrive_count = DRS_OVERDRIVE_TIME;
    }
    if (drs_status.vc_drs_overdrive_count) {
        servo1_offset = DRS_SERVO1_CLOSED + DRS_SERVO1_CLOSED_OVERDRIVE;
        servo2_offset = DRS_SERVO2_CLOSED + DRS_SERVO2_CLOSED_OVERDRIVE;
        servo1_range = DRS_SERVO1_OPEN - DRS_SERVO1_CLOSED + DRS_SERVO1_OPEN_OVERDRIVE - DRS_SERVO1_CLOSED_OVERDRIVE;
        servo2_range = DRS_SERVO2_OPEN - DRS_SERVO2_CLOSED + DRS_SERVO2_OPEN_OVERDRIVE - DRS_SERVO2_CLOSED_OVERDRIVE;
        drs_status.vc_drs_overdrive_count--;
    }
    if (drs_status.vc_drs_ramp_count < 0) drs_status.vc_drs_ramp_count = 0;
    if (drs_status.vc_drs_ramp_count > DRS_RAMP_TIME) drs_status.vc_drs_ramp_count = DRS_RAMP_TIME;
    int servo1 = servo1_offset + (servo1_range*drs_status.vc_drs_ramp_count)/DRS_RAMP_TIME;
    int servo2 = servo2_offset + (servo2_range*drs_status.vc_drs_ramp_count)/DRS_RAMP_TIME;
    DRS_set(servo1, servo2);
    drs_status.vc_drs1_setting = servo1;
    drs_status.vc_drs2_setting = servo2;
    sensor_dbc_vc_drs_status_pack((uint8_t *)&msg, &drs_status, 8);
    core_CAN_add_message_to_tx_queue(CAN_SENSE, SENSOR_DBC_VC_DRS_STATUS_FRAME_ID, 8, msg);
}
