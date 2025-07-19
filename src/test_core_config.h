#ifndef TEST_CORE_CONFIG_H
#define TEST_CORE_CONFIG_H


#define CORE_CLOCK_USE_HSE 1
/**
  * @brief  Desired system clock frequency in kHz
  */
#define CORE_CLOCK_SYSCLK_FREQ 110000
/**
  * @brief  Frequency of the internal oscillator in kHz
  */
#define CORE_CLOCK_HSI_FREQ 16000
/**
  * @brief  Divider for the P output on the PLL
  */
#define CORE_CLOCK_PLLP_DIV 12
#define CORE_CLOCK_HSE_FREQ 25000

/*** ERROR HANDLER PARAMETERS ***/
#define ERROR_HANDLER_BLINK_DELAY 200000


/*** CAN CONFIG PARAMETERS ***/

// CAN bitrate in Hz
#define CORE_CAN_BITRATE 1000000

// Number of CAN messages that can be stored in the CAN FreeRTOS queue
#define CORE_CAN_QUEUE_LENGTH 10

// Timeout for waiting on RX queue
#define CORE_CAN_RX_TIMEOUT 50

#define SOMETHING_NEW ( 101/303 )

#endif
