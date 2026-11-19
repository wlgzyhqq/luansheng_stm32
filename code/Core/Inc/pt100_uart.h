#ifndef PT100_UART_H
#define PT100_UART_H

#include "stm32f1xx_hal.h"

/* UART4: PC10 TX, PC11 RX, 9600 baud, 8N1. */
#define PT100_BAUD_RATE 9600U
#define PT100_QUERY_PERIOD_MS 1000U

void PT100_Init(void);
void PT100_SetEnabled(uint8_t enabled);
void PT100_Process(void);

extern volatile float pt100_temperature_c;
/* Simulated value; validity follows pt100_temperature_valid. */
extern volatile float pt100_simulated_temperature_c;
extern volatile uint8_t pt100_temperature_valid;
extern volatile uint8_t pt100_read_enabled;
extern volatile uint32_t pt100_tx_count;
extern volatile uint32_t pt100_tx_errors;
extern volatile uint32_t pt100_rx_bytes;
extern volatile uint32_t pt100_rx_errors;

#endif
