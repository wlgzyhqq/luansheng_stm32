#include "pt100_uart.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>

static volatile uint8_t pt100_rx_ring[128];
static volatile uint8_t pt100_rx_head = 0;
static volatile uint8_t pt100_rx_tail = 0;
volatile uint32_t pt100_rx_bytes = 0;
volatile uint32_t pt100_rx_errors = 0;

static int PT100_ReadByte(uint8_t *byte)
{
  if (pt100_rx_tail == pt100_rx_head) return 0;
  *byte = pt100_rx_ring[pt100_rx_tail];
  pt100_rx_tail = (pt100_rx_tail + 1) & 127;
  return 1;
}

void UART4_IRQHandler(void)
{
  uint32_t status = UART4->SR;
  if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
  {
    uint8_t byte = (uint8_t)UART4->DR;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
      pt100_rx_errors++;
    if (status & USART_SR_RXNE)
    {
      uint8_t next = (pt100_rx_head + 1) & 127;
      pt100_rx_bytes++;
      if (next != pt100_rx_tail)
      {
        pt100_rx_ring[pt100_rx_head] = byte;
        pt100_rx_head = next;
      }
      else pt100_rx_errors++;
    }
  }
}

volatile uint32_t pt100_tx_count = 0;
volatile uint32_t pt100_tx_errors = 0;
volatile float pt100_temperature_c = 0.0f;
/* Simulated second temperature, not a second sensor measurement. */
volatile float pt100_simulated_temperature_c = 0.0f;
static uint32_t pt100_simulation_state = 1U;
volatile uint8_t pt100_temperature_valid = 0;
volatile uint8_t pt100_read_enabled = 0;
static uint32_t pt100_tick = 0;

void PT100_Process(void)
{
  static uint8_t query[] = {0x41, 0x54, 0x2B, 0x54, 0x0D, 0x0A};
  static char reply[48];
  static uint8_t length = 0;
  static uint8_t waiting = 0;
  uint8_t count = 0;
  uint8_t byte;
  uint32_t now = HAL_GetTick();
  if (!pt100_read_enabled) { waiting = 0; length = 0; return; }
  while (count++ < sizeof(reply) && PT100_ReadByte(&byte))
  {
    if (byte == 10)
    {
      char *value;
      reply[length] = 0;
      value = strstr(reply, "+T=");
      if (value != NULL)
      {
        char *end;
        float temperature = (float)strtod(value + 3, &end);
        if (end != value + 3 && temperature >= -200 && temperature <= 650)
        {
          pt100_temperature_c = temperature;
          pt100_simulation_state = pt100_simulation_state * 1664525U + 1013904223U;
          pt100_simulated_temperature_c = temperature
            + ((int32_t)((pt100_simulation_state >> 16) % 201U) - 100) / 100.0f;
          pt100_temperature_valid = 1;
          waiting = 0;
        }
      }
      length = 0;
    }
    else if (byte != 13)
    {
      if (length < sizeof(reply) - 1) reply[length++] = (char)byte;
      else length = 0;
    }
  }
  if (now - pt100_tick >= PT100_QUERY_PERIOD_MS)
  {
    if (waiting) pt100_temperature_valid = 0;
    length = 0;
    pt100_tick = now;
    waiting = (HAL_UART_Transmit(&huart4, query, sizeof(query), 50) == HAL_OK);
    if (waiting) pt100_tx_count++;
    else { pt100_tx_errors++; pt100_temperature_valid = 0; }
  }
}

void PT100_Init(void)
{
  HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
  PT100_SetEnabled(1);
}

void PT100_SetEnabled(uint8_t enabled)
{
  pt100_read_enabled = enabled;
  pt100_temperature_valid = 0;
  pt100_tick = HAL_GetTick() - PT100_QUERY_PERIOD_MS;
}
