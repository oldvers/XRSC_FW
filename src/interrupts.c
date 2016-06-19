#include "uart.h"

void USART2_IRQHandler(void)
{
  ESP8266_UART_IRQHandler();
}

void HardFault_Handler(void)
{
  NVIC_SystemReset();
}
