#ifndef __H_UART_H__
#define __H_UART_H__

#include "types.h"
#include "stm32f10x.h"
#include "gpio.h"

/* ESP8266 UART Tx = PA2 - AFO, Push-Pull */
#define ESP8266_UART_TX_PORT                GPIOA
#define ESP8266_UART_TX_PIN                 2

/* ESP8266 UART Rx = PA3 - AFO, Push-Pull */
#define ESP8266_UART_RX_PORT                GPIOA
#define ESP8266_UART_RX_PIN                 3

/* ESP8266 UART = USART2 */
#define ESP8266_UART_CLK_R                  APB1ENR
#define ESP8266_UART_CLK_EN                 RCC_APB1ENR_USART2EN
#define ESP8266_UART                        USART2
#define ESP8266_UART_IRQN                   USART2_IRQn

void ESP8266_UART_Init( void );
void ESP8266_UART_PutByte( U8 aByte );
U8   ESP8266_UART_GetByte( U8 * pByte );
void ESP8266_UART_IRQHandler(void);

#endif //__H_UART_H__
