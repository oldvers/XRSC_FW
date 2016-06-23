#include <string.h>
#include "uart.h"

/*----------------------------------------------------------------------------*/

__weak void ESP8266_UART_RxReadyCallback(void)
{
  return;
}

/*----------------------------------------------------------------------------*/

struct UartFifo_s
{
  S16  I;
  S16  O;
  U16  C;
  U8   B[256];
} UartFifo = { 0, 0, 256, {0} };

/*----------------------------------------------------------------------------*/

U8 UartFifo_Put( U8 aByte )
{
  if(UartFifo.I == ((UartFifo.O - 1 + UartFifo.C) % UartFifo.C))
  {
    return 0; //Queue Full
  }

  UartFifo.B[UartFifo.I] = aByte;
  UartFifo.I = (UartFifo.I + 1) % UartFifo.C;
    
  return 1; //No Errors
}

/*----------------------------------------------------------------------------*/

U8 UartFifo_Get( U8 * pByte )
{
  if(UartFifo.I == UartFifo.O)
  {
    return 0; //Queue Empty - Nothing to get
  }

  *pByte = UartFifo.B[UartFifo.O];
  UartFifo.O = (UartFifo.O + 1) % UartFifo.C;

  return 1; //No Errors
}
/*----------------------------------------------------------------------------*/

void ESP8266_UART_Init( void )
{
  /* ESP8266 UART Tx = PA2 - AFO, Push-Pull */
  GPIO_Init( ESP8266_UART_TX_PORT, ESP8266_UART_TX_PIN, GPIO_TYPE_ALT_PP_10MHZ );

  /* ESP8266 UART Rx = PA3 - AFO, Push-Pull */
  GPIO_Init( ESP8266_UART_RX_PORT, ESP8266_UART_RX_PIN, GPIO_TYPE_IN_PUP_PDN );
  GPIO_Hi( ESP8266_UART_RX_PORT, ESP8266_UART_RX_PIN );

  /* ESP8266 UART = USART2 */
  RCC->ESP8266_UART_CLK_R |= ESP8266_UART_CLK_EN;
  
  ESP8266_UART->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  ESP8266_UART->CR2 = 0;
  ESP8266_UART->CR3 = 0;
  //Fck = 24 MHz
  //Baud = 115200
  //BRR = Fck / Baud = 24000000/115200/16 = 13.021 -> 0x00D0
  ESP8266_UART->BRR = 0x00D0;
  ESP8266_UART->CR1 |= USART_CR1_UE;

  NVIC_SetPriority(ESP8266_UART_IRQN, 0);
  NVIC_EnableIRQ(ESP8266_UART_IRQN);
}

/*----------------------------------------------------------------------------*/

void ESP8266_UART_PutByte( U8 aByte )
{
  if( 0 == (ESP8266_UART->CR1 & USART_CR1_UE) )
  {
    return;
  }

  while( (ESP8266_UART->SR & USART_SR_TXE) == 0 ) {};
  ESP8266_UART->DR = aByte;
}

/*----------------------------------------------------------------------------*/

U8 ESP8266_UART_GetByte( U8 * pByte )
{
  return UartFifo_Get( pByte );
}

/*----------------------------------------------------------------------------*/

void ESP8266_UART_IRQHandler(void)
{
  if ( 0 != (ESP8266_UART->SR & USART_SR_RXNE) )
  {
    U8 Byte = ESP8266_UART->DR;

    UartFifo_Put( Byte );
    ESP8266_UART_RxReadyCallback();
  }
}
