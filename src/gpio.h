#ifndef __GPIO_H__
#define __GPIO_H__

#include "types.h"
#include "stm32f10x.h"

/* PIN MODE ****************************************/
/* 0b00 = 0 - Input mode (reset state).            */
/* 0b01 = 1 - Output mode, max speed 10 MHz.       */
/* 0b10 = 2 - Output mode, max speed 2 MHz.        */
/* 0b11 = 3 - Output mode, max speed 50 MHz.       */

/* PIN CNF *****************************************/
/* In input mode (MODE[1:0] = 00) ---------------- */
/* 0b00 = 0 - Analog mode                          */
/* 0b01 = 1 - Floating input (reset state)         */
/* 0b10 = 2 - Input with Pull-Down / Pull-Up       */
/* 0b11 = 3 - Reserved                             */
/* In output mode (MODE[1:0] > 00) --------------- */
/* 0b00 = 0 - General purpose output Push-Pull     */
/* 0b01 = 1 - General purpose output Open-Drain    */
/* 0b10 = 2 - Alternate function output Push-Pull  */
/* 0b11 = 3 - Alternate function output Open-Drain */

#define GPIO_TYPE_MASK                     ((U32)0x0F)

#define GPIO_TYPE_IN_ANALOG                ((U32)0x00)
#define GPIO_TYPE_IN_FLOATING              ((U32)0x04)
#define GPIO_TYPE_IN_PUP_PDN               ((U32)0x08)

#define GPIO_TYPE_OUT_PP_10MHZ             ((U32)0x01)
#define GPIO_TYPE_OUT_PP_2MHZ              ((U32)0x02)
#define GPIO_TYPE_OUT_PP_50MHZ             ((U32)0x03)
#define GPIO_TYPE_OUT_OD_10MHZ             ((U32)0x05)
#define GPIO_TYPE_OUT_OD_2MHZ              ((U32)0x06)
#define GPIO_TYPE_OUT_OD_30MHZ             ((U32)0x07)
#define GPIO_TYPE_ALT_PP_10MHZ             ((U32)0x09)
#define GPIO_TYPE_ALT_PP_2MHZ              ((U32)0x0A)
#define GPIO_TYPE_ALT_PP_50MHZ             ((U32)0x0B)
#define GPIO_TYPE_ALT_OD_10MHZ             ((U32)0x0D)
#define GPIO_TYPE_ALT_OD_2MHZ              ((U32)0x0E)
#define GPIO_TYPE_ALT_OD_50MHZ             ((U32)0x0F)

//Modified structure
typedef struct
{
  __IO uint32_t CR[2];
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDefEx;


#define GPIO_Init(port,pin,mode) \
  RCC->APB2ENR |=  (U32)((1 << (((U32)port >> 10) & 0x0F)) | (RCC_APB2ENR_AFIOEN * (U8)(mode > 8))); \
  ((GPIO_TypeDefEx *)port)->CR[pin / 8] &= ~(GPIO_TYPE_MASK << ((pin % 8) * 4)); \
	((GPIO_TypeDefEx *)port)->CR[pin / 8] |= (mode << ((pin % 8) * 4));

#define GPIO_Hi(port,pin) \
    (port->BSRR = (1 << pin))

#define GPIO_Lo(port,pin) \
    (port->BSRR = (1 << (pin + 16)))

#endif /* __GPIO_H__ */
