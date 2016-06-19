/*  */

#include "stm32f10x.h"
#include "gpio.h"

/* PB1 - STAT1 */
#define LED_STAT1_GPIO_CLK        RCC_APB2ENR_IOPBEN
#define LED_STAT1_GPIO            GPIOB
#define LED_STAT1_GPIO_PIN        1

/* PB0 - STAT2 */
#define LED_STAT2_GPIO_CLK        RCC_APB2ENR_IOPBEN
#define LED_STAT2_GPIO            GPIOB
#define LED_STAT2_GPIO_PIN        0

/* PA1 - STAT3 */
#define LED_STAT3_GPIO_CLK        RCC_APB2ENR_IOPAEN
#define LED_STAT3_GPIO            GPIOA
#define LED_STAT3_GPIO_PIN        1


#define LED_STAT1_Off()           (LED_STAT1_GPIO->BSRR = (1 << LED_STAT1_GPIO_PIN))
#define LED_STAT1_On()            (LED_STAT1_GPIO->BSRR = (1 << (LED_STAT1_GPIO_PIN + 16)))

#define LED_STAT2_Off()           (LED_STAT2_GPIO->BSRR = (1 << LED_STAT2_GPIO_PIN))
#define LED_STAT2_On()            (LED_STAT2_GPIO->BSRR = (1 << (LED_STAT2_GPIO_PIN + 16)))

#define LED_STAT3_On()            (LED_STAT3_GPIO->BSRR = (1 << LED_STAT3_GPIO_PIN))
#define LED_STAT3_Off()           (LED_STAT3_GPIO->BSRR = (1 << (LED_STAT3_GPIO_PIN + 16)))


void LEDs_Init(void);
