#include "leds.h"

void LEDs_Init(void)
{
  //* LED STAT1 -> GPO, Open Drain *******************************************************************************
  RCC->APB2ENR |= LED_STAT1_GPIO_CLK;
#if (LED_STAT1_GPIO_PIN < 8)
  LED_STAT1_GPIO->CRL &= ~(GPIO_TYPE_MASK << (LED_STAT1_GPIO_PIN * 4));
  LED_STAT1_GPIO->CRL |= (GPIO_TYPE_OUT_PP_50MHZ << (LED_STAT1_GPIO_PIN * 4));
#else
  LED_STAT1_GPIO->CRH &= ~(GPIO_TYPE_MASK << ((LED_STAT1_GPIO_PIN - 8) * 4));
  LED_STAT1_GPIO->CRH |= (GPIO_TYPE_OUT_OD_2MHZ << ((LED_STAT1_GPIO_PIN - 8 ) * 4));
#endif
  
  //* LED STAT2 -> GPO, Open Drain *******************************************************************************
  RCC->APB2ENR |= LED_STAT2_GPIO_CLK;
#if (LED_STAT2_GPIO_PIN < 8)
  LED_STAT2_GPIO->CRL &= ~(GPIO_TYPE_MASK << (LED_STAT2_GPIO_PIN * 4));
  LED_STAT2_GPIO->CRL |= (GPIO_TYPE_OUT_PP_50MHZ << (LED_STAT2_GPIO_PIN * 4));
#else
  LED_STAT2_GPIO->CRH &= ~(GPIO_TYPE_MASK << ((LED_STAT2_GPIO_PIN - 8) * 4));
  LED_STAT2_GPIO->CRH |= (GPIO_TYPE_OUT_OD_2MHZ << ((LED_STAT2_GPIO_PIN - 8 ) * 4));
#endif
  
  //* LED STAT3 -> GPO, Push-Pull *******************************************************************************
  RCC->APB2ENR |= LED_STAT3_GPIO_CLK;
#if (LED_STAT3_GPIO_PIN < 8)
  LED_STAT3_GPIO->CRL &= ~(GPIO_TYPE_MASK << (LED_STAT3_GPIO_PIN * 4));
  LED_STAT3_GPIO->CRL |= (GPIO_TYPE_OUT_PP_50MHZ << (LED_STAT3_GPIO_PIN * 4));
#else
  LED_STAT3_GPIO->CRH &= ~(GPIO_TYPE_MASK << ((LED_STAT3_GPIO_PIN - 8) * 4));
  LED_STAT3_GPIO->CRH |= (GPIO_TYPE_OUT_PP_50MHZ << ((LED_STAT3_GPIO_PIN - 8 ) * 4));
#endif
}
