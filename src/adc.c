#include "stm32f10x.h"
#include "types.h"

/*----------------------------------------------------------------------------*/

void ADC_Init( void )
{
  volatile U32 delay;
  
  //Enable ADC clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  //Setup ADC prescaller to 4 (48 / 4 -> 12 MHz)
  RCC->CFGR |= RCC_CFGR_ADCPRE_0;
  
  ADC1->CR1 = 0;
  //Enable temperature sensor and Vrefint
  ADC1->CR2 = ADC_CR2_TSVREFE | ADC_CR2_ADON;
  
  for ( delay = 0; delay < 500; delay++ ) {};
  
  //Calibrate the ADC
  ADC1->CR2 |= ADC_CR2_CAL;
  delay = 1000;
  while ( ((ADC1->CR2 & ADC_CR2_CAL) != 0) && (0 < delay)) delay--;

  if ( 0 == delay )
  {
    ADC1->CR2 = 0;
    return;
  }

  ADC1->SMPR1 = 0x00FFFFFF;
  ADC1->SMPR2 = 0x03FFFFFF;
  
  //Configure 1 regular conversion
  ADC1->SQR1 = (0 << 20);
  ADC1->SQR2 = 0;
  //Select the chhannel for conversion (16 - temperature)
  ADC1->SQR3 = (16 << 0);
}

/*----------------------------------------------------------------------------*/

S16 ADC_ReadTemperature(void)
{
  volatile U16 delay       = 1300;
  static U32   Vaverage    = 0;
  static U8    Vcount      = 0;
  U16          Vsense;
  static S16   Temperature = 0;
  
  if ( (ADC1->CR2 & ADC_CR2_ADON) == 0 ) return Temperature;
  
  //Select the chhannel for conversion (16 - temperature)
  ADC1->SQR3 = (16 << 0);
  //Clear the flags
  ADC1->SR &= ~(ADC_SR_EOC);
  //Start conversion
  ADC1->CR2 |= ADC_CR2_ADON;
  //Wait for complete
  while ( ((ADC1->SR & ADC_SR_EOC) == 0) && (0 < delay) ) delay--;
  //Check for timeout
  if ( 0 == delay ) return Temperature;
  //Read the adc value
  Vsense = ADC1->DR;
  
  Vaverage += Vsense;
  Vcount++;
  
  if ( 32 == Vcount )
  {
    Vsense = Vaverage >> 5;
    
    //Calculate the temperatue
    //Avg_Slope = 4.3 mV/C
    //V25 = 1.43 V
    Vaverage = Vsense * 33000;
    Vsense = Vaverage >> 12;
    Temperature = ((14300 - Vsense) / 43) + 25;
    
    Vaverage = 0;
    Vcount = 0;
  }
  
  return Temperature;
}

/*----------------------------------------------------------------------------*/
