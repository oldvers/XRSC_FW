#include <rtl.h>
#include "stm32f10x.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "main_task_wifi.h"
#include "main_task_roadsign.h"


__task void MainTask()
{
  U8  ButtonPressed = 0;
  U8  FanOn         = 0;
  U8  FanEn         = 0;
  U32 FanDelay      = 0;
  S16 Temperature   = 0;
  
  //Initialize threads
  RoadSign_ThreadInit();
  WiFi_ThreadInit();
  
  //Initialize button
  GPIO_Init( BUTTON_PORT, BUTTON_PIN, GPIO_TYPE_IN_PUP_PDN );
  GPIO_Hi( BUTTON_PORT, BUTTON_PIN );
  
  //Initialize fan control
  GPIO_Init( FAN_PORT, FAN_PIN, GPIO_TYPE_OUT_PP_2MHZ );
  GPIO_Lo( FAN_PORT, FAN_PIN );
  
  //Initialize ADC
  ADC_Init();
  
  for(;;)
  {
    //Check if button is pressed
    if ( 0 == GPIO_In( BUTTON_PORT, BUTTON_PIN ) )
    {
      os_dly_wait(10);
      
      if ( 0 == GPIO_In( BUTTON_PORT, BUTTON_PIN ) )
      {
        ButtonPressed++;
      }
      else
      {
        ButtonPressed = 0;
      }
    }
    else
    {
      ButtonPressed = 0;
    }
    
    //Change the image
    if ( 1 == ButtonPressed )
    {
      RoadSign_SetImage(NULL, 0);
    }
    
    //Change the brightness
    if ( 5 == ButtonPressed )
    {
      ButtonPressed = 2;
      RoadSign_ChangeBrightness();
    }
    
    
    Temperature = ADC_ReadTemperature();
    
    if ( Temperature > 20 )
    {
      FanEn = 1;
    }
    else if ( Temperature < 15 )
    {
      FanEn = 0;
    }
    
    if ( 1 == FanEn )
    {
      FanDelay++;
    
      if ( 1500 < FanDelay )
      {
        FanDelay = 0;
        FanOn ^= 1;
      }
      
      if ( 1 == FanOn )
      {
        GPIO_Hi( FAN_PORT, FAN_PIN );
      }
      else
      {
        GPIO_Lo( FAN_PORT, FAN_PIN );
      }
    }
    else
    {
      FanOn = 0;
      FanDelay = 0;
      GPIO_Lo( FAN_PORT, FAN_PIN );
    }

    os_dly_wait(200);
  }
}



int main()
{
  DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;
  
  //os_sys_init(MainTask);
  os_sys_init_prio(MainTask, 10);
  
  while(1)
  {
    //
  }
}
