#include "stm32f10x.h"
#include "core_cm3.h"
#include "rsProcessing.h"
#include "rsUnpacking.h"
#include "rsImageRefreshing.h"
#include "gpio.h"

#define Lat_Drv_Hi()    GPIO_Hi( IR_LATCH_DRV_PORT, IR_LATCH_DRV_PIN )
#define Lat_Drv_Lo()    GPIO_Lo( IR_LATCH_DRV_PORT, IR_LATCH_DRV_PIN )

#define Sck_Drv_Hi()    GPIO_Hi( IR_SCK_DRV_PORT, IR_SCK_DRV_PIN )
#define Sck_Drv_Lo()    GPIO_Lo( IR_SCK_DRV_PORT, IR_SCK_DRV_PIN )

#define Lat_Buf_Hi()    GPIO_Hi( IR_LATCH_BUFF_PORT, IR_LATCH_BUFF_PIN )
#define Lat_Buf_Lo()    GPIO_Lo( IR_LATCH_BUFF_PORT, IR_LATCH_BUFF_PIN )

tImageRefreshing IR;

unsigned int Val = ~0x00700700, xxx = 0;

/******************************************************************************/

void ImageRefreshing_Init(pRS aRS)
{
  IR.InProgress = 1;
  IR.StartAddress = &aRS->Chain[0];
  IR.EndAddress = (U8 *)((U32)&aRS->Chain + aRS->ChainSize);
  IR.PartSize = BUFF_COUNT;
  IR.CurrentAddress = IR.StartAddress;
  IR.SubChainLength = aRS->SubChainLen;
  IR.SubChainCounter = 0;

  /* Latch_Drv -> GPO, Push-Pull */
  GPIO_Init( IR_LATCH_DRV_PORT, IR_LATCH_DRV_PIN, GPIO_TYPE_OUT_PP_50MHZ );

  /* SCK_Drv -> GPO, Push-Pull */
  GPIO_Init( IR_SCK_DRV_PORT, IR_SCK_DRV_PIN, GPIO_TYPE_OUT_PP_50MHZ );

  /* PWM_Drv -> AFO, Push-Pull */
  GPIO_Init( IR_PWM_DRV_PORT, IR_PWM_DRV_PIN, GPIO_TYPE_ALT_PP_50MHZ );

  /* Latch_Buff -> GPO, Push-Pull */
  GPIO_Init( IR_LATCH_BUFF_PORT, IR_LATCH_BUFF_PIN, GPIO_TYPE_OUT_PP_50MHZ );
  
  /* SCK_Buff -> AFO, Push-Pull */
  GPIO_Init( IR_SCK_BUFF_PORT, IR_SCK_BUFF_PIN, GPIO_TYPE_ALT_PP_50MHZ );

  /* D_Buff -> AFO, Push-Pull */
  GPIO_Init( IR_D_BUFF_PORT, IR_D_BUFF_PIN, GPIO_TYPE_ALT_PP_50MHZ );

  /* SPI */
  if(IR_SPI_CLK == RCC_APB2ENR_SPI1EN)
  {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    IR_SPI->CR1 |=
  //  SPI_CR1_BIDIMODE |   //Bidirectional Mode
  //  SPI_CR1_BIDIOE |     //Bidirectional Mode Output Enable
  //  SPI_CR1_CRCEN |      //
  //  SPI_CR1_CRCNEXT |    //
  //  SPI_CR1_DFF |        //Data Frame Format
  //  SPI_CR1_RXONLY |     //Rx Only Mode
      SPI_CR1_SSM  |       //Software Slave control
      SPI_CR1_SSI  |       //Internal Slave select
  //  SPI_CR1_LSBFIRST |   //LSB First
      SPI_CR1_BR_0 |       //Prescaller
  //  SPI_CR1_BR_1 |
  //  SPI_CR1_BR_2 |
      SPI_CR1_MSTR |       //Master mode
  //  SPI_CR1_CPOL |
  //  SPI_CR1_CPHA |
      SPI_CR1_SPE;         //SPI Enable
  }
  else
  {
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    IR_SPI->CR1 |=
  //  SPI_CR1_BIDIMODE |   //Bidirectional Mode
  //  SPI_CR1_BIDIOE |     //Bidirectional Mode Output Enable
  //  SPI_CR1_CRCEN |      //
  //  SPI_CR1_CRCNEXT |    //
  //  SPI_CR1_DFF |        //Data Frame Format
  //  SPI_CR1_RXONLY |     //Rx Only Mode
      SPI_CR1_SSM  |       //Software Slave control
      SPI_CR1_SSI  |       //Internal Slave select
  //  SPI_CR1_LSBFIRST |   //LSB First
      SPI_CR1_BR_0 |       //Prescaller
  //  SPI_CR1_BR_1 |
  //  SPI_CR1_BR_2 |
      SPI_CR1_MSTR |       //Master mode
  //  SPI_CR1_CPOL |
  //  SPI_CR1_CPHA |
      SPI_CR1_SPE;         //SPI Enable
  }

  IR_SPI->CR2 |=
  //  SPI_CR2_TXEIE |      //Tx Empty Interrupt Enable
  //  SPI_CR2_RXNEIE |     //Rx Not Empty Interrupt Enable
  //  SPI_CR2_ERRIE |      //Error Interrupt Enable
  //  SPI_CR2_SSOE |       //SS Output Enable
      SPI_CR2_TXDMAEN;     //Enable DMA Tx
  //  SPI_CR2_RXDMAEN;     //Enable DMA Rx

  //* NVIC *******************************************************************************************************

  NVIC_SetPriorityGrouping(6);

  //Enable SPI DMA IRQn Interrupt
  //NVICI.NVIC_IRQChannel = IR_SPI_DMA_TX_IRQN;
  //NVICI.NVIC_IRQChannelPreemptionPriority = 0;
  //NVICI.NVIC_IRQChannelSubPriority = 0;
  //NVICI.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVICI);
  
  NVIC_SetPriority(IR_SPI_DMA_TX_IRQN, 0);
  NVIC_ClearPendingIRQ(IR_SPI_DMA_TX_IRQN);
  NVIC_EnableIRQ(IR_SPI_DMA_TX_IRQN);
  
  //NVICI.NVIC_IRQChannel = IR_TIM_IRQN;
  //NVICI.NVIC_IRQChannelPreemptionPriority = 0;
  //NVICI.NVIC_IRQChannelSubPriority = 0;
  //NVICI.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVICI);
  
  NVIC_SetPriority(IR_TIM_IRQN, 0);
  NVIC_ClearPendingIRQ(IR_TIM_IRQN);
  NVIC_EnableIRQ(IR_TIM_IRQN);

  //* DMA ********************************************************************************************************

  RCC->AHBENR |= IR_SPI_DMA_CLK;

  IR_SPI_DMA->IFCR |=
    (IR_SPI_DMA_TX_FLAG_GL | IR_SPI_DMA_TX_FLAG_TC | IR_SPI_DMA_TX_FLAG_HT | IR_SPI_DMA_TX_FLAG_TC);
  IR_SPI_DMA_TX_CHANNEL->CPAR  = (U32)&IR_SPI->DR;
  IR_SPI_DMA_TX_CHANNEL->CMAR  = (U32)IR.CurrentAddress;
  IR_SPI_DMA_TX_CHANNEL->CNDTR = IR.PartSize;
  IR_SPI_DMA_TX_CHANNEL->CCR  |= IR_SPI_DMA_TX_MEM_INC | IR_SPI_DMA_TX_DIR | IR_SPI_DMA_TX_TCIE;
  IR_SPI_DMA_TX_CHANNEL->CCR  |= IR_SPI_DMA_TX_EN;

  //* TIM ********************************************************************************************************

  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
  AFIO->MAPR |= IR_TIM_REMAP;

  RCC->IR_TIM_CLK_R |= IR_TIM_CLK_EN;
  IR_TIM->PSC = 8 - 1;
  IR_TIM->ARR = 256 - 1;
  //Setup TIM Compare Channel 1 (Output, PWM Mode 1, Inverted, 10.67 us)
  IR_TIM->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P);
  IR_TIM->CCMR1 &= ~(TIM_CCMR1_OC1CE | TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE | TIM_CCMR1_CC1S);
  IR_TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
  IR_TIM->CCR1 = 2;
  IR_TIM->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;
  //Setup TIM Compare Channel 2 (Output, Active Mode, 7 us)
  IR_TIM->CCMR1 &= ~(TIM_CCMR1_OC2CE | TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE | TIM_CCMR1_CC2S);
  IR_TIM->CCMR1 |= TIM_CCMR1_OC2M_0;
  IR_TIM->CCR2 = 168;
  //Setup TIM Compare Channel 3 (Output, Active Mode, 8.5 us)
  IR_TIM->CCMR2 &= ~(TIM_CCMR2_OC3CE | TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE | TIM_CCMR2_CC3S);
  IR_TIM->CCMR2 |= TIM_CCMR2_OC3M_0;
  IR_TIM->CCR3 = 204;
  //Setup Interrupts
  IR_TIM->DIER |= TIM_DIER_UIE | TIM_DIER_CC2IE | TIM_DIER_CC3IE;
  //Enable TIM
  IR_TIM->CR1 |= TIM_CR1_CEN;
}

/******************************************************************************/

void ImageRefreshing_Update(pRS aRS)
{
  IR.InProgress = 0;
  IR.StartAddress = &aRS->Chain[0];
  IR.EndAddress = (U8 *)((U32)&aRS->Chain + aRS->ChainSize);
  IR.PartSize = BUFF_COUNT;
  IR.CurrentAddress = IR.StartAddress;
  IR.SubChainLength = aRS->SubChainLen;
  IR.SubChainCounter = 0;
  IR.InProgress = 1;
}

/******************************************************************************/

void IR_SPI_DMA_IRQ_HANDLER()
{
  IR_SPI_DMA_TX_CHANNEL->CCR &= ~IR_SPI_DMA_TX_EN;
  IR_SPI_DMA->IFCR |=
    (IR_SPI_DMA_TX_FLAG_GL | IR_SPI_DMA_TX_FLAG_TC | IR_SPI_DMA_TX_FLAG_HT | IR_SPI_DMA_TX_FLAG_TC);

  IR.CurrentAddress += IR.PartSize;
  //if(IR.CurrentAddress >= IR.EndAddress) IR.CurrentAddress = IR.StartAddress;
  //LED_STAT1_On();
}

/******************************************************************************/

void IR_TIM_IRQ_HANDLER()
{
  unsigned int State = IR_TIM->SR;
  IR_TIM->SR = 0;

  if(IR.InProgress == 0) return;

  if((State & TIM_SR_UIF) != 0)
  {
    Sck_Drv_Lo();
    Lat_Drv_Lo();
    IR_TIM->CCMR1 |= (TIM_CCMR1_OC1M_1);

    if(IR.SubChainCounter != IR.SubChainLength)
    {
      IR_SPI_DMA_TX_CHANNEL->CMAR = (U32)IR.CurrentAddress;
      IR_SPI_DMA_TX_CHANNEL->CNDTR = IR.PartSize;
      IR_SPI_DMA_TX_CHANNEL->CCR |= IR_SPI_DMA_TX_EN;
      //IR.CurrentAddress += IR.PartSize;
    }
  }

  if((State & TIM_SR_CC2IF) != 0)
  {
    if(IR.SubChainCounter == IR.SubChainLength)
    {
      Lat_Drv_Hi();
    }
    else
    {
      Lat_Buf_Hi();
    }
  }

  if((State & TIM_SR_CC3IF) != 0)
  {
    if(IR.SubChainCounter == IR.SubChainLength)
    {
      IR_TIM->CCMR1 &= ~(TIM_CCMR1_OC1M_1);
      IR.SubChainCounter = 0;
      if(IR.CurrentAddress >= IR.EndAddress) IR.CurrentAddress = IR.StartAddress;
    }
    else
    {
      Lat_Buf_Lo();
      Sck_Drv_Hi();
      IR.SubChainCounter++;
    }
  }
}

/******************************************************************************/

void ImageRefreshing_SetBrightness(U8 aBrightness)
{
  U16 B = aBrightness;
  
  B <<= 8;
  B /= 100;
  
  IR_TIM->CCR1 = B;
}

void ImageRefreshing_Start(void)
{
  IR.CurrentAddress = IR.StartAddress;
  IR_TIM->CCR1 = 2;
  IR.InProgress = 1;
}

U8 ImageRefreshing_IsOn(void)
{
  return IR.InProgress = 1;
}

/******************************************************************************/

U8 ImageRefreshing_End(void)
{
  IR.InProgress = 0;
  IR_TIM->CCR1 = 0;
  IR.SubChainCounter = 0;
  return (U8)(IR.InProgress == 0);
}

/******************************************************************************/

void ImageRefreshing_OneShift(void)
{
  U8 wait, reg;

  if(IR.InProgress == 0) return;

  for(reg = 0; reg < 9; reg++)
  {
    IR_SPI->DR = ((U8 *)IR.CurrentAddress)[reg];
    while(!(IR_SPI->SR & SPI_SR_TXE));
    IR.CurrentAddress++;
  }

  Lat_Buf_Hi();
  for(wait = 0; wait < 100; wait++);
  Lat_Buf_Lo();
  for(wait = 0; wait < 100; wait++);
  Sck_Drv_Hi();
  for(wait = 0; wait < 100; wait++);
  Sck_Drv_Lo();
  for(wait = 0; wait < 100; wait++);

  IR.SubChainCounter++;
  if(IR.SubChainCounter == (IR.SubChainLength - 1))
  {
    Lat_Drv_Hi();
    for(wait = 0; wait < 100; wait++);
    Lat_Drv_Lo();
    for(wait = 0; wait < 100; wait++);
  }

  if(IR.CurrentAddress >= IR.EndAddress - 1)
  {
    IR.CurrentAddress = IR.StartAddress;
    IR.InProgress = 0;
  }
}

/******************************************************************************/
