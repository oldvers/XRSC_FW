#ifndef __RS_IMAGE_REFRESHING_H
#define __RS_IMAGE_REFRESHING_H

#include "gpio.h"

#define BUFF_COUNT                 2

#define IR_LATCH_DRV_PORT          GPIOC
#define IR_LATCH_DRV_PIN           7

#define IR_SCK_DRV_PORT            GPIOB
#define IR_SCK_DRV_PIN             8

#define IR_PWM_DRV_PORT            GPIOC
#define IR_PWM_DRV_PIN             6

#define IR_LATCH_BUFF_PORT         GPIOB
#define IR_LATCH_BUFF_PIN          12

#define IR_SCK_BUFF_PORT           GPIOB
#define IR_SCK_BUFF_PIN            13

#define IR_D_BUFF_PORT             GPIOB
#define IR_D_BUFF_PIN              15

#define IR_D_IN_PORT               GPIOB
#define IR_D_IN_PIN                14

#define IR_LATCH_IN_PORT           GPIOC
#define IR_LATCH_IN_PIN            8

#define IR_SPI_CLK                 RCC_APB1ENR_SPI2EN
#define IR_SPI                     SPI2

#define IR_SPI_DMA                 DMA1
#define IR_SPI_DMA_CLK             RCC_AHBENR_DMA1EN

#define IR_SPI_DMA_RX_CHANNEL      DMA1_Channel4

#define IR_SPI_DMA_TX_CHANNEL      DMA1_Channel5
#define IR_SPI_DMA_TX_FLAG_TC      DMA_ISR_TCIF5
#define IR_SPI_DMA_TX_FLAG_TE      DMA_ISR_TEIF5
#define IR_SPI_DMA_TX_FLAG_HT      DMA_ISR_HTIF5
#define IR_SPI_DMA_TX_FLAG_GL      DMA_ISR_GIF5
#define IR_SPI_DMA_TX_MEM_INC      DMA_CCR5_MINC
#define IR_SPI_DMA_TX_DIR          DMA_CCR5_DIR
#define IR_SPI_DMA_TX_TCIE         DMA_CCR5_TCIE
#define IR_SPI_DMA_TX_EN           DMA_CCR5_EN
#define IR_SPI_DMA_TX_IRQN         DMA1_Channel5_IRQn
#define IR_SPI_DMA_IRQ_HANDLER     DMA1_Channel5_IRQHandler

#define IR_TIM_CLK_R               APB1ENR
#define IR_TIM_CLK_EN              RCC_APB1ENR_TIM3EN
#define IR_TIM_REMAP               AFIO_MAPR_TIM3_REMAP
#define IR_TIM                     TIM3
#define IR_TIM_IRQN                TIM3_IRQn
#define IR_TIM_IRQ_HANDLER         TIM3_IRQHandler

typedef __packed struct Ports_s
{
  U8 P[BUFF_COUNT];
} Ports_t, * Ports_p;

typedef struct sImageRefreshing
{
  unsigned char   InProgress;
  unsigned char * StartAddress;
  unsigned char * EndAddress;
  unsigned char   PartSize;
  unsigned char * CurrentAddress;
  unsigned int    SubChainLength;
  unsigned int    SubChainCounter;
} tImageRefreshing, * pImageRefreshing;

#endif //__RS_IMAGE_REFRESHING_H
