#ifndef __RS_PROCESSING_H
#define __RS_PROCESSING_H

#include "types.h"

typedef struct sRS
{
  unsigned int   ChainSize;
  unsigned char  Chain[4096];
  unsigned char  WidthInModules;
  unsigned char  HeightInModules;
  unsigned char  ModuleWidth;
  unsigned char  ModuleHeight;
  unsigned int   SubChainLen;
} tRS, * pRS;

void p20mrgb_unpack2chain( U8C * aPackedBuf, U16C aPackedSize, pRS RS, U8 aWidth, U8 aHeight );

void ImageRefreshing_Init( pRS aRS );
void ImageRefreshing_Start( void );
U8   ImageRefreshing_End( void );
void ImageRefreshing_SetBrightness(U8 aBrightness);
U8 ImageRefreshing_IsOn( void );
void ImageRefreshing_OneShift( void );

#endif //__RS_PROCESSING_H
