#ifndef _MAIN_TASK_ROADSIGN_H_
#define _MAIN_TASK_ROADSIGN_H_

#include "types.h"

//void RoadSign_Init( void );
void RoadSign_On( void );
void RoadSign_Off( void );
void RoadSign_ThreadInit( void );
void RoadSign_SetImage(U8 * ImageBuffer, U16 aSize);
void RoadSign_SetSlide(U8 * ImageBuffer, U16 aSize);
void RoadSign_SetBrightness(U8 Brightness);
void RoadSign_SetDelay(U16 Delay);

#endif //_MAIN_TASK_ROADSIGN_H_
