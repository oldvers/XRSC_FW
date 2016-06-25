#include "rsProcessing.h"
#include "rsUnpacking.h"
#include "rsImageRefreshing.h"
#include "p20mrgb.h"

const U8 BuffBits[2][6] =
{
  /* Buff 0 */
  {
    /* Row 1 */
    (1 << P20M_ROW1_R),
    (1 << P20M_ROW1_G),
    (1 << P20M_ROW1_B),
    /* Row 2 */
    (1 << P20M_ROW2_R),
    (1 << P20M_ROW2_G),
    (1 << P20M_ROW2_B),
  },
  /* Buff 1 */
  {
    /* Row 3 */
    (1 << P20M_ROW3_R),
    (1 << P20M_ROW3_G),
    (1 << P20M_ROW3_B),
    /* Row 4 */
    (1 << P20M_ROW4_R),
    (1 << P20M_ROW4_G),
    (1 << P20M_ROW4_B),
  },
};

/*----------------------------------------------------------------------------*/

void p20mrgb_unpack2chain(U8C * aPackedBuf, U16C aPackedSize, pRS RS, U8 aColumns, U8 aRows)
{
  U8 CurByte, Buff, Bit;
  U16 DiodeInChain, DiodeInImage;
  U32 PX, PY, col, colOffset, colDelta, row, rowOffset, rowDelta;
  U8 * unpacked;
  Ports_p PF;
  
  unpacked = unpack_full(aPackedBuf, aPackedSize);
  
  RS->ChainSize = (aColumns * P20M_WIDTH * P20M_HEIGHT * BUFF_COUNT);
  RS->WidthInModules = aColumns;
  RS->HeightInModules = aRows;
  RS->ModuleWidth = P20M_WIDTH;
  RS->ModuleHeight = P20M_HEIGHT;
  RS->SubChainLen = (aColumns * P20M_WIDTH * P20M_HEIGHT);
  
  colOffset = 0;
  colDelta  = P20M_WIDTH;// * P20M_HEIGHT;
  rowOffset = 0;
  rowDelta  = P20M_WIDTH * P20M_HEIGHT * aColumns;
  
  for(col = 0; col < aColumns; col++)
  {
    DiodeInImage = 0;
    
    for(PY = 0; PY < P20M_HEIGHT; PY++)
    {
      for(PX = 0; PX < P20M_WIDTH; PX++)
      {
        //DiodeInChain = (127 - DiodeInImage) - ((DiodeInImage >> 3) & 1) * 56 + ((DiodeInImage >> 4) & 7) * 8;
        DiodeInChain = DiodeInImage + ((DiodeInImage >> 3) & 1) * 56 - ((DiodeInImage >> 4) & 7) * 8;
        DiodeInImage++;

        //PF = (Ports_p)&RS->Chain[DiodeInChain * BUFF_COUNT];
        PF = (Ports_p)&RS->Chain[(DiodeInChain + col * P20M_WIDTH * P20M_HEIGHT) * BUFF_COUNT];
        for(CurByte = 0; CurByte < BUFF_COUNT; CurByte++) PF->P[CurByte] = 0;
      
        rowOffset = 0;
        for(row = 0; row < aRows; row++)
        {
          //Buff = BUFF_COUNT - (row / BUFF_COUNT) - 1;
          Buff = row / BUFF_COUNT;
          Bit = (row % 2) * 3;
          //Row 1
          //CurByte = unpacked[PY * P20M_WIDTH + PX];
          CurByte = unpacked[rowOffset + colOffset + PY * P20M_WIDTH * aColumns + PX];
          if(CurByte & 0x01) PF->P[Buff] |= BuffBits[Buff][Bit + 0];
          if(CurByte & 0x02) PF->P[Buff] |= BuffBits[Buff][Bit + 1];
          if(CurByte & 0x04) PF->P[Buff] |= BuffBits[Buff][Bit + 2];
          
          rowOffset += rowDelta;
        }
      }
    }
    
    colOffset += colDelta;
  }
}

/*----------------------------------------------------------------------------*/
