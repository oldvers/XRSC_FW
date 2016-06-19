#include "rsUnpacking.h"

//Buffer for Unpacked Picture
unsigned char unpacked[MAX_PICTURE_SIZE];

unsigned char * unpack_full(unsigned char const * apackedbuf, unsigned short const apackedsize)
{
  unsigned char CurByte;
  unsigned char UnpByte;
  int CurBit;
  unsigned char * P;
  int i, j;
  unsigned short UnpIndex;
 
  CurByte = 0;
  CurBit = 0;
  UnpByte = 0;
  UnpIndex = 0;
  P = (unsigned char *)apackedbuf;

  for(i = 0; i < apackedsize; i++)
  {
    CurByte = *P;

    for(j = 0; j < 8; j++)
    {
      if( CurByte & (1 << j) )
      {
        if(CurBit % 3 == 0) UnpByte |= 0x01;
        if(CurBit % 3 == 1) UnpByte |= 0x02;
        if(CurBit % 3 == 2) UnpByte |= 0x04;
      }
      if(CurBit % 3 == 2)
      {
        unpacked[UnpIndex] = UnpByte;
        UnpIndex++;
        UnpByte = 0;
      }
      CurBit++;
    }
    P++;
  }
  
  return (unsigned char *)&unpacked;
}




unsigned char * unpack_mono(unsigned char const * apackedbuf, unsigned short const apackedsize)
{
  unsigned char CurByte;
  unsigned char UnpByte;
  unsigned char * P;
  int i, j;
  unsigned short UnpIndex;

  CurByte = 0;
  UnpByte = 0;
  UnpIndex = 0;
  P = (unsigned char *)apackedbuf;

  for(i = 0; i < apackedsize; i++)
  {
    CurByte = *P;

    for(j = 0; j < 8; j++)
    {
      if( CurByte & (1 << j) ) UnpByte = 0x07;
      unpacked[UnpIndex] = UnpByte;
      UnpIndex++;
      UnpByte = 0;
    }
    P++;
  }
  
  return (unsigned char *)&unpacked;
}
