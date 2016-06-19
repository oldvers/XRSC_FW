#include <stdio.h>
#include <string.h>
#include "RTL.h"
#include "uniquedevid.h"
#include "wifi.h"
#include "main_task_wifi.h"
#include "main_task_roadsign.h"

static U8  CmdBuffer[2048];
static U16 CmdLength;
static U8  ImageBuffer[1024];
static U8  SSID[16];
static U8  Pass[16];

static __task void WiFiThread(void);

/*----------------------------------------------------------------------------*/

void WiFi_ThreadInit( void )
{
  H_drvWiFi_Init();

  os_tsk_create( WiFiThread, 10 );
}

/*----------------------------------------------------------------------------*/

void StrToRaw(U8 * pStr, U8 * pRaw, U16 aSize)
{
  aSize /= 2;
  
  while ( aSize-- )
  {
    *pRaw =(((pStr[0] > '9' ? pStr[0] - '7' : pStr[0] - '0') << 4) |
             (pStr[1] > '9' ? pStr[1] - '7' : pStr[1] - '0'));
    pRaw++;
    pStr += 2;
  }
}

/*----------------------------------------------------------------------------*/

U8 WiFiParseCommand(void)
{
  U8   Brightness, Result = 0;
  U8 * pPacket = CmdBuffer;
  U16  Delay;
  
  //Road Sign On
  if ( 0 == strncmp((char *)CmdBuffer, "ROAD_SIGN->ON", sizeof("ROAD_SIGN->ON") - 1) )
  {
    RoadSign_On();
  }

  //Road Sign Off
  if ( 0 == strncmp((char *)CmdBuffer, "ROAD_SIGN->OFF", sizeof("ROAD_SIGN->OFF") - 1) )
  {
    RoadSign_Off();
  }  

  //Set Image
  if ( 0 == strncmp((char *)pPacket, "SHOW_IMG->", sizeof("SHOW_IMG->") - 1) )
  {
    pPacket += (sizeof("SHOW_IMG->") - 1);
    
    Brightness = 0;
    while ( ':' != *pPacket )
    {
      Brightness = Brightness * 10 + (*pPacket++ - '0');
    }
    pPacket++;
    
    if ( 1152 == (CmdLength - (pPacket - CmdBuffer)) )
    {
      StrToRaw((U8*)pPacket, ImageBuffer, 1152);
      RoadSign_SetImage(ImageBuffer, 576);
      RoadSign_SetBrightness(Brightness);
    }
  }

  //Set Slide
  if ( 0 == strncmp((char *)pPacket, "SHOW_SLD->", sizeof("SHOW_SLD->") - 1) )
  {
    pPacket += (sizeof("SHOW_SLD->") - 1);
    
    Brightness = 0;
    while ( ',' != *pPacket )
    {
      Brightness = Brightness * 10 + (*pPacket++ - '0');
    }
    pPacket++;
    
    Delay = 0;
    while ( ':' != *pPacket )
    {
      Delay = Delay * 10 + (*pPacket++ - '0');
    }
    pPacket++;
    
    if ( 1152 == (CmdLength - (pPacket - CmdBuffer)) )
    {
      StrToRaw((U8*)pPacket, ImageBuffer, 1152);
      RoadSign_SetSlide(ImageBuffer, 576);
      RoadSign_SetBrightness(Brightness);
      RoadSign_SetDelay(Delay);
    }
//  if(numberImg >= 9)
//  {   
//      numberImg =0;
//  }                   
//  if(os_mut_wait (&setIMG, 100)!=OS_R_TMO)
//  {   
//      memcpy((u8*)&slideShowImg[numberImg *576],getImage,576);    
//      numberImg++;
//      os_mut_release (&setIMG); 
//  }
  }

  return Result;
}

/*----------------------------------------------------------------------------*/

//U8 SetDefSlideShow(void)
//{
//  U8 Result = 0;
//  
//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_1", sizeof("SET_ANIMATION_1")) )
//  {
//    Result = 1;
//  }
//  
//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_2", sizeof("SET_ANIMATION_2")) )
//  {
//    Result = 2;
//  }
//  
//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_3", sizeof("SET_ANIMATION_3")) )
//  {
//    Result = 3;
//  }

//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_4", sizeof("SET_ANIMATION_4")) )
//  {
//    Result = 4;
//  }

//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_5", sizeof("SET_ANIMATION_5")) )
//  {
//    Result = 5;
//  }
//  
//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_6", sizeof("SET_ANIMATION_6")) )
//  {
//    Result = 6;
//  }

//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_7", sizeof("SET_ANIMATION_7")) )
//  {
//    Result = 7;
//  }
//  
//  if ( 0 == strncmp((char *)CmdBuffer, "SET_ANIMATION_8", sizeof("SET_ANIMATION_8")) )
//  {
//    Result = 8;
//  }
//  
//  return Result;
//}

/*----------------------------------------------------------------------------*/

U8 WiFiParsePacket(void)
{
  U8   Result = 0;
  U16  wCS, rCS, i;
  
  //Check if '$' simbol is present on the end of the packet
  if ( '$' == CmdBuffer[CmdLength - 5] )
  {
    CmdLength -= 4;
    
    //Read received Control Sum
    StrToRaw(&CmdBuffer[CmdLength], (U8 *)&rCS, 4);
    
    //Calculate the Control Sum of packet
    wCS = 0;
    for ( i = 0; i < CmdLength; i++ )
    {
      wCS += CmdBuffer[i];
    }
    
    //If received CS equals calculated CS - parse the command in the packet
    if ( rCS == wCS )
    {
      CmdLength -= 1;

      Result = WiFiParseCommand();
    }
  }
  
  return Result;
}

/*----------------------------------------------------------------------------*/

__task void WiFiThread(void)
{
  U8  Socket;

  while ( 1 )
  {
    //Start WiFi Access Point
    sprintf( (char *)SSID, "RS%08X", UDID_0 );
    sprintf( (char *)Pass, "%s", "RoadSign" );
    if ( 1 != H_drvWiFi_Connect(E_WIFI_MODE_AP, SSID, Pass) )
    {
      os_dly_wait(5000);
      continue;
    }

    //Start the Server
    SocketOpen( E_SOCKET_SERVER, E_SOCKET_TYPE_TCP, NULL, 333 );
    
    while ( H_drvWiFi_IsConnected() )
    {
      //Read Available Request
      if ( 1 == SocketRead(&Socket, CmdBuffer, &CmdLength) )
      {
        if ( 1 == WiFiParsePacket() )
        {
          //Write the Answer
          SocketWrite(Socket, CmdBuffer, CmdLength);
        }
      }
    }
  }
}
