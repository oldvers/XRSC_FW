#include <stdio.h>
#include <string.h>
#include "RTL.h"
#include "uniquedevid.h"
#include "wifi.h"
#include "main_task_wifi.h"
#include "main_task_roadsign.h"

/*----------------------------------------------------------------------------*/

#define RS_CMD_REFRESH_ON       0
#define RS_CMD_SET_BRIGHT       1
#define RS_CMD_SET_IMAGE        2
#define RS_CMD_SET_SLIDE        3
#define RS_CMD_SET_CLIP         4
#define RS_CMD_GET_STATUS       5
#define RS_CMD_REFRESH_OFF    126
 
/*----------------------------------------------------------------------------*/

typedef __packed struct RsPacketHeader_s
{
  U16 Length;     //0 - Length      - 2   bytes
  U8  Command;    //2 - Command     - 1   byte
  U8  Brightness; //3 - Brightness  - 1   byte
  U16 Parameter;  //4 - Parameter   - 2   bytes (Delay, Number of animation )
} RsPacketHeader_t, * RsPacketHeader_p;

typedef __packed struct RsPacketGeneral_s
{
  RsPacketHeader_t Header;
  U16              CS;
} RsPacketGeneral_t, * RsPacketGeneral_p;

typedef __packed struct RsPacketImage_s
{
  RsPacketHeader_t Header;
  U8  ImageCount; //6 - Image count - 1   byte  (optionally)
  U8  Image[576]; //7 - Image 0     - 576 bytes (optionally)
  U16 CS;         //    CS          - 2   bytes
} RsPacketImage_t, * RsPacketImage_p;

typedef __packed struct RsPacketSlide_s
{
  RsPacketHeader_t Header;
  U8  ImageCount;    //6 - Image count - 1       byte  (optionally)
  U8  Image[2][576]; //7 - Image 0,1   - 576 x 2 bytes (optionally)
  U16 CS;            //    CS          - 2       bytes
} RsPacketSlide_t, * RsPacketSlide_p;

typedef __packed struct RsPacketClip_s
{
  RsPacketHeader_t Header;
  U8  ImageCount;    //6 - Image count - 1       byte  (optionally)
  U8  Image[3][576]; //7 - Image 0,1   - 576 x 2 bytes (optionally)
  U16 CS;            //    CS          - 2       bytes
} RsPacketClip_t, * RsPacketClip_p;

typedef union RsPacket_u
{
  RsPacketGeneral_t General;
  RsPacketImage_t   Image;
  RsPacketSlide_t   Slide;
  RsPacketClip_t    Clip;
} RsPacket_t, * RsPacket_p;

/*----------------------------------------------------------------------------*/

static U8  CmdBuffer[2048];
static U16 CmdLength;
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

U8 WiFiParseCommand(void)
{
  U8         Result = 0;
  RsPacket_p pPacket = (RsPacket_p)CmdBuffer;

  //Road Sign On
  if ( RS_CMD_REFRESH_ON == pPacket->General.Header.Command )
  {
    RoadSign_On();
    RoadSign_SetBrightness(pPacket->General.Header.Brightness);
  }
  
  //Road Sign Set Brightness
  if ( RS_CMD_SET_BRIGHT == pPacket->General.Header.Command )
  {
    if ( 0 < pPacket->General.Header.Brightness )
    {
      RoadSign_On();
      RoadSign_SetBrightness(pPacket->General.Header.Brightness);
    }
    else
    {
      RoadSign_Off();
    }
  }
  
  //Road Sign Get Status
  if ( RS_CMD_GET_STATUS == pPacket->General.Header.Command )
  {
    pPacket->General.Header.Length     = sizeof(RsPacketGeneral_t);
    pPacket->General.Header.Command    = RS_CMD_GET_STATUS;
    pPacket->General.Header.Brightness = 0;
    pPacket->General.Header.Parameter  = 0;
    pPacket->General.CS                = sizeof(RsPacketGeneral_t) + RS_CMD_GET_STATUS;
    CmdLength = sizeof(RsPacketGeneral_t);
    Result = 1;
  }

  //Road Sign Off
  if ( RS_CMD_REFRESH_OFF == pPacket->General.Header.Command )
  {
    RoadSign_Off();
  }

  //Set Image
  if ( RS_CMD_SET_IMAGE == pPacket->General.Header.Command )
  {
    if ( 576 == (pPacket->General.Header.Length - sizeof(RsPacketGeneral_t) - 1) )
    {
      if ( 1 == pPacket->Image.ImageCount )
      {
        RoadSign_SetImage(pPacket->Image.Image, 576);
        RoadSign_SetBrightness(pPacket->Image.Header.Brightness);
      }
    }
  }

  //Set Slide
  if ( RS_CMD_SET_SLIDE == pPacket->General.Header.Command )
  {
    if ( 1152 == (pPacket->General.Header.Length - sizeof(RsPacketGeneral_t) - 1) )
    {
      if ( 2 == pPacket->Image.ImageCount )
      {
        RoadSign_SetSlide( pPacket->Slide.Image[0], 576, pPacket->Slide.Image[1], 576 );
        RoadSign_SetBrightness( pPacket->Image.Header.Brightness );
        RoadSign_SetDelay( pPacket->Slide.Header.Parameter );
      }
    }
  }

  return Result;
}

/*----------------------------------------------------------------------------*/

U8 WiFiParsePacket(void)
{
  U8   Result = 0;
  U16  rLen, wCS, rCS, i;
  
  rLen = (CmdBuffer[1] << 8) + CmdBuffer[0];
  
  if ( (rLen == CmdLength) && (rLen < 2048) )
  {
    //Read received Control Sum of packet
    rCS = (CmdBuffer[CmdLength - 1] << 8) + CmdBuffer[CmdLength - 2];
    
    //Calculate the Control Sum of packet
    wCS = 0;
    rLen -= 2;
    for ( i = 0; i < rLen; i++ )
    {
      wCS += CmdBuffer[i];
    }
    
    //If received CS equals calculated CS - parse the command in the packet
    if ( rCS == wCS )
    {
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
    sprintf( (char *)SSID, "RS%08X", UDID_0 ^ UDID_1 ^ UDID_2 );
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
