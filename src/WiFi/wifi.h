#ifndef __H_DRV_WIFI_H__
#define __H_DRV_WIFI_H__

#include "types.h"
#include "gpio.h"

/* ESP8266 POWER = PC1 - GPIO, Push-Pull */
#define ESP8266_POWER_PORT                  GPIOC
#define ESP8266_POWER_PIN                   1

/* ESP8266 RESET = PC0 - GPIO, Push-Pull */
#define ESP8266_RESET_PORT                  GPIOC
#define ESP8266_RESET_PIN                   0

typedef enum WiFiMode_e
{
  E_WIFI_MODE_AP,
  E_WIFI_MODE_STATION,
  E_WIFI_MODE_DOUBLE,
  E_WIFI_MODE_WPS,
} WiFiMode_t;

typedef enum Socket_e
{
  E_SOCKET_NONE = 0,
  E_SOCKET_CLIENT,
  E_SOCKET_SERVER,
} Socket_t;

typedef enum SocketType_e
{
  E_SOCKET_TYPE_TCP,
  E_SOCKET_TYPE_UDP,
  E_SOCKET_TYPE_SSL,
} SocketType_t;

#define ESP8266_Power_Off()      GPIO_Hi( ESP8266_POWER_PORT, ESP8266_POWER_PIN )
#define ESP8266_Power_On()       GPIO_Lo( ESP8266_POWER_PORT, ESP8266_POWER_PIN )

#define ESP8266_Reset_Lo()       GPIO_Lo( ESP8266_RESET_PORT, ESP8266_RESET_PIN )
#define ESP8266_Reset_Hi()       GPIO_Hi( ESP8266_RESET_PORT, ESP8266_RESET_PIN )

void H_drvWiFi_Init(void);
U8 H_drvWiFi_WaitReady(U32 aTimeOut);
U8 H_drvWiFi_Start(U32 aTimeOut);
U8 H_drvWiFi_Test(void);
U8 H_drvWiFi_Connect(WiFiMode_t aMode, U8 * pSSID, U8 * pPass);
U8 H_drvWiFi_IsConnected(void);

U8 SocketOpen(Socket_t aSocket, SocketType_t aType, U8 * pLink, U16 aPort);
U8 SocketWrite(U8 aSocket, U8 * pData, U16 aSize);
U8 SocketRead(U8 * pSocket, U8 * pData, U16 * pSize);
U8 SocketClose(U8 aSocket);

U8 H_drvWiFi_AT_GetStatus(void);

U8 H_drvWiFi_AT_Execute(U8 * pCommand);
U8 H_drvWiFi_AT_Wait(U8 * pAnswer, U32 aTimeOut);
U8 H_drvWiFi_AT_Get(U8 * pParameter, const U8 * pFormat, ...);
U8 H_drvWiFi_AT_Set(U8 * pParameter, const U8 * pFormat, ...);

void WiFi_Connect(void);
U8 RawFifo_Put( U8 aByte );
U8 RawFifo_Get( U8 * pByte );

#endif //__H_DRV_WIFI_H__
