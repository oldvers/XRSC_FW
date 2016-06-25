#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "RTL.h"
#include "uart.h"
#include "wifi.h"

#define SIGNAL_DATA_RECEIVED          0x01
#define SIGNAL_PROCESSING_COMPLETED   0x02

#define WIFI_EV_CONNECTED             0x01
#define WIFI_EV_GOT_IP                0x02
#define WIFI_EV_DISCONNECTED          0x04
#define WIFI_EV_CONNECT               0x08
#define WIFI_EV_DISCONNECT            0x10
#define WIFI_EV_SOCKET_OPENED_C       0x20
#define WIFI_EV_SOCKET_OPENED_S       0x40
#define WIFI_EV_SOCKET_CLOSED         0x80

typedef enum WiFiRxData_e
{
  E_WIFI_RX_DATA_AT,
  E_WIFI_RX_DATA_RAW,
} WiFiRxData_t;

typedef enum WiFiRx_e
{
  E_WIFI_RX_OK = 0,
  E_WIFI_RX_ERROR,
  E_WIFI_RX_LINE,
  E_WIFI_RX_RAW,
  E_WIFI_RX_CONNECTED,
  E_WIFI_RX_GOT_IP,
  E_WIFI_RX_DISCONNECTED,
  E_WIFI_RX_TIMEOUT,
  E_WIFI_RX_READY,
  E_WIFI_RX_SOCKET_OPENED,
  E_WIFI_RX_SOCKET_CLOSED,
  E_WIFI_RX_READY_FOR_RAW,
} WiFiRx_t;


/* CMSIS RTOS */
//static void WiFiRxThread(void const *argument);
//static void WiFiThread(void const *argument);
/* Keil RTX */
static __task void WiFiRxThread(void);
static __task void WiFiStatusThread(void);


typedef union WiFiMessage_s
{
  struct Command_s
  {
    U32 Type   : 4;
    U32 ID     : 2;
    U32 Length : 16;
  } Cmd;
  U32 Raw;
} WiFiMessage_t, * WiFiMessage_p;

/* CMSIS RTOS */
//osThreadId  WiFiRxThreadId;
//osThreadDef(WiFiRxThread, WiFiRxThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
//osThreadId  WiFiThreadId;
//osThreadDef(WiFiThread, WiFiThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

/* Keil RTX */
static OS_TID WiFiRxThreadId;
static OS_TID WiFiThreadId;

/* CMSIS RTOS */
//osMessageQId   WiFiRxSyncQueueId;
//osMessageQId   WiFiRxRawQueueId;
//osSemaphoreId  WiFiRxSemaphoreId;
//osMutexId      WiFiMutexId;
//osMessageQDef(WiFiRxSyncQueue, 1, U32);
//osMessageQDef(WiFiRxRawQueue, 16, U32);
//osSemaphoreDef(WiFiRxSemaphore);
//osMutexDef(WiFiMutex);

/* Keil RTX */
os_mbx_declare(WiFiRxSyncQueue, 1);
os_mbx_declare(WiFiRxRawQueue, 16);
static OS_MUT WiFiMutex;


#define WIFI_DEFAULT_TIMEOUT  15000
#define WIFI_RESET_TIMEOUT    15000
#define WIFI_CONNECT_TIMEOUT  15000
#define WIFI_SOCKET_TIMEOUT   15000

typedef struct WiFi_s
{
  U8   RxBuffer[256];
  U8   TxBuffer[256];
  U32  TimeOut;
  U8   Waiting;
  struct Status_s
  {
    U8  Connected : 1;
    U8  Ready : 1;
  } Status;
  Socket_t Socket;
  U8       SSID[64];
  U8       PWD[64];
} WiFi_t, * WiFi_p;

static WiFi_t WiFi = { {0}, {0}, WIFI_DEFAULT_TIMEOUT, 0, {0, 0}, E_SOCKET_NONE, {0}, {0} };

/*----------------------------------------------------------------------------*/

static struct RawFifo_s
{
  S16  I;
  S16  O;
  U16  C;
  U8   B[4096];
} RawFifo = { 0, 0, 4096, {0} };

/*----------------------------------------------------------------------------*/

U8 RawFifo_Put( U8 aByte )
{
  if(RawFifo.I == ((RawFifo.O - 1 + RawFifo.C) % RawFifo.C))
  {
    return 0; //Queue Full
  }

  RawFifo.B[RawFifo.I] = aByte;
  RawFifo.I = (RawFifo.I + 1) % RawFifo.C;
    
  return 1; //No Errors
}

/*----------------------------------------------------------------------------*/

U8 RawFifo_Get( U8 * pByte )
{
  if(RawFifo.I == RawFifo.O)
  {
    return 0; //Queue Empty - Nothing to get
  }

  *pByte = RawFifo.B[RawFifo.O];
  RawFifo.O = (RawFifo.O + 1) % RawFifo.C;

  return 1; //No Errors
}

/*----------------------------------------------------------------------------*/

typedef struct Line_s
{
  U8 * Start;
  U16  Length;
} LT;

struct Lines_s
{
  U8  Count;
  LT  Item[16];
} Lines = { 0, {0} };

/*----------------------------------------------------------------------------*/

__weak void H_WiFi_RxReadyCallback( U8 * pAnswer, U16 aSize )
{
  return;
}

/*----------------------------------------------------------------------------*/

void H_drvWiFi_Init( void )
{
  Lines.Count = 0;
  WiFi.Status.Ready = 0;

  /* ESP8266 POWER = PC1 - GPIO, Push-Pull */
  GPIO_Init( ESP8266_POWER_PORT, ESP8266_POWER_PIN, GPIO_TYPE_OUT_PP_2MHZ );

  /* ESP8266 RESET = PC0 - GPIO, Push-Pull */
  GPIO_Init( ESP8266_RESET_PORT, ESP8266_RESET_PIN, GPIO_TYPE_OUT_PP_2MHZ );

  /* CMSIS RTOS */
  //WiFiRxSyncQueueId = osMessageCreate(osMessageQ(WiFiRxSyncQueue), NULL);
  //SYSTEM_SW_ERROR_CHECK(NULL != WiFiRxSyncQueueId);
  /* Keil RTX */
  os_mbx_init(&WiFiRxSyncQueue, sizeof(WiFiRxSyncQueue));
  
  /* CMSIS RTOS */
  //WiFiRxRawQueueId = osMessageCreate(osMessageQ(WiFiRxRawQueue), NULL);
  //SYSTEM_SW_ERROR_CHECK(NULL != WiFiRxRawQueueId);
  /* Keil RTX */
  os_mbx_init(&WiFiRxRawQueue, sizeof(WiFiRxRawQueue));
  
  /* CMSIS RTOS */
  //WiFiMutexId = osMutexCreate(osMutex(WiFiMutex));
  //SYSTEM_SW_ERROR_CHECK(NULL != WiFiMutexId);
  /* Keil RTX */
  os_mut_init(&WiFiMutex);
  
  /* CMSIS RTOS */
  //WiFiRxThreadId = osThreadCreate(osThread(WiFiRxThread), NULL); 
  //SYSTEM_SW_ERROR_CHECK(NULL != WiFiRxThreadId);
  /* Keil RTX */
  WiFiRxThreadId = os_tsk_create(WiFiRxThread, 10);
  
  /* CMSIS RTOS */
  //WiFiThreadId = osThreadCreate(osThread(WiFiThread), NULL); 
  //SYSTEM_SW_ERROR_CHECK(NULL != WiFiThreadId);
  /* Keil RTX */
  WiFiThreadId = os_tsk_create(WiFiStatusThread, 10);
}

/*----------------------------------------------------------------------------*/

//Notification : Data was received via UART
void ESP8266_UART_RxReadyCallback( void )
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiRxThreadId, SIGNAL_DATA_RECEIVED );
  /* Keil RTX */
  isr_evt_set(SIGNAL_DATA_RECEIVED, WiFiRxThreadId);
}

/*----------------------------------------------------------------------------*/

//Notification : Command/Line processing completed
void h_drvWiFi_NotifyProcessingCompleted( void )
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiRxThreadId, SIGNAL_PROCESSING_COMPLETED );
  /* Keil RTX */
  os_evt_set(SIGNAL_PROCESSING_COMPLETED, WiFiRxThreadId);
}

/*----------------------------------------------------------------------------*/

U8 WiFiLock( U32 aTimeOut )
{
  /* CMSIS RTOS */
  //return (U8)( osOK == osMutexWait(WiFiMutexId, aTimeOut) );
  /* Keil RTX */
  return (U8)( OS_R_TMO != os_mut_wait(&WiFiMutex, aTimeOut) );
}

/*----------------------------------------------------------------------------*/

void WiFiUnlock( void )
{
  /* CMSIS RTOS */
  //osMutexRelease(WiFiMutexId);
  /* Kei RTX */
  os_mut_release(&WiFiMutex);
}

/*----------------------------------------------------------------------------*/

void WiFiIndicateRxTxtReady( U8 aType, U16 aLength )
{
  WiFiMessage_t Msg;
  
  Msg.Cmd.Type = aType;
  Msg.Cmd.ID = 0;
  Msg.Cmd.Length = aLength;
  
  /* Put message to the Queue */
  /* CMSIS RTOS */
  //osMessagePut(WiFiRxSyncQueueId, Msg.Raw, osWaitForever);
  /* Keil RTX */
  os_mbx_send(&WiFiRxSyncQueue, (void *)Msg.Raw, 0xFFFF);

  
  /* Wait till message processing completed */
  /* CMSIS RTOS */
  //osSignalWait(SIGNAL_PROCESSING_COMPLETED, osWaitForever);
  /* Keil RTX */
  os_evt_wait_or(SIGNAL_PROCESSING_COMPLETED, 0xFFFF);
}

/*----------------------------------------------------------------------------*/

void WiFiIndicateRxRawReady( U8 aID, U16 aLength )
{
  WiFiMessage_t Msg;
  
  Msg.Cmd.Type = E_WIFI_RX_RAW;
  Msg.Cmd.Length = aLength;
  
  /* CMSIS RTOS */
  //osMessagePut(WiFiRxRawQueueId, Msg.Raw, 0);
  /* Keil RTX */
  os_mbx_send(&WiFiRxRawQueue, (void *)Msg.Raw, 0);
}

/*----------------------------------------------------------------------------*/

WiFiRxData_t WiFiWaitForRx( U8 * pBuffer, U16 * pLength )
{
  static WiFiRxData_t DataType = E_WIFI_RX_DATA_AT;
  static U16 Index = 0, IpdLength = 0, RawLength = 0;
  U8 Byte;
  U8 ID;
  
  *pLength = 0;
  
  while ( 1 )
  {
    while ( 1 == ESP8266_UART_GetByte(&Byte) )
    {
      if( DataType == E_WIFI_RX_DATA_AT )
      {
        switch( Byte )
        {
          case '+':
          case 'I':
          case 'P':
          case 'D':
          case ',':
            pBuffer[Index++] = Byte;
            IpdLength++;
            break;

          case ':':
            pBuffer[Index++] = Byte;
            if( (IpdLength == 5) && (RawLength <= 2048) )
            {
              Index = RawLength;
              DataType = E_WIFI_RX_DATA_RAW;
            }
            if( (IpdLength == 7) && (RawLength <= 2048) )
            {
              Index = RawLength;
              DataType = E_WIFI_RX_DATA_RAW;
            }
            IpdLength = 0;
            break;
          case '>':
            pBuffer[Index++] = Byte;
          case 0x0D:
            if( Index > 0 )
            {
              *pLength = Index;
              pBuffer[Index++] = 0x00;
              Index = 0;
              return E_WIFI_RX_DATA_AT;
            }
          case 0x0A:
            break;
          default:
            pBuffer[Index++] = Byte;

            switch ( IpdLength )
            {
              case 5: // "+IPD,XXXX:"
                RawLength = (Byte - 0x30) + RawLength * 10;
                break;
              case 6: // "+IPD,X,"
                ID = RawLength;
                if ( ID > 4 ) ID = 0;
                RawLength = (Byte - 0x30);
                IpdLength++;
                break;
              case 7: // "+IPD,X,XXXX:"
                RawLength = (Byte - 0x30) + RawLength * 10;
                break;
              default:
                IpdLength = 0;
                RawLength = 0;
                break;
            }
            break;
        }
      }
      else
      {
        RawFifo_Put( Byte );
        Index--;
      
        if( 0 == Index )
        {
          WiFiIndicateRxRawReady(ID, RawLength);
          DataType = E_WIFI_RX_DATA_AT;

          return E_WIFI_RX_DATA_RAW;
        }
      }
    }
    
    /* CMSIS RTOS */
    //osSignalWait(SIGNAL_DATA_RECEIVED, osWaitForever);
    /* Keil RTX */
    os_evt_wait_or( SIGNAL_DATA_RECEIVED, 0xFFFF );
  }
}

/*----------------------------------------------------------------------------*/

__weak void WiFi_ConnectedCallback(void)
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiThreadId, WIFI_EV_CONNECTED );
  /* Keil RTX */
  os_evt_set( WIFI_EV_CONNECTED, WiFiThreadId );
}

/*----------------------------------------------------------------------------*/

__weak void WiFi_GotIPCallback(void)
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiThreadId, WIFI_EV_GOT_IP );
  /* Keil RTX */
  os_evt_set( WIFI_EV_GOT_IP, WiFiThreadId );
}

/*----------------------------------------------------------------------------*/

__weak void WiFi_DisConnectedCallback(void)
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiThreadId, WIFI_EV_DISCONNECTED );
  /* Keil RTX */
  os_evt_set( WIFI_EV_DISCONNECTED, WiFiThreadId );
}

/*----------------------------------------------------------------------------*/

__weak void WiFi_DisConnect(void)
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiThreadId, WIFI_EV_DISCONNECT );
  /* Keil RTX */
  os_evt_set( WIFI_EV_DISCONNECT, WiFiThreadId );
}

/*----------------------------------------------------------------------------*/

U8 * WiFiParseAt( U8 * pBuffer, U16 aLength )
{
  if ( (aLength == 2) && (strncmp((char *)pBuffer, "OK", aLength) == 0) )
  {
    WiFiIndicateRxTxtReady(E_WIFI_RX_OK, 0);
    Lines.Count = 0;
    return WiFi.RxBuffer;
  }
  
  if ( (aLength == 5) && (strncmp((char *)pBuffer, "ERROR", aLength) == 0) )
  {
    WiFiIndicateRxTxtReady(E_WIFI_RX_ERROR, 0);
    Lines.Count = 0;
    return WiFi.RxBuffer;
  }
  
  if ( (aLength == 14) && (strncmp((char *)pBuffer, "WIFI CONNECTED", aLength) == 0) )
  {
    WiFi_ConnectedCallback();
    return pBuffer;
  }
  
  if ( (aLength == 11) && (strncmp((char *)pBuffer, "WIFI GOT IP", aLength) == 0) )
  {
    WiFi_GotIPCallback();
    return pBuffer;
  }
  
  //TODO:CONNECT,DISCONNECT,SEND OK
  if ( (aLength == 7) && (strncmp((char *)pBuffer, "CONNECT", aLength) == 0) )
  {
    /* CMSIS RTOS */
    //osSignalSet( WiFiThreadId, WIFI_EV_SOCKET_CONNECTED );
    /* CMSIS RTOS */
    os_evt_set( WIFI_EV_SOCKET_OPENED_C, WiFiThreadId );
    return pBuffer;
  }
  
  if ( (aLength == 9) && (strncmp((char *)(pBuffer + 2), "CONNECT", aLength) == 0) )
  {
    //WiFiIndicateRxTxtReady(E_WIFI_RX_SOCKET_CONNECTED, 0);
    //osSignalSet( WiFiThreadId, WIFI_EV_SOCKET_CONNECTED );
    os_evt_set( WIFI_EV_SOCKET_OPENED_S, WiFiThreadId );
    return pBuffer;
  }
  
  if ( (aLength == 1) && (strncmp((char *)pBuffer, ">", aLength) == 0) )
  {
    WiFiIndicateRxTxtReady(E_WIFI_RX_READY_FOR_RAW, 0);
    Lines.Count = 0;
    return WiFi.RxBuffer;
  }
  
  if ( (aLength == 7) && (strncmp((char *)pBuffer, "SEND OK", aLength) == 0) )
  {
    WiFiIndicateRxTxtReady(E_WIFI_RX_OK, 0);
    Lines.Count = 0;
    return WiFi.RxBuffer;
  }
  
  if ( (aLength == 6) && (strncmp((char *)pBuffer, "CLOSED", aLength) == 0) )
  {
    /* CMSIS RTOS */
    //osSignalSet( WiFiThreadId, WIFI_EV_SOCKET_CONNECTED );
    /* Keil RTX */
    os_evt_set( WIFI_EV_SOCKET_CLOSED, WiFiThreadId );
    return pBuffer;
  }
  
  if ( (aLength == 8) && (strncmp((char *)(pBuffer + 2), "CLOSED", aLength) == 0) )
  {
    /* CMSIS RTOS */
    //osSignalSet( WiFiThreadId, WIFI_EV_SOCKET_CONNECTED );
    /* Keil RTX */
    os_evt_set( WIFI_EV_SOCKET_CLOSED, WiFiThreadId );
    return pBuffer;
  }
  
  if ( (aLength == 15) && (strncmp((char *)pBuffer, "WIFI DISCONNECT", aLength) == 0) )
  {
    WiFi_DisConnectedCallback();
    return pBuffer;
  }
  else
  {
    if ( 0 == WiFi.Waiting )
    {
      if ( 16 > Lines.Count )
      {
        Lines.Item[Lines.Count].Start = pBuffer;
        Lines.Item[Lines.Count].Length = aLength;
        Lines.Count++;
        return (pBuffer + aLength + 1);
      }
      else
      {
        return pBuffer;
      }
    }
    else
    {
      if ( 0 ==  strncmp((char *)pBuffer, (char *)WiFi.TxBuffer, WiFi.Waiting) )
      {
        WiFiIndicateRxTxtReady(E_WIFI_RX_LINE, aLength);
        WiFi.Waiting = 0;
      }
      return pBuffer;
    }
  }
}

/*----------------------------------------------------------------------------*/

/* CMSIS RTOS */
//void WiFiRxThread(void const *argument)
/* Keil RTX */
__task void WiFiRxThread(void)
{
  U16  Length = 0;
  U8 * Buffer = WiFi.RxBuffer;
  
  ESP8266_Power_Off();
  os_dly_wait(100);
  ESP8266_Power_On();
  os_dly_wait(100);
  ESP8266_Reset_Lo();
  os_dly_wait(50);
  ESP8266_Reset_Hi();
  os_dly_wait(500);
  
  ESP8266_UART_Init();
  
  WiFiIndicateRxTxtReady(E_WIFI_RX_READY, 0);
  
  while ( 1 )
  {
    if ( E_WIFI_RX_DATA_AT == WiFiWaitForRx( Buffer, &Length ) )
    {
      Buffer = WiFiParseAt( Buffer, Length );
    }
  }
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_WaitReady(U32 aTimeOut)
{
  /* CMSIS RTOS */
  //osEvent       Event;
  /* Keil RTX */
  OS_RESULT     MsgRes;
  WiFiMessage_t Msg;
  U8            Result = 0;

  /* CMSIS RTOS */
  //Event = osMessageGet(WiFiRxSyncQueueId, aTimeOut);
  /* Keil RTX */
  MsgRes = os_mbx_wait(&WiFiRxSyncQueue, (void *)&Msg.Raw, aTimeOut);
  
  /* CMSIS RTOS */
  //if (Event.status == osEventMessage)
  /* Keil RTX */
  if( OS_R_TMO != MsgRes )
  {
    /* CMSIS RTOS */
    //Msg.Raw = Event.value.v;
    //Msg = *((WiFiMessage_p)pMsg);
    //if( (Event.status == osEventMessage) && (E_WIFI_RX_READY == Msg.Cmd.Type) )
    /* Keil RTX */
    if( E_WIFI_RX_READY == Msg.Cmd.Type )
    {
      Result = 1;
      /* CMSIS RTOS */
      //osSignalSet( WiFiRxThreadId, SIGNAL_PROCESSING_COMPLETED );
      /* Keil RTX */
      os_evt_set( SIGNAL_PROCESSING_COMPLETED, WiFiRxThreadId );
    }
  }

  return Result;
}

/*----------------------------------------------------------------------------*/

WiFiRx_t H_drvWiFi_Read(U8 * pBuffer, U16 * pLength, U32 aTimeOut)
{
  /* CMSIS RTOS */
  //osEvent         Event;
  //void *        pMsg;
  /* Keil RTX */
  OS_RESULT     MsgRes;
  WiFiMessage_t   Msg;
  WiFiRx_t        Result = E_WIFI_RX_TIMEOUT;

  /* CMSIS RTOS */
  //Event = osMessageGet(WiFiRxSyncQueueId, aTimeOut);
  /* Keil RTX */
  MsgRes = os_mbx_wait(&WiFiRxSyncQueue, (void *)&Msg.Raw, aTimeOut);
  
  /* CMSIS RTOS */
  //if (Event.status == osEventMessage)
  /* Keil RTX */
  if( OS_R_TMO != MsgRes )
  {
    /* CMSIS RTOS */
    //Msg.Raw = Event.value.v;
    //Msg = *((WiFiMessage_p)pMsg);
    if ( NULL != pBuffer ) pBuffer = WiFi.RxBuffer;
    if ( NULL != pLength ) *pLength = Msg.Cmd.Length;
    Result = (WiFiRx_t)Msg.Cmd.Type;
  }
  else
  {
    *pBuffer = NULL;
    *pLength = 0;
  }

  return Result;
}

/*----------------------------------------------------------------------------*/

void H_drvWiFi_Write(U8 * pBuffer, U16 aSize)
{
  for ( U16 i = 0; i < aSize; i++ ) ESP8266_UART_PutByte( pBuffer[i] );
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_AT_Execute(U8 * pCommand)
{
  U16 tLength, rLength;
  WiFiRx_t Result;
  U8 Repeats = 3;
  
  if ( NULL == pCommand )
  {
    tLength = sprintf((char *)WiFi.TxBuffer, "AT\r\n");
  }
  else
  {
    tLength = sprintf((char *)WiFi.TxBuffer, "AT%s\r\n", pCommand);
  }
  if( tLength == 0 ) return 0;
  
  while ( 0 < Repeats-- )
  {
    H_drvWiFi_Write(WiFi.TxBuffer, tLength);
  
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;
  
    h_drvWiFi_NotifyProcessingCompleted();
  
    if ( E_WIFI_RX_OK == Result ) return 1;
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_AT_Set(U8 * pParameter, const U8 * pFormat, ...)
{
  U16 tLength, rLength;
  char * Buffer;
  WiFiRx_t Result;
  U8 Repeats = 3;

  Buffer = (char *)WiFi.TxBuffer;
  
  Buffer += sprintf(Buffer, "AT%s=", pParameter);
  
  va_list ArgList;
  va_start( ArgList, pFormat );
  Buffer += vsprintf( Buffer, (const char *)pFormat, ArgList );
  va_end( ArgList );

  Buffer += sprintf(Buffer, "\r\n");
  
  tLength = (Buffer - (char *)WiFi.TxBuffer);
  
  if ( 0 == tLength ) return 0;
  
  
  while ( 0 < Repeats-- )
  {
    H_drvWiFi_Write(WiFi.TxBuffer, tLength);
  
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;
  
    h_drvWiFi_NotifyProcessingCompleted();
  
    if ( E_WIFI_RX_OK == Result ) return 1;
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_AT_Get(U8 * pParameter, const U8 * pFormat, ...)
{
  U16       tLength, wLength, rLength;
  char *    Buffer;
  int       ParamsCount = EOF;
  WiFiRx_t  Result;
  U8        Repeats = 3;

  tLength = sprintf((char *)WiFi.TxBuffer, "AT%s?\r\n", pParameter);
  
  if ( tLength == 0 ) return 0;
  
  wLength = strlen( (char *)pParameter );
  
  if ( wLength == 0 ) return 0;
  
 
  while ( 0 < Repeats-- )
  {
    H_drvWiFi_Write(WiFi.TxBuffer, tLength);
    
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;

    if ( 0 < Lines.Count )
    {
      Buffer = (char *)Lines.Item[0].Start;
      
      if ( 0 == strncmp( (char *)pParameter, Buffer, wLength ) )
      {
        Buffer += wLength;
        
        va_list ArgList;
        va_start( ArgList, pFormat );
        ParamsCount = vsscanf( (char *)Buffer, (char *)pFormat, ArgList );
        va_end( ArgList );
      }
    }
    
    h_drvWiFi_NotifyProcessingCompleted();
    
    if (( E_WIFI_RX_OK == Result ) && ( 0 < ParamsCount )) return ParamsCount;
  }
  
  return 0;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_AT_GetStatus(void)
{
  U16       tLength, rLength;
  char *    Buffer;
  WiFiRx_t  Result;
  U8        Repeats = 3;
  U8        Status = 0;

  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;

  tLength = sprintf((char *)WiFi.TxBuffer, "AT+CIPSTATUS\r\n");
  
  if ( tLength == 0 ) return 0;
  
  while ( 0 < Repeats-- )
  {
    H_drvWiFi_Write(WiFi.TxBuffer, tLength);
    
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;

    if ( 0 < Lines.Count )
    {
      Buffer = (char *)Lines.Item[0].Start;
      
      if ( 0 == strncmp( "STATUS:", Buffer, 7 ) )
      {
        Buffer += 7;
        Status = *Buffer - 0x30;
          
        if (( 2 > Status ) || ( 5 < Status ))
        {
          Status = 0;
        }
      }
    }
    
    if ( 3 == Status )
    {
      Buffer = (char *)Lines.Item[1].Start;

      if ( E_WIFI_RX_LINE == Result )
      {
        //TODO: Parse Connection info
      }
    }
    

    h_drvWiFi_NotifyProcessingCompleted();
    
    if (( E_WIFI_RX_OK == Result ) && ( 0 != Status )) break;
  }
  
  WiFiUnlock( );
  
  return Status;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_AT_Wait( U8 * pAnswer, U32 aTimeOut )
{
  U16       wLength, rLength;
  WiFiRx_t  Result;
  U8        Ready = 0;
  
  wLength = sprintf((char *)WiFi.TxBuffer, "%s", (char *)pAnswer);
  
  if ( wLength == 0 ) return 0;
  
  WiFi.Waiting = wLength;
  
  do
  {
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, aTimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) return 0;
    
    if ( E_WIFI_RX_LINE == Result ) Ready = 1;
    
    h_drvWiFi_NotifyProcessingCompleted();
  }
  while ( Ready == 0 );
  
  return Ready;
}
  
/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_Start(U32 aTimeOut)
{
  U8 AutoConnect;
  
  WiFi.Status.Ready = 0;
  WiFi.Status.Connected = 0;
  
  //Wait till another thread finish its working with module
  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;
  
  while ( 0 == WiFi.Status.Ready )
  {
    //Check if module succesfully answers for Test Command
    if ( 0 == H_drvWiFi_AT_Execute(NULL) )
    {
      ESP8266_Reset_Lo();
      os_dly_wait(30);
      ESP8266_Reset_Hi();
      os_dly_wait(100);
      continue;
    }
    
    //Software reset
    if ( 0 == H_drvWiFi_AT_Execute((U8 *)"+RST") ) continue;
    
    //Wait till module ready for work
    if ( 0 == H_drvWiFi_AT_Wait((U8 *)"ready", WIFI_RESET_TIMEOUT) ) continue;
    
    //Switch off the echo
    if ( 0 == H_drvWiFi_AT_Execute((U8 *)"E0") ) continue;
    
    //Get AT Commands set version
    if ( 0 == H_drvWiFi_AT_Execute((U8 *)"+GMR") ) continue;
    
    //Check if Autoconnection is off
    if ( 0 == H_drvWiFi_AT_Get((U8 *)"+CWAUTOCONN", (U8 *)":%d", &AutoConnect) ) continue;
    
    //Switch off AutoConnect
    if ( 0 != AutoConnect )
    {
      if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CWAUTOCONN", (U8 *)"%d", 0) ) continue;
    }
    
    //Set Tx power (Max legal power is 100 mW)
    //mW -- 1 2 3 4 5 6 8 10 12 15 20 25 30 40 50 60 80 100
    //dBm - 1 2 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
    //RF TX power, range 0 ~ 82, unit : 0.25 dBm
    if ( 0 == H_drvWiFi_AT_Set((U8 *)"+RFPOWER", (U8 *)"%d", 80) ) continue;
    
    //Module is inited succesfully
    WiFi.Status.Ready = 1;
  }
  
  //Unlock the module
  WiFiUnlock( );

  return WiFi.Status.Ready;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_Test( void )
{
  U8 Result = 0;
  
  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;
  
  Result = H_drvWiFi_AT_Execute(NULL);
  
  WiFiUnlock( );
  
  return Result;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_Connect(WiFiMode_t aMode, U8 * pSSID, U8 * pPass)
{
  U8       Ret = 0;
  U8       Mode;
  WiFiRx_t Result;
  
  if ( 0 == WiFi.Status.Ready ) return 0;

  if ( 1 == WiFi.Status.Connected ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFiLock( WiFi.TimeOut * 10 ) ) return 0;
  
  switch ( aMode )
  {
    case E_WIFI_MODE_AP: //---------------------------------------------------
    {
      //Check WiFi Mode
      if( 1 != H_drvWiFi_AT_Get((U8 *)"+CWMODE_CUR", (U8 *)":%d", &Mode) ) break;
    
      //Switch to Access Piont mode
      if ( 2 != Mode )
      {
        if ( 1 != H_drvWiFi_AT_Set((U8 *)"+CWMODE_CUR", (U8 *)"%d", 2) ) break;
      }
      
      //Set SSID of Access Point, Password, Channel etc.
      if ( 1 != H_drvWiFi_AT_Set( (U8 *)"+CWSAP_CUR",
                                  (U8 *)"\"%s\",\"%s\",%d,%d,%d",
                                  pSSID,   /* ssid */
                                  pPass,   /* pwd */
                                  3,       /* chl */
                                  4,       /* ecn == WPA_WPA2_PSK */
                                  1        /* max_conn */
                                ) ) break;
      
      //Set the default IP address of Access Point
      if ( 1 != H_drvWiFi_AT_Set( (U8 *)"+CIPAP_CUR", 
                                  (U8 *)"\"%s\",\"%s\",\"%s\"",
                                  "192.168.4.1",
                                  "192.168.4.1",
                                  "255.255.255.0"
                                ) ) break;
      
      WiFi.Status.Connected = 1;
      
      Ret = 1;
      break;
    }
    case E_WIFI_MODE_STATION: //----------------------------------------------
    {
      //Check WiFi Mode
      if( 1 != H_drvWiFi_AT_Get((U8 *)"+CWMODE_CUR", (U8 *)":%d", &Mode) ) break;
    
      //Switch to Station mode
      if ( 1 != Mode )
      {
        if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CWMODE_CUR", (U8 *)"%d", 1) ) break;
      }
      
      //Connect to Access Point
      WiFi.TimeOut = WIFI_CONNECT_TIMEOUT;
      if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CWJAP_CUR", (U8 *)"\"%s\",\"%s\"", pSSID, pPass) ) break;
      
      Ret = 1;
      
      break;
    }
    case E_WIFI_MODE_DOUBLE: //-----------------------------------------------
    {
      break;
    }
    case E_WIFI_MODE_WPS: //--------------------------------------------------
    {
      //Check WiFi Mode
      if( 1 != H_drvWiFi_AT_Get((U8 *)"+CWMODE_CUR", (U8 *)":%d", &Mode) ) break;
    
      //Switch to Station mode
      if ( 1 != Mode )
      {
        if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CWMODE_CUR", (U8 *)"%d", 1) ) break;
      }
      
      //Try to connect via WPS
      if ( 0 == H_drvWiFi_AT_Set((U8 *)"+WPS", (U8 *)"%d", 1) ) break;
      
      //Wait for got IP
      do
      {
        Result = H_drvWiFi_Read(NULL, NULL, WiFi.TimeOut * 10);
  
        if ( E_WIFI_RX_TIMEOUT == Result )
        {
          H_drvWiFi_AT_Set((U8 *)"+WPS", (U8 *)"%d", 0);
          break;
        }

        if ( E_WIFI_RX_GOT_IP == Result )
        {
          WiFi.Status.Connected = 1;
        }
      }
      while ( WiFi.Status.Connected == 0 );
      
      break;
    }
    default: //---------------------------------------------------------------
    {
      break;
    }
  }
  
  WiFi.TimeOut = WIFI_DEFAULT_TIMEOUT;
  
  WiFiUnlock( );
  
  return Ret;
}

/*----------------------------------------------------------------------------*/

U8 H_drvWiFi_IsConnected(void)
{
  return WiFi.Status.Connected;
}

U8 H_drv_WiFi_IsReady(void)
{
  return WiFi.Status.Ready;
}

/*----------------------------------------------------------------------------*/

U8 WiFi_WaitForEvent( U32 aTimeOut )
{
  /* CMSIS RTOS */
  //osEvent evt;
  /* Keil RTX */
  OS_RESULT EvtRes;
  U16 res = 0;
  
  /* CMSIS RTOS */
  //evt = osSignalWait(0xFF, aTimeOut);
  /* Keil RTX */
  EvtRes = os_evt_wait_or(0xFF, aTimeOut);
  
  /* CMSIS RTOS */
  //if ( evt.status == osEventSignal)
  /* Keil RTX */
  if ( OS_R_EVT == EvtRes )
  {
    /* CMSIS RTOS */
    //res = evt.value.signals;
    /* Keil RTX */
    res = os_evt_get( );
  }
  
  return (U8)res;
}

/*----------------------------------------------------------------------------*/

void WiFi_Connect(void)
{
  /* CMSIS RTOS */
  //osSignalSet( WiFiThreadId, WIFI_EV_CONNECT );
  /* Keil RTX */
  os_evt_set( WIFI_EV_CONNECT, WiFiThreadId );
}

/*----------------------------------------------------------------------------*/

void WiFi_ProcessEvents(U8 aEvents)
{
  if ( WIFI_EV_DISCONNECTED == (aEvents & WIFI_EV_DISCONNECTED) )
  {
    WiFi.Status.Connected = 0;
    WiFi.Socket = E_SOCKET_NONE;
  }
  
  if ( WIFI_EV_CONNECTED == (aEvents & WIFI_EV_CONNECTED) )
  {
    WiFi.Status.Connected = 0;
    WiFi.Socket = E_SOCKET_NONE;
  }

  if ( WIFI_EV_GOT_IP == (aEvents & WIFI_EV_GOT_IP) )
  {
    WiFi.Status.Connected = 1;
  }

//  if ( WIFI_EV_CONNECT == (aEvents & WIFI_EV_CONNECT) )
//  {
////    H_drvWiFi_Connect( E_WIFI_MODE_STATION, (U8 *)"HomeWLAN", (U8 *)"wlanH020785endrix!" );
//  }

  if ( WIFI_EV_DISCONNECT == (aEvents & WIFI_EV_DISCONNECT) )
  {
    WiFi.Status.Connected = 0;
    WiFi.Socket = E_SOCKET_NONE;
  }
  
  if ( WIFI_EV_SOCKET_OPENED_C == (aEvents & WIFI_EV_SOCKET_OPENED_C) )
  {
    WiFi.Socket = E_SOCKET_CLIENT;
  }
  
  if ( WIFI_EV_SOCKET_OPENED_S == (aEvents & WIFI_EV_SOCKET_OPENED_S) )
  {
    WiFi.Socket = E_SOCKET_SERVER;
  }
  
  if ( WIFI_EV_SOCKET_CLOSED == (aEvents & WIFI_EV_SOCKET_CLOSED) )
  {
    WiFi.Socket = E_SOCKET_NONE;
  }
}

/*----------------------------------------------------------------------------*/

U8 WiFi_CheckStatus(void)
{
  U8 Status;
  
  Status = H_drvWiFi_AT_GetStatus( );
    
  if ( 0 == Status ) return 0;
  
  switch ( Status )
  {
    case 2: //Got IP (WiFi)
      //WiFi.Connected = 1;
      break;
    case 3: //Connected (IP/Net)
      break;
    case 4: //Disconnected (IP/Net)
      break;
    case 5: //WiFi Connection Fail (WiFi)
      //WiFi.Connected = 0;
      break;
  }
  
  return 1;
}

/*----------------------------------------------------------------------------*/

/* CMSIS RTOS */
//void WiFiThread(void const *argument)
/* Keil RTX */
__task void WiFiStatusThread(void)
{
  U8 Events;
  
  /* CMSIS RTOS */
  //H_drvWiFi_WaitReady( osWaitForever );
  /* Keil RTX */
  H_drvWiFi_WaitReady( 0xFFFF );
  
  while ( 1 )
  {
    H_drvWiFi_Start( 10000 );

    while ( 1 == H_drv_WiFi_IsReady() )
    {
      Events = WiFi_WaitForEvent( 30000 );
      
      if ( 0 != Events )
      {
        WiFi_ProcessEvents( Events );
      }
      else
      {
        if ( 0 == WiFi_CheckStatus( ) ) break;
      }
    }
  }
}

/*----------------------------------------------------------------------------*/

U8 SocketOpen(Socket_t aSocket, SocketType_t aType, U8 * pLink, U16 aPort)
{
  /* CMSIS RTOS */
  //WiFiRx_t Result;
  /* Keil RTX */
  char sType[5];
  U8 Ret = 0;
  
  if ( 0 == WiFi.Status.Ready ) return 0;
  if ( 0 == WiFi.Status.Connected ) return 0; //TODO: Error definitions, Disconnection
  
  //if ( 1 == WiFi.Socket ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;
  
  switch ( aSocket )
  {
    case E_SOCKET_CLIENT: //--------------------------------------------------
    {
      switch ( aType )
      {
        case E_SOCKET_TYPE_TCP:
          sprintf(sType, "TCP");
          break;
        case E_SOCKET_TYPE_UDP:
          sprintf(sType, "UDP");
          break;
        case E_SOCKET_TYPE_SSL:
          sprintf(sType, "SSL");
          break;
      }
      
      //Switch to single connection mode
      if( 1 != H_drvWiFi_AT_Set((U8 *)"+CIPMUX", (U8 *)"%d", 0) ) break;
    
      //Connect to the Server
      if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CIPSTART", (U8 *)"\"%s\",\"%s\",%d", sType, pLink, aPort) ) break;
        
      Ret = 1;
      
      break;
    }
    case E_SOCKET_SERVER: //--------------------------------------------------
    {
      //Switch to multiple connection mode
      if( 1 != H_drvWiFi_AT_Set((U8 *)"+CIPMUX", (U8 *)"%d", 1) ) break;
    
      //Start the Server
      if ( 0 == H_drvWiFi_AT_Set((U8 *)"+CIPSERVER", (U8 *)"%d,%d", 1, aPort) ) break;
        
      Ret = 1;
      
      break;
    }
    default: //---------------------------------------------------------------
    {
      break;
    }
  }
  
  WiFiUnlock( );
  
  return Ret;
}

/*----------------------------------------------------------------------------*/

U8 SocketWrite(U8 aSocket, U8 * pData, U16 aSize)
{
  U8 Ret = 0;
  U16 rLength;
  WiFiRx_t Result;
  U8 Repeats = 3;
  
  if ( 0 == WiFi.Status.Ready ) return 0;
  if ( 0 == WiFi.Status.Connected ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFi.Socket ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;

  if ( 0 == aSize ) return 0;
  
  while ( 0 < Repeats-- )
  {
    //Write the Send Command with Raw Data Size
    if ( E_SOCKET_CLIENT == WiFi.Socket )
    {
      if( 1 != H_drvWiFi_AT_Set((U8 *)"+CIPSEND", (U8 *)"%d", aSize) ) continue;
    }
    else
    {
      if( 1 != H_drvWiFi_AT_Set((U8 *)"+CIPSEND", (U8 *)"%d,%d", aSocket, aSize) ) continue;
    }
  
    //Wait for ">"
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    //If TimeOut - repeat command
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;

    //If ">" received - Send Raw Data
    if ( E_WIFI_RX_READY_FOR_RAW == Result )
    {
      H_drvWiFi_Write(pData, aSize);
    }
    
    h_drvWiFi_NotifyProcessingCompleted();
  
    //Wait for "SEND OK"
    Result = H_drvWiFi_Read(WiFi.RxBuffer, &rLength, WiFi.TimeOut);
  
    if ( E_WIFI_RX_TIMEOUT == Result ) continue;

    h_drvWiFi_NotifyProcessingCompleted();
    
    if ( E_WIFI_RX_OK == Result )
    {
      Ret = 1;
      break;
    }
  }
  
  WiFiUnlock( );
  
  return Ret;
}

/*----------------------------------------------------------------------------*/
  
U8 SocketRead(U8 * pSocket, U8 * pData, U16 * pSize)
{
  /* CMSIS RTOS */
  //osEvent         Event;
  //void *          pMsg;
  /* Keil RTX */
  OS_RESULT       MsgRes;
  WiFiMessage_t   Msg;
  U8              Result = 0;

  if ( NULL == pData ) return 0;

  if ( 0 == WiFi.Status.Ready ) return 0;
  if ( 0 == WiFi.Status.Connected ) return 0; //TODO: Error definitions, Disconnection
  
  //if ( E_SOCKET_NONE == WiFi.Socket ) return 0; //TODO: Error definitions, Disconnection
  
  //if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;
  
  /* CMSIS RTOS */
  //Event = osMessageGet(WiFiRxRawQueueId, WiFi.TimeOut);
  /* Keil RTX */
  MsgRes = os_mbx_wait(&WiFiRxRawQueue, (void *)&Msg.Raw, WiFi.TimeOut);
  
  /* CMSIS RTOS */
  //if (Event.status == osEventMessage)
  /* Keil RTX */
  if( OS_R_TMO != MsgRes )
  {
    /* CMSIS RTOS */
    //Msg.Raw = Event.value.v;
    //Msg = *((WiFiMessage_p)pMsg);
    
    *pSize = Msg.Cmd.Length;
    *pSocket = Msg.Cmd.ID;
    
    for ( U16 i = 0; i < Msg.Cmd.Length; i++ )
    {
      if ( 0 ==  RawFifo_Get( pData++ ) )
      {
        *pSize = i;
        break;
      }
    }

    Result = 1;
  }
  else
  {
    *pSize = 0;
  }
  
  //WiFiUnlock( );

  return Result;
}

/*----------------------------------------------------------------------------*/

U8 SocketClose(U8 aSocket)
{
  U8 Ret = 0;
  
  if ( 0 == WiFi.Status.Ready ) return 0;
  if ( 0 == WiFi.Status.Connected ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFi.Socket ) return 0; //TODO: Error definitions, Disconnection
  
  if ( 0 == WiFiLock( WiFi.TimeOut ) ) return 0;
  
  switch ( aSocket )
  {
    case E_SOCKET_CLIENT: //--------------------------------------------------
    {
      if( 1 != H_drvWiFi_AT_Execute((U8 *)"+CIPCLOSE") ) break;

      Ret = 1;

      break;
    }
    case E_SOCKET_SERVER: //--------------------------------------------------
    {
      if( 1 != H_drvWiFi_AT_Set((U8 *)"+CIPCLOSE", (U8 *)"%d", aSocket) ) break;
      
      Ret = 1;
      
      break;
    }
    default: //---------------------------------------------------------------
    {
      break;
    }
  }
  
  WiFi.Socket = E_SOCKET_NONE;
  
  WiFiUnlock( );
  
  return Ret;
}

/*----------------------------------------------------------------------------*/
