/******************************************************************************
  Filename:       GenericApp.c
  Revised:        $Date: 2014-09-07 13:36:30 -0700 (Sun, 07 Sep 2014) $
  Revision:       $Revision: 40046 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2014 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License"). You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product. Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful - it is
  intended to be a simple example of an application's structure.

  This application periodically sends a "Hello World" message to
  another "Generic" application (see 'txMsgDelay'). The application
  will also receive "Hello World" packets.

  This application doesn't have a profile, so it handles everything
  directly - by itself.

  Key control:
    SW1:  changes the delay between TX packets
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "hal_SHT30.h"
#include "GenericApp.h"
#include "DebugTrace.h"
#include "hal_GPIO.h"
#include "hal_WaterBat.h"

#if !defined( WIN32 ) || defined( ZBIT )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
  GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
  GENERICAPP_ENDPOINT,              //  int Endpoint;
  GENERICAPP_PROFID,                //  uint16 AppProfId[2];
  GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  GENERICAPP_FLAGS,                 //  int   AppFlags:4;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
  GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // GenericApp_Init() is called.
static byte EndDevice_Sensor_Type;

devStates_t GenericApp_NwkState;

byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

// Number of recieved messages
static uint16 rxMsgCount;
static uint8 TxBufStart = 8;
static uint8 ReadWaterLevel[8] = {0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};
static uint8 TxBuf[128];
static uint8 RxBuf[128];
static bool  NetworkFlag=FALSE;
static uint8 WaterLevel_H=0;
static uint8 WaterLevel_L=0;
// Time interval between sending messages
//static uint32 txMsgDelay = GENERICAPP_SEND_MSG_TIMEOUT;

/*Sensor Type*/
#define TEMPERATURE_HUMITURE 0x01
#define FLOOD                0x03
#define SMOKE                0x04
#define WATERLEVEL           0x05

#define Debug 1
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendTheMessage(void);
static void UART_CallBack(uint8 port, uint8 event);
void MacAddr_Init(void);
#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
  //EndDevice_Sensor_Type = TEMPERATURE_HUMITURE;
  //EndDevice_Sensor_Type = FLOOD;
  EndDevice_Sensor_Type = SMOKE;
  //EndDevice_Sensor_Type = WATERLEVEL;
  
  //if (GenericApp_NwkState == DEV_ZB_COORD | GenericApp_NwkState == DEV_ROUTER)
  //  NLME_PermitJoining = FALSE;
  
  GenericApp_TaskID = task_id;
  GenericApp_NwkState = DEV_INIT;
  GenericApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  // Fill out the endpoint description.
  GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
  GenericApp_epDesc.task_id = &GenericApp_TaskID;
  GenericApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
  GenericApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &GenericApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( GenericApp_TaskID );
  //Init UART0
  if(Debug || GenericApp_NwkState == DEV_ZB_COORD)
  {
    Uart_Init(HAL_UART_PORT_0,HAL_UART_BR_115200);
  }
  if(GenericApp_NwkState == DEV_END_DEVICE)
  {
    MacAddr_Init();
    if(EndDevice_Sensor_Type == TEMPERATURE_HUMITURE)
    {
      HalSHT30Init();
      osal_start_timerEx( GenericApp_TaskID,
                          COLLECT_TEMPERATURE_HUMITURE_EVT,
                          COLLECT_TEMPERATURE_HUMITURE_TIMEOUT );
    }
    else if(EndDevice_Sensor_Type == FLOOD)
          osal_start_timerEx( GenericApp_TaskID,
                                      COLLECT_WATER_SOAK_EVT,
                                      COLLECT_WATER_SOAK_TIMEOUT);
    else if(EndDevice_Sensor_Type == SMOKE)
          osal_start_timerEx( GenericApp_TaskID,
                                      COLLECT_SMOKE_EVT,
                                      COLLECT_SMOKE_TIMEOUT);
    else if(EndDevice_Sensor_Type == WATERLEVEL)
    {
      Uart_Init(HAL_UART_PORT_1,HAL_UART_BR_9600);
      osal_start_timerEx( GenericApp_TaskID,
                          COLLECT_WATER_LEVEL_EVT,
                          COLLECT_WATER_LEVEL_TIMEOUT);
    }
  }
#if defined( IAR_ARMCM3_LM )
  // Register this task with RTOS task initiator
  RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif
}
//Init UART
void Uart_Init(uint8 port,uint8 baud)
{
  halUARTCfg_t uartConfig;
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = baud;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 64; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = 128;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = 6;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = UART_CallBack;
  HalUARTOpen (port, &uartConfig);
}

static void UART_CallBack(uint8 port, uint8 event)
{
  uint16 len=0;
  switch(port)
  {
    case HAL_UART_PORT_0:
      
      len=HalUARTRead(HAL_UART_PORT_0,RxBuf,128);
      if(len > 0)
      {
        HalUARTWrite( HAL_UART_PORT_0, RxBuf, len );
        len=0;
      }
      break;
    case HAL_UART_PORT_1:
      len=HalUARTRead(HAL_UART_PORT_1,RxBuf,128);
      if(len>0)
      {
        if(EndDevice_Sensor_Type == WATERLEVEL)
        {
          WaterLevel_H = RxBuf[3];
          WaterLevel_L = RxBuf[4];
          GenericApp_SendTheMessage();
        }
      }
      break;
  }
}

void MacAddr_Init(void)
{
  (void)NLME_GetExtAddr(); 
  uint8 j=7;
  for(int i=0;i<8;i++)
  {
    TxBuf[i] = saveExtAddr[j];
    j--;
  }
}
/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;

          sentEP = afDataConfirm->endpoint;
          (void)sentEP;  // This info not used now
          sentTransID = afDataConfirm->transID;
          (void)sentTransID;  // This info not used now

          sentStatus = afDataConfirm->hdr.status;
          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          GenericApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:         
          GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          switch(GenericApp_NwkState)
          {
            case DEV_ZB_COORD:
              break;
            case DEV_ROUTER:
              break;
            case DEV_END_DEVICE:
				NetworkFlag = TRUE;
              break;
            default:
              break;
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if(events & COLLECT_TEMPERATURE_HUMITURE_EVT)
  {
    HalSHT30ReadData();
    GenericApp_SendTheMessage();
    return (events ^ COLLECT_TEMPERATURE_HUMITURE_EVT);
  }
  
  if(events & COLLECT_SMOKE_EVT)
  {
    HalReadGPIOSta();
    return (events ^ COLLECT_SMOKE_EVT);
  }
  
  if(events & COLLECT_WATER_SOAK_EVT)
  {
    HalReadGPIOSta();
    return (events ^ COLLECT_WATER_SOAK_EVT);
  }
  
  if(events & COLLECT_WATER_LEVEL_EVT)
  {
    HalUARTWrite( HAL_UART_PORT_1, ReadWaterLevel, 8 );
    return (events ^ COLLECT_WATER_LEVEL_EVT);
  }

#if defined( IAR_ARMCM3_LM )
  // Receive a message from the RTOS queue
  if ( events & GENERICAPP_RTOS_MSG_EVT )
  {
    // Process message from RTOS queue
    GenericApp_ProcessRtosMessage();

    // return unprocessed events
    return (events ^ GENERICAPP_RTOS_MSG_EVT);
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        //HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined( BLINK_LEDS )
      else
      {
        // Flash LED to show failure
        //HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            GenericApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            //HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
  // Shift is used to make each button/switch dual purpose.

  if ( shift )
  {
    if ( keys & HAL_KEY_SW_6 )
    {
      
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
          //uint8 Data[]={ 'k','e','y','1'};
          //HalUARTWrite( UART_PORT, Data, 4 );
          //HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    }
    //Coordinator????????????
    if ( keys & HAL_KEY_SW_2 )
    {
     //uint8 Data[]={ 'k','e','y','2'};
     //     HalUARTWrite( UART_PORT, Data, 4 );
     // HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      
      //NLME_PermitJoiningRequest(60);
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {

    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case GENERICAPP_CLUSTERID:
      rxMsgCount += 1;  // Count this message
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_BLINK );  // Blink an LED
      HalUARTWrite(HAL_UART_PORT_0,pkt->cmd.Data,pkt->cmd.DataLength);
      break;
  }
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendTheMessage(void)
{
	if(!NetworkFlag) return;
	afAddrType_t my_DstAddr; 
    my_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    my_DstAddr.endPoint = GENERICAPP_ENDPOINT;
    my_DstAddr.addr.shortAddr = 0x0000;
    uint8 length = 0;
    if(EndDevice_Sensor_Type == TEMPERATURE_HUMITURE)
    {
        length=12;
        TxBuf[TxBufStart] = TEMPERATURE_HUMITURE;
        TxBuf[TxBufStart+1] = Humidity;
        TxBuf[TxBufStart+2] = (uint8)Temperature;
        TxBuf[TxBufStart+3] = (uint8)(Temperature>>8);
    }
    if(EndDevice_Sensor_Type == FLOOD)
    {
        length=10;
        TxBuf[TxBufStart] = FLOOD;
        TxBuf[TxBufStart+1]= GPIOStatic;
    }
    if(EndDevice_Sensor_Type == SMOKE)
    {
        length=10;
        TxBuf[TxBufStart] = SMOKE;
        TxBuf[TxBufStart+1] = GPIOStatic;
    }
    if(EndDevice_Sensor_Type == WATERLEVEL)
    {
        length = 11;
        TxBuf[TxBufStart] = WATERLEVEL;
        TxBuf[TxBufStart+1] = WaterLevel_H;
        TxBuf[TxBufStart+2] = WaterLevel_L;
    }
    AF_DataRequest( &my_DstAddr, &GenericApp_epDesc,
                       GENERICAPP_CLUSTERID,
                       length,
                       TxBuf,
                       &GenericApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS );
}
#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
  osalQueue_t inMsg;

  if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
  {
    uint8 cmndId = inMsg.cmnd;
    uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

    switch ( cmndId )
    {
      case CMD_INCR:
        counter += 1;  /* Increment the incoming counter */
                       /* Intentionally fall through next case */

      case CMD_ECHO:
      {
        userQueue_t outMsg;

        outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
        osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
        osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
        break;
      }

      default:
        break;  /* Ignore unknown command */
    }
  }
}
#endif

/*********************************************************************
 */
