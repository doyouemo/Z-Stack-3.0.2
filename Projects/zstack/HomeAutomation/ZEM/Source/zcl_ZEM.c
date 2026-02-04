/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"

#include "zcl_ZEM.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "bdb_interface.h"
#include "bdb_Reporting.h"
   
/*********************************************************************
 * MACROS
 */

// how often to report temperature
#define ZEM_REPORT_INTERVAL   10000

#define GUI_LOCAL_TEMP    1

#define APP_TITLE "   Temp Sensor  "

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclZEM_TaskID;

extern int16 zdpExternalStateTaskID;

// UART Buffer
static uint8 zclZEM_UartBuf[ZCLZEM_UART_BUF_LEN];

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

uint8 zclZEM_NwkState = 0; // DEV_INIT

//static uint8 aProcessCmd[] = { 1, 0, 0, 0 }; // used for reset command, { length + cmd0 + cmd1 + data }

// Application Endpoint for data communication
static endPointDesc_t ZEM_AppEp =
{
  ZEM_ENDPOINT,                       // Application endpoint
  0,
  &zclZEM_TaskID,
  &zclZEM_SimpleDesc,                 // Simple description for this application
  (afNetworkLatencyReq_t)0            // No Network Latency req
};
#ifdef BDB_REPORTING
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 8
  uint8 reportableChange[] = {0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 0x2C01 is 300 in int16
#endif
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 4
  uint8 reportableChange[] = {0x2C, 0x01, 0x00, 0x00}; // 0x2C01 is 300 in int16
#endif 
#if BDBREPORTING_MAX_ANALOG_ATTR_SIZE == 2
  uint8 reportableChange[] = {0x2C, 0x01}; // 0x2C01 is 300 in int16
#endif 
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclZEM_HandleKeys( byte shift, byte keys );
static void zclZEM_BasicResetCB( void );

static void zclZEM_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclZEM_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclZEM_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclZEM_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclZEM_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclZEM_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclZEM_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclZEM_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_DISCOVER

static void zclSampleApp_BatteryWarningCB( uint8 voltLevel);

/*********************************************************************
 * STATUS STRINGS
 */

/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclZEM_CmdCallbacks =
{
  zclZEM_BasicResetCB,        // Basic Cluster Reset command
  NULL,                                           // Identify Trigger Effect command
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};

/*********************************************************************
 * @fn          zclZEM_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclZEM_Init( byte task_id )
{
  zclZEM_TaskID = task_id;

  // Register the Simple Descriptor for this application
  bdb_RegisterSimpleDescriptor( &zclZEM_SimpleDesc ); 
  
  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( ZEM_ENDPOINT, &zclZEM_CmdCallbacks );

  // Register the application's attribute list
  zclZEM_ResetAttributesToDefaultValues();
  zcl_registerAttrList( ZEM_ENDPOINT, zclZEM_NumAttributes, zclZEM_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclZEM_TaskID );

  // Register low voltage NV memory protection application callback
  RegisterVoltageWarningCB( zclSampleApp_BatteryWarningCB );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclZEM_TaskID );

  bdb_RegisterCommissioningStatusCB( zclZEM_ProcessCommissioningStatus );

#ifdef ZDO_COORDINATOR
  // 重置网络状态，强制每次都重新初始化网络
  // 设置ZCD_STARTOPT_DEFAULT_NETWORK_STATE位，使设备每次都重新组网
  extern uint8 zgWriteStartupOptions( uint8 action, uint8 bitOptions );
  
  // 设置默认网络状态选项
  zgWriteStartupOptions( 1, 0x01 );
  
  // 执行网络形成
  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_FORMATION |
                          BDB_COMMISSIONING_MODE_FINDING_BINDING );
  NLME_PermitJoiningRequest(255);
  // Broadcast
  osal_start_timerEx(zclZEM_TaskID,
                     ZEM_BROADCAST_EVT,
                     ZEM_BROADCAST_PERIOD);
  // groupcast
  osal_start_timerEx(zclZEM_TaskID,
                     ZEM_GROUPCAST_EVT,
                     ZEM_GROUPCAST_PERIOD);
#else
  bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_STEERING |
                          BDB_COMMISSIONING_MODE_FINDING_BINDING );
  // Add group
  aps_Group_t group = {
    .ID = GROUP_ID,
    .name = "",
  };
  aps_AddGroup(ZEM_ENDPOINT, &group);
  // P2P
  osal_start_timerEx(zclZEM_TaskID,
                     ZEM_P2P_EVT,
                     ZEM_P2P_PERIOD);
#endif

  // Initialize UART communication
  zclZEM_InitUart();
  
  // Read and send MAC address
  zclZEM_ReadAndSendMAC();

#ifdef BDB_REPORTING
  //Adds the default configuration values for the temperature attribute of the ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT cluster, for endpoint ZEM_ENDPOINT
  //Default maxReportingInterval value is 10 seconds
  //Default minReportingInterval value is 3 seconds
  //Default reportChange value is 300 (3 degrees)
  bdb_RepAddAttrCfgRecordDefaultToList(ZEM_ENDPOINT, ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, ATTRID_MS_TEMPERATURE_MEASURED_VALUE, 0, 10, reportableChange);
#endif
  
  zdpExternalStateTaskID = zclZEM_TaskID;
   osal_start_timerEx(
      zclZEM_TaskID,
      ZEMAPP_EVT,
      3000);
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclZEM_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclZEM_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclZEM_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case AF_INCOMING_MSG_CMD:
          // Incoming AF data messages
          zclZEM_AF_RxProc( MSGpkt );
          break;

        case KEY_CHANGE:
          zclZEM_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          // 处理网络状态变化
          {
            uint8 newState = ((uint8*)MSGpkt)[0];
            zclZEM_NwkState = newState; // 更新网络状态
            
            if (newState == 5) // DEV_END_DEVICE
            {
              // 终端设备已加入网络
              uint8 successMsg[] = "Network joined successfully!\r\n";
              HalUARTWrite(HAL_UART_PORT_0, successMsg, sizeof(successMsg)-1);
              
              // 联网成功后向协调器发送MAC地址
              #ifndef ZDO_COORDINATOR
              zclZEM_SendMACToCoordinator();
              #endif
            }
            else if (newState == 9) // DEV_NWK_ORPHAN
            {
              // 设备成为孤立设备
              #ifndef ZDO_COORDINATOR
              {
                static uint8 retryCount = 0;
                if (retryCount < 3)
                {
                  retryCount++;
                  osal_start_timerEx(zclZEM_TaskID, ZEM_REJOIN_EVT, 2000);
                }
              }
              #endif
            }
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // 先处理LED闪烁事件，确保不会被SYS_EVENT_MSG阻塞
  if(events & ZEMAPP_EVT)
  { 
    HalLedBlink(
        HAL_LED_1, // 指定使用LED1
        10, // 指定闪烁次数为10次
        50, // 指定50%的时间LED处于点亮状态
        1000); // 指定每次闪烁周期为1000ms
    
    // 设置下一次LED闪烁定时器
    osal_start_timerEx(zclZEM_TaskID, ZEMAPP_EVT, 10000);
    
    return (events ^ ZEMAPP_EVT);
  }
  
#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & ZEM_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ ZEM_END_DEVICE_REJOIN_EVT );
  }
#endif

  if ( events & ZEM_LCD_AUTO_UPDATE_EVT )
  {
    return ( events ^ ZEM_LCD_AUTO_UPDATE_EVT );
  }

  if ( events & ZEM_KEY_AUTO_REPEAT_EVT )
  {
    return ( events ^ ZEM_KEY_AUTO_REPEAT_EVT );
  }
  
  // Broadcast event
  if ( events & ZEM_BROADCAST_EVT )
  {
    uint8 broadcastData[] = "Broadcast";
    zclZEM_AF_Broadcast(CLUSTER_BROADCAST, 
                        sizeof(broadcastData)-1, 
                        broadcastData);
    osal_start_timerEx(zclZEM_TaskID,
                      ZEM_BROADCAST_EVT,
                      ZEM_BROADCAST_PERIOD);
    return ( events ^ ZEM_BROADCAST_EVT );
  }
  
  // Groupcast event
  if ( events & ZEM_GROUPCAST_EVT )
  {
    uint8 groupcastData[] = "Groupcast";
    zclZEM_AF_Groupcast(GROUP_ID,
                        CLUSTER_GROUPCAST,
                        sizeof(groupcastData)-1, 
                        groupcastData);
    osal_start_timerEx(zclZEM_TaskID,
                      ZEM_GROUPCAST_EVT,
                      ZEM_GROUPCAST_PERIOD);
    return ( events ^ ZEM_GROUPCAST_EVT );
  }
  
  // P2P event
  if ( events & ZEM_P2P_EVT )
  {
    uint8 p2pData[] = "P2P";
    zclZEM_AF_P2P(0x0000, // 目标设备网络地址（这里使用0x0000作为示例）
                  CLUSTER_P2P,
                  sizeof(p2pData)-1, 
                  p2pData);
    osal_start_timerEx(zclZEM_TaskID,
                      ZEM_P2P_EVT,
                      ZEM_P2P_PERIOD);
    return ( events ^ ZEM_P2P_EVT );
  }
  #ifndef ZDO_COORDINATOR
    if ( events & ZEM_REJOIN_EVT )//如果事件类型为重新加入网络事件
    {
        // 重新加入网络
        
        /* 重新加入网络 */
        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING |
                                    BDB_COMMISSIONING_MODE_FINDING_BINDING );
      return ( events ^ ZEM_REJOIN_EVT );
    }
  #endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclZEM_InitUart
 *
 * @brief   Initialize UART for communication
 *
 * @param   none
 *
 * @return  none
 */
static void zclZEM_InitUart(void)
{
  // 统一使用协调器的UART引脚配置（P1.4/P1.5）
  /********** 1. 配置UART0的P1.4/P1.5和PERCFG寄存器 **********/
  // PERCFG的bit0(U0CFG)设置为1：UART0映射到Alternative 2位置(P1.4/P1.5)
  PERCFG |= (1 << 0);  // 配置PERCFG |= 0x01来设置相应的bit0

  /********** 2. 配置P1.4/P1.5为UART0功能和P1端口寄存器 **********/
  // P1SEL的bit4(P1.4)和bit5(P1.5)设置为1：配置为外设功能(UART)
  P1SEL |= (1 << 4) | (1 << 5);  // 配置P1SEL |= 0x30(00110000)
  // P1DIR的bit4(P1.4=TX)设置为1：输出；bit5(P1.5=RX)设置为0：输入
  P1DIR |= (1 << 4);   // P1.4(TX)作为输出
  P1DIR &= ~(1 << 5);  // P1.5(RX)作为输入

  halUARTCfg_t uartConfig;
  /* UART Configuration */
  uartConfig.configured          = TRUE;      // Enable configuration
  uartConfig.baudRate            = HAL_UART_BR_115200;  // Baud rate
  uartConfig.flowControl         = FALSE;     // Disable hardware flow control
  uartConfig.flowControlThreshold= 0;         // Flow control threshold
  uartConfig.rx.maxBufSize       = ZCLZEM_UART_BUF_LEN;  // Receive buffer size
  uartConfig.tx.maxBufSize       = 0;         // Transmit buffer size
  uartConfig.idleTimeout         = 6;         // Default timeout
  uartConfig.intEnable           = TRUE;      // Enable interrupt
  uartConfig.callBackFunc        = zclZEM_UartCB;  // Set callback function
  /* Start UART */
  // 统一使用UART0端口（协调器配置）
  HalUARTOpen(HAL_UART_PORT_0, &uartConfig);  // Open UART0 with configuration
}


void zclZEM_SendMACToCoordinator(void)
{
  uint8* macAddr;
  uint8 macData[10]; // MAC地址数据包：1字节类型标识 + 8字节MAC地址 + 1字节结束符
  
  // 读取MAC地址
  macAddr = NLME_GetExtAddr();
  
  // 构建数据包
  macData[0] = 0x01; // 类型标识：MAC地址数据
  osal_memcpy(&macData[1], macAddr, 8); // 复制MAC地址
  macData[9] = 0x00; // 结束符
  
  // 向协调器发送数据（协调器网络地址通常为0x0000）
  zclZEM_AF_P2P(0x0000, // 协调器网络地址
                CLUSTER_P2P, // 使用点对点集群
                10, // 数据长度
                macData); // 数据内容
  
  // 同时通过串口显示发送信息
  uint8 sendMsg[] = "MAC address sent to coordinator!\r\n";
  HalUARTWrite(HAL_UART_PORT_0, sendMsg, sizeof(sendMsg)-1);
}

/*********************************************************************
 * @fn      zclZEM_ReadAndSendMAC
 *
 * @brief   Read MAC address and send it via UART
 *
 */
void zclZEM_ReadAndSendMAC(void)
{
  uint8* macAddr;
  uint8 macStr[50]; // 足够存储MAC地址字符串
  uint8 i, pos = 0;
  
  // Read MAC address from system
  macAddr = NLME_GetExtAddr();
  
  // 手动构建MAC地址字符串
  const uint8 header[] = "MAC: ";
  osal_memcpy(macStr, header, 4);
  pos = 4;
  
  // 添加MAC地址字节，格式化为十六进制
  for (i = 0; i < 8; i++) {
    // 转换高4位
    uint8 highNibble = (macAddr[i] >> 4) & 0x0F;
    macStr[pos++] = (highNibble < 10) ? ('0' + highNibble) : ('A' + highNibble - 10);
    
    // 转换低4位
    uint8 lowNibble = macAddr[i] & 0x0F;
    macStr[pos++] = (lowNibble < 10) ? ('0' + lowNibble) : ('A' + lowNibble - 10);
    
    // 添加冒号分隔符（最后一个字节后不加）
    if (i < 7) {
      macStr[pos++] = ':';
    }
  }
  
  // 添加换行符
  macStr[pos++] = '\r';
  macStr[pos++] = '\n';
  
  // Send MAC address via UART
  // 统一使用UART0端口（协调器配置）
  HalUARTWrite(HAL_UART_PORT_0, macStr, pos);
}

/*********************************************************************
 * @fn      zclZEM_UartCB
 *
 * @brief   UART callback function
 *
 * @param   port - UART port
 * @param   event - UART event
 *
 * @return  none
 */
static void zclZEM_UartCB(uint8 port, uint8 event)
{
  // 获取当前串口接收缓冲区有多少字节的数据
  uint8 rxLen = Hal_UART_RxBufLen(HAL_UART_PORT_0); 
  if(rxLen != 0) // 如果字节数数量不等于0 
  {
    // 从串口缓冲区中读取数据
    HalUARTRead(HAL_UART_PORT_0, zclZEM_UartBuf, rxLen);
    // 通过串口发送数据，例如把数据发送到串口助手
    HalUARTWrite(HAL_UART_PORT_0, zclZEM_UartBuf, rxLen);  
  }
}

/*********************************************************************
 * @fn      zclZEM_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclZEM_HandleKeys( byte shift, byte keys )
{
}

/*********************************************************************
 * @fn      zclZEM_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
static void zclZEM_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg)
{
    switch(bdbCommissioningModeMsg->bdbCommissioningMode)
    {
      case BDB_COMMISSIONING_FORMATION:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
          bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
          // 网络形成成功
          #ifdef ZDO_COORDINATOR
          uint8 successMsg[] = "Network formation successful!\r\n";
          HalUARTWrite(HAL_UART_PORT_0, successMsg, sizeof(successMsg)-1);
          #endif
        }
        else
        {
          //Want to try other channels?
          //try with bdb_setChannelAttribute
          // 网络形成失败
          #ifdef ZDO_COORDINATOR
          uint8 failMsg[] = "Network formation failed! Retrying...\r\n";
          HalUARTWrite(HAL_UART_PORT_0, failMsg, sizeof(failMsg)-1);
          
          // 打印具体的失败原因
          uint8 statusMsg[64];
          osal_memcpy(statusMsg, "Failure reason: ", 15);
          
          switch(bdbCommissioningModeMsg->bdbCommissioningStatus) {
            case BDB_COMMISSIONING_NO_NETWORK:
              osal_memcpy(&statusMsg[15], "No network found", 15);
              break;
            case BDB_COMMISSIONING_FORMATION_FAILURE:
              osal_memcpy(&statusMsg[15], "Formation failure", 17);
              break;
            case BDB_COMMISSIONING_TCLK_EX_FAILURE:
              osal_memcpy(&statusMsg[15], "TCLK exchange failure", 20);
              break;
            case BDB_COMMISSIONING_FAILURE:
              osal_memcpy(&statusMsg[15], "General failure", 14);
              break;
            default:
              osal_memcpy(&statusMsg[15], "Unknown error", 13);
              break;
          }
          osal_memcpy(&statusMsg[30], "\r\n", 2);
          HalUARTWrite(HAL_UART_PORT_0, statusMsg, 32);
          
          // 直接重新调用组网函数，不需要通过定时器
          bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_FORMATION |
                                      BDB_COMMISSIONING_MODE_FINDING_BINDING );
          #endif
        }
      break;
      case BDB_COMMISSIONING_NWK_STEERING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
          //We are on the nwk, what now?
          // 网络引导成功
          #ifdef ZDO_COORDINATOR
          uint8 deviceJoinedMsg[] = "Device joined the network!\r\n";
          HalUARTWrite(HAL_UART_PORT_0, deviceJoinedMsg, sizeof(deviceJoinedMsg)-1);
          #endif
        }
        else
        {
          // 网络引导失败
          #ifdef ZDO_COORDINATOR
          /* 协调器 */
          #else
          /* 路由器或者终端 */
          uint8 failMsg[] = "Network steering failed! Retrying in 1 second...\r\n";
          HalUARTWrite(HAL_UART_PORT_0, failMsg, sizeof(failMsg)-1);
          
          /* 无限重试网络引导 */
          osal_start_timerEx(zclZEM_TaskID, 
                                    ZEM_REJOIN_EVT, 
                                    ZEM_REJOIN_PERIOD);
          #endif
        }
        break;
      case BDB_COMMISSIONING_FINDING_BINDING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
        }
        else
        {
          //YOUR JOB:
          //retry?, wait for user interaction?
        }
      break;
      case BDB_COMMISSIONING_INITIALIZATION:
        //Initialization notification can only be successful. Failure on initialization 
        //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification
        
        //YOUR JOB:
        //We are on a network, what now?
        
      break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        //osal_start_timerEx(zclZEM_TaskID, SAMPLEAPP_END_DEVICE_REJOIN_EVT, SAMPLEAPP_END_DEVICE_REJOIN_DELAY);
        
        /* 协调器 */
#ifdef ZDO_COORDINATOR
        /* 路由器或者终端 */
#else
      /* 启动重新加入事件 */
      osal_start_timerEx(zclZEM_TaskID, 
                        ZEM_REJOIN_EVT, 
                        ZEM_REJOIN_PERIOD);
#endif
      }
    break;
#endif 
    }

}

/*********************************************************************
 * @fn      zclZEM_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclZEM_BasicResetCB( void )
{
  zclZEM_ResetAttributesToDefaultValues();
  
}

/*********************************************************************
 * @fn      zclSampleApp_BatteryWarningCB
 *
 * @brief   Called to handle battery-low situation.
 *
 * @param   voltLevel - level of severity
 *
 * @return  none
 */
void zclSampleApp_BatteryWarningCB( uint8 voltLevel )
{
  if ( voltLevel == VOLT_LEVEL_CAUTIOUS )
  {
    // Send warning message to the gateway and blink LED
  }
  else if ( voltLevel == VOLT_LEVEL_BAD )
  {
    // Shut down the system
  }
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclZEM_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclZEM_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclZEM_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclZEM_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclZEM_ProcessInConfigReportCmd( pInMsg );
      break;
      case ZCL_CMD_READ_REPORT_CFG:
      //zclZEM_ProcessInReadReportCfgCmd( pInMsg );
      break;
    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclZEM_ProcessInConfigReportRspCmd( pInMsg );
      break;
    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclZEM_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclZEM_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclZEM_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclZEM_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclZEM_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclZEM_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclZEM_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclZEM_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclZEM_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclZEM_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclZEM_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclZEM_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclZEM_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclZEM_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER




/*********************************************************************
 * @fn      zclZEM_AF_P2P
 *
 * @brief   点对点发送函数
 *
 * @param   destNwkAddr - 目标设备的网络地址
 * @param   cid - Cluster ID，后续课程将会详细讲解
 * @param   len - 数据长度
 * @param   data - 数据内容
 *
 * @return  none
 */
static void zclZEM_AF_P2P(
    uint16 destNwkAddr,
    uint16 cid,
    uint8 len,
    uint8 *data)
{
    afAddrType_t dstAddr; //寻址信息配置
    static uint8 transferId = 0;//传输ID，是数据包的标识符
    
    /* Destination */
    dstAddr.addrMode = afAddr16Bit;// 设置目标地址模式为16位网络地址，表示使用P2P的通信方式
    dstAddr.addr.shortAddr = destNwkAddr;//目标设备的网络地址
    dstAddr.endPoint = ZEM_ENDPOINT;//目标设备的端点号
    
    transferId++;
    
    AF_DataRequest(&dstAddr,
        &ZEM_AppEp,//应用程序端点描述符
        cid,
        len,
        data,
        &transferId,
        AF_DISCV_ROUTE,//进行路由扫描操作，用于建立发送数据报文的通信路径。关于这个参数，暂时保持例程默认的代码就可以了
        AF_DEFAULT_RADIUS);//指定最大的路由跳转级数
}

/*********************************************************************
 * @fn      zclZEM_AF_Broadcast
 *
 * @brief   广播发送函数
 *
 * @param   cid - Cluster ID
 * @param   len - 待发送数据的长度
 * @param   data - 待发送数据的内容
 *
 * @return  none
 */
static void zclZEM_AF_Broadcast(
    uint16 cid,
    uint8 len,
    uint8 *data)
{
    afAddrType_t dstAddr;
    static uint8 transferId = 0;
    
    /* Destination */
    dstAddr.addrMode = afAddrBroadcast;   // 使用广播模式
    dstAddr.addr.shortAddr = 0xFFFF; // 广播地址
    dstAddr.endPoint = ZEM_ENDPOINT; // 目标设备的端点号
    
    /* Transfer id */
    transferId++;
    
    /* Send */
    AF_DataRequest(&dstAddr,
        &ZEM_AppEp,//应用程序端点描述符
        cid,
        len,
        data,
        &transferId,
        AF_TX_OPTIONS_NONE,
        AF_DEFAULT_RADIUS );//指定了最大的路由跳转级数
}

/*********************************************************************
 * @fn      zclZEM_AF_Groupcast
 *
 * @brief   组播发送函数
 *
 * @param   groupId - 组ID
 * @param   cid - ClusterID，后续章节将会详细讲解
 * @param   len - 待发送数据的长度
 * @param   data - 待发送数据的内容
 *
 * @return  none
 */
static void zclZEM_AF_Groupcast(
    uint16 groupId,
    uint16 cid,
    uint8 len,
    uint8 *data)
{
    afAddrType_t dstAddr;
    static uint8 transferId = 0;
    
    /* Destination */
    dstAddr.addrMode = afAddrGroup;//使用组播通信模式
    dstAddr.addr.shortAddr = groupId;
    dstAddr.endPoint = ZEM_ENDPOINT;//组中的设备的端点号
    
    /* Transfer id */
    transferId++;
    
    /* Send */
    AF_DataRequest(&dstAddr,
        &ZEM_AppEp,//应用程序端点描述符
        cid,
        len,
        data,
        &transferId,
        AF_TX_OPTIONS_NONE,
        AF_DEFAULT_RADIUS );//指定最大的路由跳转级数，暂时可忽略
}

/*********************************************************************
 * @fn      zclZEM_AF_RxProc
 *
 * @brief   接收数据处理函数
 *
 * @param   MSGpkt - 接收到数据
 *
 * @return  none
 */
static void zclZEM_AF_RxProc(afIncomingMSGPacket_t *MSGpkt)
{
    /*计数器，记录接收到的点对点通信数据包个数*/
    static uint8 p2pCnt = 0;
    /*计数器，记录接收到的广播通信数据包个数*/
    static uint8 bcCnt = 0;
    /*计数器，记录接收到的组播通信数据包个数*/
    static uint8 gcCnt = 0;
    
    switch( MSGpkt->clusterId )  // 判断接收到的数据包的Cluster ID
    {
        case CLUSTER_P2P:
            p2pCnt++;  // 接收到P2P数据包，进行计数
            // 通过串口输出接收到的数据和计数器值
            {
                uint8 p2pMsg[64];
                osal_memcpy(p2pMsg, "P2P Data: ", 10);
                osal_memcpy(p2pMsg + 10, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
                osal_memcpy(p2pMsg + 10 + MSGpkt->cmd.DataLength, " Count: ", 8);
                p2pMsg[18 + MSGpkt->cmd.DataLength] = '0' + p2pCnt;
                p2pMsg[19 + MSGpkt->cmd.DataLength] = '\r';
                p2pMsg[20 + MSGpkt->cmd.DataLength] = '\n';
                HalUARTWrite(HAL_UART_PORT_0, p2pMsg, 21 + MSGpkt->cmd.DataLength);
            }
            break;
        case CLUSTER_BROADCAST:
            bcCnt++;  // 接收到广播数据包，进行计数
            // 通过串口输出接收到的数据和计数器值
            {
                uint8 bcMsg[64];
                osal_memcpy(bcMsg, "Broadcast Data: ", 16);
                osal_memcpy(bcMsg + 16, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
                osal_memcpy(bcMsg + 16 + MSGpkt->cmd.DataLength, " Count: ", 8);
                bcMsg[24 + MSGpkt->cmd.DataLength] = '0' + bcCnt;
                bcMsg[25 + MSGpkt->cmd.DataLength] = '\r';
                bcMsg[26 + MSGpkt->cmd.DataLength] = '\n';
                HalUARTWrite(HAL_UART_PORT_0, bcMsg, 27 + MSGpkt->cmd.DataLength);
            }
            break;
        case CLUSTER_GROUPCAST:
            gcCnt++;  // 接收到组播数据包，进行计数
            // 通过串口输出接收到的数据和计数器值
            {
                uint8 gcMsg[64];
                osal_memcpy(gcMsg, "Groupcast Data: ", 16);
                osal_memcpy(gcMsg + 16, MSGpkt->cmd.Data, MSGpkt->cmd.DataLength);
                osal_memcpy(gcMsg + 16 + MSGpkt->cmd.DataLength, " Count: ", 8);
                gcMsg[24 + MSGpkt->cmd.DataLength] = '0' + gcCnt;
                gcMsg[25 + MSGpkt->cmd.DataLength] = '\r';
                gcMsg[26 + MSGpkt->cmd.DataLength] = '\n';
                HalUARTWrite(HAL_UART_PORT_0, gcMsg, 27 + MSGpkt->cmd.DataLength);
            }
            break;
        default:
            break;
    }
}

/****************************************************************************
****************************************************************************/


