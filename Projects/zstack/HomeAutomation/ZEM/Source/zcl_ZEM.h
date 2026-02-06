#ifndef ZCL_ZEM_H
#define ZCL_ZEM_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"

/*********************************************************************
 * CONSTANTS
 */
#define ZEM_ENDPOINT            8 
  
#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define ZEM_TEMP_SEND_EVT       0x0001
#define ZEM_END_DEVICE_REJOIN_EVT             0x0002  
  
// UI Events
#define ZEM_LCD_AUTO_UPDATE_EVT        0x0010  
#define ZEM_KEY_AUTO_REPEAT_EVT        0x0020  
#define ZEM_PERIODIC_EVT               0x0040  
#define ZEMAPP_EVT                     0x0080  

// Communication events and periods (从小到大排序)
#define ZEM_REJOIN_EVT                 0x0100  // 终端设备重新加入事件
#define ZEM_P2P_EVT                    0x0200  // 点对点事件
#define ZEM_RETRY_FORMATION_EVT        0x0400  // 协调器网络形成重试事件
#define ZEM_GROUPCAST_EVT              0x0800  // 组播事件
#define ZEM_BROADCAST_EVT              0x1000  // 广播事件

// Event periods
#define ZEM_REJOIN_PERIOD              1000    // 重新加入间隔：1000ms(1秒)
#define ZEM_P2P_PERIOD                 1000    // 点对点间隔：1000ms(2秒)
#define ZEM_RETRY_FORMATION_PERIOD     5000    // 重试间隔：5000ms(5秒)
#define ZEM_GROUPCAST_PERIOD           1000    // 组播间隔：1000ms(3秒)
#define ZEM_BROADCAST_PERIOD           1000    // 广播间隔：1000ms(5秒)

/* 协调器 */
#ifdef ZDO_COORDINATOR
  // 协调器专用事件定义
/* 路由器或者终端 */
#else
  // 终端设备专用事件定义
#endif

// Device version and flags
#define ZEM_DEVICE_VERSION            0
#define ZEM_FLAGS                     0
#define GROUP_ID                       21  // Default group ID
#define CLUSTER_P2P             0
#define CLUSTER_BROADCAST       1
#define CLUSTER_GROUPCAST       2

// UART Configuration
#define ZCLZEM_UART_BUF_LEN           64

// Function Prototypes
static void zclZEM_InitUart(void);
static void zclZEM_UartCB(uint8 port, uint8 event);
static void zclZEM_ReadAndSendMAC(void);
static void zclZEM_SendMACToCoordinator(void);  
  
#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY 10000

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclZEM_SimpleDesc;

extern CONST zclAttrRec_t zclZEM_Attrs[];

extern CONST uint8 zclZEM_NumAttributes;

extern uint8  zclZEM_OnOff;

extern uint16 zclZEM_IdentifyTime;

// Temperature Measurement Cluster
extern int16 zclZEM_MeasuredValue;
extern const int16 zclZEM_MinMeasuredValue;
extern const uint16 zclZEM_MaxMeasuredValue;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclZEM_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclZEM_event_loop( byte task_id, UINT16 events );
   
/*
 *  Reset all writable attributes to their default values.
 */
extern void zclZEM_ResetAttributesToDefaultValues(void);

/*
 * 点对点发送函数
 */
static void zclZEM_AF_P2P(
    uint16 destNwkAddr,
    uint16 cid,
    uint8 len,
    uint8 *data);

/*
 * 广播发送函数
 */
static void zclZEM_AF_Broadcast(
    uint16 cid,
    uint8 len,
    uint8 *data);

/*
 * 组播发送函数
 */
static void zclZEM_AF_Groupcast(
    uint16 groupId,
    uint16 cid,
    uint8 len,
    uint8 *data);

/*
 * 接收数据处理函数
 */
static void zclZEM_AF_RxProc(afIncomingMSGPacket_t *MSGpkt);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_ZEM_H */
