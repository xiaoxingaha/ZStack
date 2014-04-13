/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"
#include "MT_UART.h"   //add by 1305106

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"

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
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID,
};

// 简单设备描述符（描述一个zigbee设备节点）
const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;      // 节点描述符

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// 任务优先级
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

// 数据发送序列号
uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );   // 消息处理函数
void SampleApp_SendPeriodicMessage( void );     // 数据发送函数
void SampleApp_SendFlashMessage( uint16 flashTime );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
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
void SampleApp_Init( uint8 task_id )    // 任务初始化函数
{
    // 初始化应用层的任务ID号，也就是任务优先级（任务优先级由协议栈的操作系统OSAL分配）
    SampleApp_TaskID = task_id; 
    
    // 初始化应用设备的网络状态为 ZDO层中定义的初始化值，即 not connected to anything
    // 在设备初始化的时候一定要把他初始化为 什么状态都没有 \
       那么他就要去检测整个环境，看是否能重新建立或者加入存在的网络
    // DEV_INIT     Initialized - not connected to anything
    SampleApp_NwkState = DEV_INIT;
    
    // 发送数据包的序号初始化为0
    SampleApp_TransID = 0;        
  
    // Device hardware initialization can be added here or in main() (Zmain.c).
    // If the hardware is application specific - add it here.
    // If the hardware is other parts of the device add it in main().
    // 关于硬件初始化在哪添加的问题
    // 如果硬件是特殊用途，需要在此添加
    // 如果是其他的硬件初始化，就在主函数中添加
    
    
   #if defined ( BUILD_ALL_DEVICES )
    // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
    // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
    // together - if they are - we will start up a coordinator. Otherwise,
    // the device will start as a router.
    // 为了在形成网络过程中节省所需的设备，所有的路由设备可以通过 jump定义作为一个协调器。
    if ( readCoordinatorJumper() )
      zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
    else
      zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
  #endif // BUILD_ALL_DEVICES
  
  #if defined ( HOLD_AUTO_START )
    // HOLD_AUTO_START is a compile option that will surpress(抑制) ZDApp
    //  from starting the device and wait for the application to
    //  start the device.
    // 如果定义了HOLD_AUTO_START选项，则调用层的ZDOInitDevice，按照默认顺序 \
      网络中的第一个设备作为协调器，其他的设备作为子设备
    ZDOInitDevice(0);
  #endif

/**********************设置发送数据的方式和目的地址寻址方式***********************/    
    // Setup for the periodic message's destination address
    // Broadcast to everyone
    // 周期消息，广播发送
    // 为周期性发送消息设置目的地址，广播给每一个节点设备
    SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;    // 发送模式(广播)
    SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;     // 指定端点号 20
    SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;   // 指定目的网络地址为广播地址
  
    // Setup for the flash command's destination address - Group 1
    // 闪烁消息，发送到组
    // 为 group1中的 flash发送命令设置目的地址
    SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;   // 发送模式(组寻址)
    SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;    // 指定端点号 20
    // 在 sampleApp.h中已经定义了 #define SAMPLEAPP_FLASH_GROUP   0x0001
    SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;   // 组号 0x0001
/*******************************************END*******************************************/

/*****************************初始化终端节点描述符****************************************/
    // Fill out the endpoint description.(endpoint 简称EP)
    // 对终端节点描述符进行初始化
    // 定义本设备用来通信的 APS层端点描述符
    SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;     // EP描述符的EP号
    SampleApp_epDesc.task_id = &SampleApp_TaskID;       // EP描述符的任务ID号
    // sampleApp EP简单描述符
    SampleApp_epDesc.simpleDesc
              = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
    // 终端节点描述符等待请求，也就是延时策略
    SampleApp_epDesc.latencyReq = noLatencyReqs;
/**************************************END************************************************/
    
    // Register the endpoint description with the AF
    // 向AF层登记注册 EP描述符。登记后，才能使用OSAL提供的系统服务。
    /*
        AF层登记相应应用对象，要对该应用进行初始化并在 AF进行登记，告诉应用层有这么一个 
    EP已经可以使用，那么下层要是有关于该应用的信息或者应用要对下层做哪些操作，就自动得
    到下层的配合。其实在这个应用中，只是让 AF配合 SAMPLEAPP_PROFID / SAMPLEAPP_ENDPOINT
    这两个应用。那么通过什么呢，通过发送 OSAL SYS_EVENT_MSG消息中的（AF_INCOMING_CMD）
    事件到 SampleApp 任务 ID。
    */
    afRegister( &SampleApp_epDesc );
  
    // Register for all key events - This app will handle all key events
    // 登记注册可用的 OSAL或 HAL系统服务。
    // 另一种说法：登记注册所有的按键事件
    RegisterForKeys( SampleApp_TaskID );
    MT_UartRegisterTaskID( SampleApp_TaskID ); //add by 1305106

/**************************************设立一个新组***************************************************/    
    // By default, all devices start out in Group 1
    // 设立一个新组group 1，默认情况下，所有设备都在 Group 1中开始
    SampleApp_Group.ID = 0x0001;        // 设定组号
    
    /**************memcpy 函数*****************************************
    原型：extern void *memcpy( void *dst, void *src, unsigned int count )
    用法：#include <string.h>
    功能：由 src所指内存区域复制 count个字节到 dest所指内存区域
    说明：src 和 dest 所指内存区域不能重叠，函数返回指向 dest 指针
    */
    // Flash信息被发送到 group1,同样也只有在 group1的设备才能接收到这个信息，\
      设备启动时已经被设定为 group 1设备了，但是可以通过按 SW1退出/加入 group1
    osal_memcpy( SampleApp_Group.name, "Group 1", 7  );       // 设定组名
    aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );     // 把该组登记添加到APS中
/**********************************************END********************************************************/
    
    // 如果支持LCD，显示一串字符
  #if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
  #endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *         
 *          一般请求任务事件处理器，这个函数调用处理所有事件任务。事件包括时间、
 *          信息和所有其他使用者定义的事件。
 *          这个任务中一共存在 按键时间、接收数据和设备状态转换 3个事件。按键时间
 *          通过键盘实现相应的操作，接收数据事件是完成一次数据接收后对数据的处理。
 *
 * @param   task_id  - The OSAL assigned task ID.     操作系统分配的任务ID
 * @param   events - events to process.  This is a bit map and can    
 *                   contain more than one event.    事件的处理
 *
 * @return  none
 */
// 在 ZStack中，每个应用任务都通过 SampleApp_ProcessEvent() 函数来处理任务中的事件。 \
  一旦 SampleApp_TaskID任务的某个 OSAL事件发生，那么就可以通过调用 SampleApp_ProcessEvent() \
   函数来处理。在 SampleApp_ProcessEvent()中有一个事件处理循环，循环检测是哪个事件发生。\
     本例程中调用 uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )函数处理任务事件， \
       除了强制事件（一个任务事件 SYS_EVENT_MSG(0X8000),被保留必须通过 OSAL任务设计）之外， \
         任何一个 OSAL任务能被定义多达15个任务事件。
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )   // 任务处理函数
{
    // MSGpkt用于指向接收消息结构体的指针 
    afIncomingMSGPacket_t *MSGpkt;
    (void)task_id;  // Intentionally unreferenced parameter
    
    if ( events & SYS_EVENT_MSG )
    {
        // osal_msg_receive（）从消息队列上接收消息  
        MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
        while ( MSGpkt )
        {
            switch ( MSGpkt->hdr.event )
            {
              // 按键触发事件。
              // 判断是否是按键事件，如果键盘事件就调用键盘处理函数，关于按键子程序在本文中 \
                不再展开论述，因为其比较简单，就是通过按键实现了两个功能：通过按键触发实现 \
                  flash 发送模式通过按键实现加入或退出 group1
              // Received when a key is pressed
              case KEY_CHANGE:
                SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
                break;
      
              // 接收数据事件。
              // 接受到新数据的消息的ID是AF_INCOMING_MSG_CMD,这个宏是在协议栈中定义好的值为0x1A  
              // 接受到的是无线数据包  
              // Received when a messages is received (OTA) for this endpoint
              case AF_INCOMING_MSG_CMD:
                // 功能是完成对接受数据的处理 
                SampleApp_MessageMSGCB( MSGpkt );
                break;;
      
              // 设备状态变化事件。
              // 只要网络状态发生改变，那么通过 ZDO_STATE_CHANGE事件通知所有的任务。 \
                  注意：在这个例子中，一旦设备成功加入网络，是通过定时运行的方式运行的。\
                  一旦网络状态为加入“JOINED”，那么他可能不需要任何的认证操作就能绑定 \
                  其他设备，因为设置为自动发现并绑定的。
              // Received whenever the device changes state in the network
              case ZDO_STATE_CHANGE:
                SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                if ( (SampleApp_NwkState == DEV_ZB_COORD)
                    || (SampleApp_NwkState == DEV_ROUTER)
                    || (SampleApp_NwkState == DEV_END_DEVICE) )
                {
                  // 在一个时间间隔中周期性发送一个数据。
                  // 可以看到网络状态一旦改变（建立或加入网络），\
                      就调用了定时触发事件 SAMPLEAPP_SEND_PERIODIC_MSG_EVT
                  // Start sending the periodic message in a regular interval.
                  HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
                  osal_start_timerEx( SampleApp_TaskID,
                                    SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                    SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                }
                else
                {
                  // Device is no longer in the network
                }
                break;
      
              default:
                break;
          }
          // 接收到的消息处理完后，释放消息所占的存储空间 
          osal_msg_deallocate( (uint8 *)MSGpkt );    // Release the memory
          // 处理完一个消息后，再从消息队列里接受消息，然后对其进行相应处理，直到所有消息处理完
          // Next - if one is available
          MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );   
      }
      
      // 返回没有处理的事件
      return (events ^ SYS_EVENT_MSG);    // return unprocessed events
    }
  
    // 发送一个数据出去，这个任务是产生一个时间，通俗一点就是通过该函数实现周期性定时的功能，\
      这里调用了发送数据函数 SampleApp_SendPeriodicMessage()，而之后又调用了定时触发事件 \
      SAMPLEAPP_SEND_PERIODIC_MSG_EVT，所以该事件不停的在触发并发送数据。
    // Send a message out - This event is generated by a timer
    //  (setup in SampleApp_Init()).
    if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
    {
      SampleApp_SendPeriodicMessage();   // Send the periodic message  发送一个周期性信息
      // Setup to send message again in normal period (+ a little jitter)
      osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
          (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) ); 
      // return unprocessed events  ^代表取反的意思
      return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);     
    }
 
    /*
       这里检测事件是否为周期发送信息事件。因为在 SampleApp.h中定义了：#define 
    SAMPLEAPP_SEND_PERIODIC_MSG_EVT   0x0001 在这个应用中，调用了 osal_start_timerEx()
    函数来定时产生发送周期信息事件。而定时器的运行是设备一旦加入网络就不停地运行。
       从上面可以看到，用函数 SampleApp_SendPeriodicMessage()发送周期信息，而用函数 
    osal_start_timerEx( SampleApp_TaskID,
                                    SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                    SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
    来继续运行定时器定时发送这个周期信息。关于这个 osal_start_timerEx()的具体参数如下，
    第一个参数是任务的ID；第二个参数是说明事件到了，产生一个什么事件；第三个参数是需要定时
    的时间，这里就是发送周期信息的时间周期。
    */
    
    // 丢掉没有定义的事件
    // Discard unknown events
    return 0;   
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
    (void)shift;  // Intentionally unreferenced parameter
    
    if ( keys & HAL_KEY_SW_6 ) 
    {
      /* This key sends the Flash Command is sent to Group 1.
       * This device will not receive the Flash Command from this
       * device (even if it belongs to group 1).
       */
      SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
    }
  
    if ( keys & HAL_KEY_SW_2 )
    {
        /* The Flashr Command is sent to Group 1.
         * This key toggles this device in and out of group 1.
         * If this device doesn't belong to group 1, this application
         * will not receive the Flash command sent to group 1.
         */
        aps_Group_t *grp;
        grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
        if ( grp )
        {
          // Remove from the group
          aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
        }
        else
        {
          // Add to the flash group
          aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
        }
    }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *          信息的接收（适用与两种发送模式）
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *        
 * @param   none
 *
 * @return  none
 */
/*
    接收数据就是发送的过程，那么接收之后，在应用层有什么反应呢？最直观的反应就是会发送一个
AF_INCOMING_MSG_CMD消息事件。表示收到某个信息，然后在里面调用了收到信息的信息处理函数。
具体函数如下所示：
*/
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
    uint16 flashTime;
    unsigned char *buf;
    
    // 判断簇ID，因为前面已经对两种发送模式定义不同的簇ID，以此来区分收到的消息是通过 \
      哪种发送模式发来的，针对不同的消息就相当于收到了不同的命令，然后根据不同的命令 \
      做出不同的处理。
    switch ( pkt->clusterId )
    {
        // 周期性发送模式ID
        case SAMPLEAPP_PERIODIC_CLUSTERID:
          buf = pkt->cmd.Data;
          HalUARTWrite(0, buf, 8);
          HalUARTWrite(0,"\r\n", 2);
          break;
      
        // flash发送模式ID
        case SAMPLEAPP_FLASH_CLUSTERID:
          flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
          HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );   // 小灯闪烁
          break;
    }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *          信息的周期性发送
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
    // 下面这个 "~HELLO!~"是自己添加的要发送的数据
    char buf[]="~HELLO!~";
    
    /*
       这里详细说明一下关于要发送的目的地址 SampleApp_Periodic_DstAddr，如果我们预先定义为0xFFFF
    则表示发送方能向网络内的其他所有节点发送预先定义的数据（即这里虽然是个地址，但主要还是代表了
    广播发送这么一个发送模式），相反如果定义为 0x0000，则表示下面的节点均向协调器发送数据。
    */
    // 这里调用了 AF_DataRequest()函数用来发送数据。关于发送数据的具体过程就不做深入研究。\
      不外乎就是把数据从应用层传到网络层，再传到MAC层，再传到物理层，最后通过 OTA发送出去。
    AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,  // 目的地址，发送模式
                       SAMPLEAPP_PERIODIC_CLUSTERID,           // 周期性发送簇的ID
                       8,                      // len 要发送的数据长度
                       (unsigned char*)buf,    // uint8 *buf 指向发送数据缓冲的指针
                       &SampleApp_TransID,     // uint8 *transID 事务序列号指针。如果缓存发送，这个函数将增加这个数字
                       AF_DISCV_ROUTE,         // 发送选项。由一项或者几项或运算得到。
                       AF_DEFAULT_RADIUS );    // 最大的跳数，用默认值 AF_DEFAULT_RADIUS
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
    uint8 buffer[3];
    buffer[0] = (uint8)(SampleAppFlashCounter++);
    buffer[1] = LO_UINT16( flashTime );
    buffer[2] = HI_UINT16( flashTime );
  
    if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                         SAMPLEAPP_FLASH_CLUSTERID,
                         3,
                         buffer,
                         &SampleApp_TransID,
                         AF_DISCV_ROUTE,
                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
      // Error occurred in request to send.
    }
}
/*********************************************************************
*********************************************************************/