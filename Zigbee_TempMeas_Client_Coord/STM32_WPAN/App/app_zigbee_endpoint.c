/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_zigbee_endpoint.c #
  * Description        : Zigbee Application to manage endpoints and these clusters.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>

#include "app_common.h"
#include "app_conf.h"
#include "log_module.h"
#include "app_entry.h"
#include "app_zigbee.h"
#include "dbg_trace.h"
#include "ieee802154_enums.h"
#include "EPD_Display_Values.h"
#include "EPD_1in9.h"
#include "mcp_enums.h"

#include "stm32_lpm.h"
#include "stm32_rtos.h"
#include "stm32_timer.h"
#include "stm32_lpm_if.h"

#include "zigbee.h"
#include "zigbee.nwk.h"
#include "zigbee.security.h"

/* Private includes -----------------------------------------------------------*/
#include "zcl/zcl.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zcl/general/zcl.color.h"
#include "zcl/general/zcl.onoff.h"

/* USER CODE BEGIN PI */
#include "app_bsp.h"

#include "zigbee_plat.h"

/* USER CODE END PI */

/* Private defines -----------------------------------------------------------*/
#define APP_ZIGBEE_CHANNEL                12u
#define APP_ZIGBEE_CHANNEL_MASK           ( 1u << APP_ZIGBEE_CHANNEL )
#define APP_ZIGBEE_TX_POWER               ((int8_t) 10)    /* TX-Power is at +10 dBm. */

#define APP_ZIGBEE_ENDPOINT               17u
#define APP_ZIGBEE_PROFILE_ID             ZCL_PROFILE_HOME_AUTOMATION
#define APP_ZIGBEE_DEVICE_ID              ZCL_DEVICE_TEMPERATURE_SENSOR
#define APP_ZIGBEE_GROUP_ADDRESS          0x0001u

#define APP_ZIGBEE_CLUSTER_ID             ZCL_CLUSTER_MEAS_TEMPERATURE
#define APP_ZIGBEE_CLUSTER_NAME           "TempMeas Client"

/* MeasTemperature specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_TEMP_MIN               -4000
#define APP_ZIGBEE_TEMP_MAX               12500
#define APP_ZIGBEE_TEMP_TOLERANCE         50
/* USER CODE BEGIN MeasTemperature defines */
/* USER CODE END MeasTemperature defines */

/* Color Sensor specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_COLOR_ENDPOINT         18u
#define APP_ZIGBEE_COLOR_DEVICE_ID        ZCL_DEVICE_COLOR_CONTROLLER
#define APP_ZIGBEE_COLOR_CLUSTER_ID       ZCL_CLUSTER_COLOR_CONTROL
#define APP_ZIGBEE_COLOR_CLUSTER_NAME     "Color Client"

/* On/Off Light specific defines ------------------------------------------------*/
#define APP_ZIGBEE_ONOFF_ENDPOINT        16u
#define APP_ZIGBEE_ONOFF_DEVICE_ID       ZCL_DEVICE_ONOFF_LIGHT
#define APP_ZIGBEE_ONOFF_CLUSTER_ID      ZCL_CLUSTER_ONOFF
#define APP_ZIGBEE_ONOFF_CLUSTER_NAME    "On/Off Server"

/* Second On/Off Light specific defines ------------------------------------------------*/
#define APP_ZIGBEE_ONOFF2_ENDPOINT       15u
#define APP_ZIGBEE_ONOFF2_DEVICE_ID      ZCL_DEVICE_ONOFF_LIGHT
#define APP_ZIGBEE_ONOFF2_CLUSTER_ID     ZCL_CLUSTER_ONOFF
#define APP_ZIGBEE_ONOFF2_CLUSTER_NAME   "On/Off Server 2"
/* USER CODE BEGIN PD */
#define APP_ZIGBEE_STARTUP_FAIL_DELAY     500u

#define APP_ZIGBEE_TEMP_START             2500                /* +25C   */
#define APP_ZIGBEE_TEMPMEAS_UPDATE_PERIOD (uint32_t)( 15000u ) /* 15 seconds */

#define APP_ZIGBEE_APPLICATION_NAME       APP_ZIGBEE_CLUSTER_NAME
#define APP_ZIGBEE_APPLICATION_OS_NAME    "."

/* Display cycle timer defines */
#define APP_ZIGBEE_DISPLAY_CYCLE_PERIOD   (uint32_t)( 1500u ) /* 1.5 seconds - show each value */
#define CFG_TASK_ZIGBEE_APP_DISPLAY_CYCLE CFG_TASK_ZIGBEE_APP2
#define TASK_ZIGBEE_APP_DISPLAY_CYCLE_PRIORITY CFG_SEQ_PRIO_1

/* USER CODE BEGIN PD */

// -- Redefine task to better code read --
#define CFG_TASK_ZIGBEE_APP_SENSOR_READ         CFG_TASK_ZIGBEE_APP1
#define TASK_ZIGBEE_APP_SENSOR_READ_PRIORITY    CFG_SEQ_PRIO_1
#define TempMeasClient                    pstZbCluster[0]
#define ColorClient                       pstZbCluster[1]
#define OnOffServer                       pstZbCluster[2]
#define OnOffServer2                      pstZbCluster[3]

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private constants ---------------------------------------------------------*/
/* USER CODE BEGIN PC */

/* USER CODE END PC */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UTIL_TIMER_Object_t      stTimerUpdateMeasure;
static int16_t                  iTemperatureCurrent;
static uint8_t                  uiColorHueCurrent;
static uint8_t                  uiColorSatCurrent;
static uint8_t                  uiColorRCurrent = 255;    /* RGB Red value */
static uint8_t                  uiColorGCurrent = 0;      /* RGB Green value */
static uint8_t                  uiColorBCurrent = 0;      /* RGB Blue value */
static float                    fTemperatureCurrent = 0.0f; /* Temperature in degrees C */
static uint16_t                 serverShortAddr = ZB_NWK_ADDR_UNDEFINED;  // Will be set dynamically when a device joins
static uint8_t                  onoff_state = 0;  // Track on/off state for toggle operation

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static void APP_ZIGBEE_ApplicationTaskInit        ( void );
static void APP_ZIGBEE_TempMeasAttributeUpdate    ( void );
static void APP_ZIGBEE_ColorAttributeUpdate       ( void );
static void APP_ZIGBEE_DisplayCycleTask           ( void );
static void APP_ZIGBEE_HSVtoRGB                   ( uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b );
static void APP_ZIGBEE_TimerUpdateCallback        ( void * arg );
static void APP_ZIGBEE_TempMeasReadCallback       ( const struct ZbZclReadRspT *rsp, void *arg );
static void APP_ZIGBEE_ColorReadCallback          ( const struct ZbZclReadRspT *rsp, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOffOn     ( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOffOff    ( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOffToggle ( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2On    ( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2Off   ( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2Toggle( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg );

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Zigbee application initialization
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ApplicationInit(void)
{
  LOG_INFO_APP( "ZIGBEE Application Init" );

  /* Initialization of the Zigbee stack */
  APP_ZIGBEE_Init();

  /* Configure Application Form/Join parameters : Startup, Persistence and Start with/without Form/Join */
  stZigbeeAppInfo.eStartupControl = ZbStartTypeForm;
  stZigbeeAppInfo.bPersistNotification = false;
  stZigbeeAppInfo.bNwkStartup = true;

  /* USER CODE BEGIN APP_ZIGBEE_ApplicationInit */
  /* Initialization of used Tasks */
  APP_ZIGBEE_ApplicationTaskInit();

  /* USER CODE END APP_ZIGBEE_ApplicationInit */

  /* Initialize Zigbee stack layers */
  APP_ZIGBEE_StackLayersInit();
}

/**
 * @brief  Zigbee application start
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ApplicationStart( void )
{
  /* USER CODE BEGIN APP_ZIGBEE_ApplicationStart */
    /* Enable permit join for 120 seconds to allow routers to join */
  APP_ZIGBEE_PermitJoin( 120 );
  
  /* Update default Temperature */
  iTemperatureCurrent = APP_ZIGBEE_TEMP_START;

  /* Update default Color */
  uiColorHueCurrent = 0;  /* Red */
  uiColorSatCurrent = 254;  /* Full saturation */

  /* Display Extended & Short Address */
  LOG_INFO_APP( "Use Short Address : 0x%04X", ZbShortAddress( stZigbeeAppInfo.pstZigbee ) );
  LOG_INFO_APP( "%s ready to work !", APP_ZIGBEE_APPLICATION_NAME );

  /* Start periodic Sensor Measure */
  UTIL_TIMER_Start( &stTimerUpdateMeasure );

  /* USER CODE END APP_ZIGBEE_ApplicationStart */

#if ( CFG_LPM_LEVEL != 0)
  /* Authorize LowPower now */
  UTIL_LPM_SetMaxMode( 1 << CFG_LPM_APP, UTIL_LPM_MAX_MODE );
#endif /* CFG_LPM_LEVEL */
}


/**
 * @brief  Configure Zigbee application endpoints
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ConfigEndpoints(void)
{
  struct ZbApsmeAddEndpointReqT   stRequest;
  struct ZbApsmeAddEndpointConfT  stConfig;
  /* USER CODE BEGIN APP_ZIGBEE_ConfigEndpoints1 */

  /* USER CODE END APP_ZIGBEE_ConfigEndpoints1 */

  /* Add EndPoint */
  LOG_INFO_APP( "Adding Endpoint %d...", APP_ZIGBEE_ENDPOINT );
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  assert( stConfig.status == ZB_STATUS_SUCCESS );
  LOG_INFO_APP( "Endpoint %d added", APP_ZIGBEE_ENDPOINT );

  /* Add TempMeas Client Cluster */
  LOG_INFO_APP( "Allocating TempMeas Client..." );
  stZigbeeAppInfo.TempMeasClient = ZbZclTempMeasClientAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT );
  assert( stZigbeeAppInfo.TempMeasClient != NULL );
  LOG_INFO_APP( "TempMeas Client allocated" );
  
  LOG_INFO_APP( "Registering TempMeas Endpoint..." );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.TempMeasClient ) == false )
  {
    LOG_ERROR_APP( "Error during TempMeas Client Endpoint Register." );
  }
  LOG_INFO_APP( "TempMeas Endpoint registered" );

  /* Add Color Endpoint */
  LOG_INFO_APP( "Adding Color Endpoint %d...", APP_ZIGBEE_COLOR_ENDPOINT );
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_COLOR_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_COLOR_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  assert( stConfig.status == ZB_STATUS_SUCCESS );
  LOG_INFO_APP( "Color Endpoint %d added", APP_ZIGBEE_COLOR_ENDPOINT );

  /* Add Color Client Cluster */
  LOG_INFO_APP( "Allocating Color Client..." );
  stZigbeeAppInfo.ColorClient = ZbZclColorClientAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_COLOR_ENDPOINT );
  if ( stZigbeeAppInfo.ColorClient == NULL ) {
    LOG_ERROR_APP( "Color Client alloc failed" );
  } else {
    LOG_INFO_APP( "Color Client allocated" );
  }
  
  if ( stZigbeeAppInfo.ColorClient != NULL ) {
    LOG_INFO_APP( "Registering Color Endpoint..." );
    if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.ColorClient ) == false ) {
      LOG_ERROR_APP( "Error during Color Client Endpoint Register." );
    } else {
      LOG_INFO_APP( "Color Endpoint registered" );
    }
  }

  /* Add On/Off Light Endpoint */
  LOG_INFO_APP( "Adding On/Off Endpoint %d...", APP_ZIGBEE_ONOFF_ENDPOINT );
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_ONOFF_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_ONOFF_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  assert( stConfig.status == ZB_STATUS_SUCCESS );
  LOG_INFO_APP( "On/Off Endpoint %d added", APP_ZIGBEE_ONOFF_ENDPOINT );

  /* Add On/Off Server Cluster */
  LOG_INFO_APP( "Allocating On/Off Server..." );
  struct ZbZclOnOffServerCallbacksT onoffCallbacks = {
    .on = APP_ZIGBEE_OnOffOn,
    .off = APP_ZIGBEE_OnOffOff,
    .toggle = APP_ZIGBEE_OnOffToggle
  };
  stZigbeeAppInfo.OnOffServer = ZbZclOnOffServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ONOFF_ENDPOINT, &onoffCallbacks, NULL );
  if ( stZigbeeAppInfo.OnOffServer == NULL ) {
    LOG_ERROR_APP( "On/Off Server alloc failed" );
  } else {
    LOG_INFO_APP( "On/Off Server allocated" );
  }
  
  if ( stZigbeeAppInfo.OnOffServer != NULL ) {
    LOG_INFO_APP( "Registering On/Off Endpoint..." );
    if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.OnOffServer ) == false ) {
      LOG_ERROR_APP( "Error during On/Off Server Endpoint Register." );
    } else {
      LOG_INFO_APP( "On/Off Endpoint registered" );
    }
  }

  /* Add Second On/Off Light Endpoint */
  LOG_INFO_APP( "Adding Second On/Off Endpoint %d...", APP_ZIGBEE_ONOFF2_ENDPOINT );
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_ONOFF2_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_ONOFF2_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  assert( stConfig.status == ZB_STATUS_SUCCESS );
  LOG_INFO_APP( "Second On/Off Endpoint %d added", APP_ZIGBEE_ONOFF2_ENDPOINT );

  /* Add Second On/Off Server Cluster */
  LOG_INFO_APP( "Allocating Second On/Off Server..." );
  struct ZbZclOnOffServerCallbacksT onoff2Callbacks = {
    .on = APP_ZIGBEE_OnOff2On,
    .off = APP_ZIGBEE_OnOff2Off,
    .toggle = APP_ZIGBEE_OnOff2Toggle
  };
  stZigbeeAppInfo.OnOffServer2 = ZbZclOnOffServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ONOFF2_ENDPOINT, &onoff2Callbacks, NULL );
  if ( stZigbeeAppInfo.OnOffServer2 == NULL ) {
    LOG_ERROR_APP( "Second On/Off Server alloc failed" );
  } else {
    LOG_INFO_APP( "Second On/Off Server allocated" );
  }
  
  if ( stZigbeeAppInfo.OnOffServer2 != NULL ) {
    LOG_INFO_APP( "Registering Second On/Off Endpoint..." );
    if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.OnOffServer2 ) == false ) {
      LOG_ERROR_APP( "Error during Second On/Off Server Endpoint Register." );
    } else {
      LOG_INFO_APP( "Second On/Off Endpoint registered" );
    }
  }

  /* USER CODE BEGIN APP_ZIGBEE_ConfigEndpoints2 */

  /* USER CODE END APP_ZIGBEE_ConfigEndpoints2 */
}

/**
 * @brief  Set Group Addressing mode (if used)
 * @param  None
 * @retval 'true' if Group Address used else 'false'.
 */
bool APP_ZIGBEE_ConfigGroupAddr( void )
{
  struct ZbApsmeAddGroupReqT  stRequest;
  struct ZbApsmeAddGroupConfT stConfig;

  memset( &stRequest, 0, sizeof( stRequest ) );

  // Add group address for TempMeas endpoint
  stRequest.endpt = APP_ZIGBEE_ENDPOINT;
  stRequest.groupAddr = APP_ZIGBEE_GROUP_ADDRESS;
  ZbApsmeAddGroupReq( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );

  // Add group address for OnOff endpoint
  stRequest.endpt = APP_ZIGBEE_ONOFF_ENDPOINT;
  stRequest.groupAddr = APP_ZIGBEE_GROUP_ADDRESS;
  ZbApsmeAddGroupReq( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );

  // Add group address for Second OnOff endpoint
  stRequest.endpt = APP_ZIGBEE_ONOFF2_ENDPOINT;
  stRequest.groupAddr = APP_ZIGBEE_GROUP_ADDRESS;
  ZbApsmeAddGroupReq( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );

  return true;
}

/**
 * @brief  Return the Startup Configuration
 * @param  pstConfig  Configuration structure to fill
 * @retval None
 */
void APP_ZIGBEE_GetStartupConfig( struct ZbStartupT * pstConfig )
{
  /* Attempt to join a zigbee network */
  ZbStartupConfigGetProDefaults( pstConfig );

  /* Using the default HA preconfigured Link Key */
  memcpy( pstConfig->security.preconfiguredLinkKey, sec_key_ha, ZB_SEC_KEYSIZE );

  /* Setting up additional startup configuration parameters */
  pstConfig->startupControl = stZigbeeAppInfo.eStartupControl;
  pstConfig->channelList.count = 1;
  pstConfig->channelList.list[0].page = 0;
  pstConfig->channelList.list[0].channelMask = APP_ZIGBEE_CHANNEL_MASK;

  /* Set the TX-Power */
  if ( APP_ZIGBEE_SetTxPower( APP_ZIGBEE_TX_POWER ) == false )
  {
    LOG_ERROR_APP( "Switching to %d dB failed.", APP_ZIGBEE_TX_POWER );
    return;
  }

  /* USER CODE BEGIN APP_ZIGBEE_GetStartupConfig */

  /* USER CODE END APP_ZIGBEE_GetStartupConfig */
}

/**
 * @brief  Manage a New Device on Network (called only if Coord or Router).
 * @param  iShortAddress      Short Address of new Device
 * @param  dlExtendedAddress  Extended Address of new Device
 * @param  cCapability        Capability of new Device
 * @retval Group Address
 */
void APP_ZIGBEE_SetNewDevice( uint16_t iShortAddress, uint64_t dlExtendedAddress, uint8_t cCapability )
{
  LOG_INFO_APP( "New Device (%d) on Network : with Extended ( " LOG_DISPLAY64() " ) and Short ( 0x%04X ) Address.", cCapability, LOG_NUMBER64( dlExtendedAddress ), iShortAddress );

  /* Keep the joined device short address for future attribute read requests */
  serverShortAddr = iShortAddress;
  LOG_INFO_APP( "Server short address updated to 0x%04X", serverShortAddr );

  /* USER CODE BEGIN APP_ZIGBEE_SetNewDevice */

  /* USER CODE END APP_ZIGBEE_SetNewDevice */
}

/**
 * @brief  Print application information to the console
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_PrintApplicationInfo(void)
{
  LOG_INFO_APP( "**********************************************************" );
  LOG_INFO_APP( "Network config : CENTRALIZED COORDINATOR" );

  /* USER CODE BEGIN APP_ZIGBEE_PrintApplicationInfo1 */
  LOG_INFO_APP( "Application Flashed : Zigbee %s %s", APP_ZIGBEE_APPLICATION_NAME, APP_ZIGBEE_APPLICATION_OS_NAME );

  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo1 */
  LOG_INFO_APP( "Channel used: %d.", APP_ZIGBEE_CHANNEL );

  APP_ZIGBEE_PrintGenericInfo();

  LOG_INFO_APP( "Clusters allocated are:" );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER_NAME, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_COLOR_CLUSTER_NAME, APP_ZIGBEE_COLOR_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_ONOFF_CLUSTER_NAME, APP_ZIGBEE_ONOFF_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_ONOFF2_CLUSTER_NAME, APP_ZIGBEE_ONOFF2_ENDPOINT );

  /* USER CODE BEGIN APP_ZIGBEE_PrintApplicationInfo2 */

  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo2 */

  LOG_INFO_APP( "**********************************************************" );
}
/**
 * @brief  Convert HSV to RGB
 * @param  hue: Hue value (0-254)
 * @param  sat: Saturation value (0-254)
 * @param  r: Pointer to red value (0-255)
 * @param  g: Pointer to green value (0-255)
 * @param  b: Pointer to blue value (0-255)
 * @retval None
 */
static void APP_ZIGBEE_HSVtoRGB( uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b )
{
  float h = (float)hue / 255.0f * 360.0f;
  float s = (float)sat / 255.0f;
  float v = 1.0f;  // Full brightness

  int i = (int)(h / 60.0f) % 6;
  float f = (h / 60.0f) - i;
  float p = v * (1.0f - s);
  float q = v * (1.0f - f * s);
  float t = v * (1.0f - (1.0f - f) * s);

  float rf, gf, bf;

  switch(i) {
    case 0: rf = v; gf = t; bf = p; break;
    case 1: rf = q; gf = v; bf = p; break;
    case 2: rf = p; gf = v; bf = t; break;
    case 3: rf = p; gf = q; bf = v; break;
    case 4: rf = t; gf = p; bf = v; break;
    case 5: rf = v; gf = p; bf = q; break;
    default: rf = 0; gf = 0; bf = 0; break;
  }

  *r = (uint8_t)(rf * 255.0f);
  *g = (uint8_t)(gf * 255.0f);
  *b = (uint8_t)(bf * 255.0f);
}
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/**
 * @brief  Zigbee application Task initialization
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ApplicationTaskInit( void )
{
  /* Task that Update Sensor AttributeValues */
  UTIL_SEQ_RegTask( 1U << CFG_TASK_ZIGBEE_APP_SENSOR_READ, UTIL_SEQ_RFU, APP_ZIGBEE_TempMeasAttributeUpdate );
  
  /* Create timer to get the measure of temperature sensor */
  UTIL_TIMER_Create( &stTimerUpdateMeasure, APP_ZIGBEE_TEMPMEAS_UPDATE_PERIOD, UTIL_TIMER_PERIODIC, APP_ZIGBEE_TimerUpdateCallback, NULL );
  
  /* Register Display Cycle Task */
  UTIL_SEQ_RegTask( 1U << CFG_TASK_ZIGBEE_APP_DISPLAY_CYCLE, UTIL_SEQ_RFU, APP_ZIGBEE_DisplayCycleTask );
}
  
/**
 * @brief  Management of the SW1 button. Start periodic measure of Temperature.
 * @param  None
 * @retval None
 */
void APP_BSP_Button1Action(void)
{
  UTIL_TIMER_Start( &stTimerUpdateMeasure ); 
}


/**
 * @brief  Management of the SW2 button. Stop periodic measure of Temperature.
 * @param  None
 * @retval None
 */
void APP_BSP_Button2Action(void)
{
  UTIL_TIMER_Stop( &stTimerUpdateMeasure );
}

/**
 * @brief  Management of the UpdateTimer Callback to launch a Temperature Read Task
 * @param  arg    Argument 
 * @retval None
 */
static void APP_ZIGBEE_TimerUpdateCallback( void * arg )
{
  UTIL_SEQ_SetTask( 1u << CFG_TASK_ZIGBEE_APP_SENSOR_READ, TASK_ZIGBEE_APP_SENSOR_READ_PRIORITY );
}


/**
 * @brief  Read Temp Measure attribute from server
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_TempMeasAttributeUpdate( void )
{
  enum ZclStatusCodeT eStatus;

  if ( serverShortAddr == ZB_NWK_ADDR_UNDEFINED )
  {
    LOG_INFO_APP( "[TEMP MEAS] No server short address known yet; skipping read." );
    return;
  }
  
  LOG_INFO_APP( "[TEMP MEAS] Reading TempMeasure from server 0x%04X", serverShortAddr );
  
  struct ZbZclReadReqT req;
  memset(&req, 0, sizeof(req));
  req.dst.mode = ZB_APSDE_ADDRMODE_SHORT;
  req.dst.endpoint = APP_ZIGBEE_ENDPOINT;
  req.dst.nwkAddr = serverShortAddr;
  req.count = 1;
  req.attr[0] = ZCL_TEMP_MEAS_ATTR_MEAS_VAL;
  
  eStatus = ZbZclReadReq( stZigbeeAppInfo.TempMeasClient, &req, APP_ZIGBEE_TempMeasReadCallback, NULL );
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[TEMP MEAS] Read request error (0x%02X)", eStatus );
  }
  
  /* Read Color attributes */
  APP_ZIGBEE_ColorAttributeUpdate();
}


/**
 * @brief  Read Color attributes from server
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ColorAttributeUpdate( void )
{
  if ( stZigbeeAppInfo.ColorClient == NULL ) {
    return;
  }

  if ( serverShortAddr == ZB_NWK_ADDR_UNDEFINED )
  {
    LOG_INFO_APP( "[COLOR] No server short address known yet; skipping read." );
    return;
  }

  enum ZclStatusCodeT eStatus;
  
  LOG_INFO_APP( "[COLOR] Reading Color from server 0x%04X", serverShortAddr );
  
  struct ZbZclReadReqT req;
  memset(&req, 0, sizeof(req));
  req.dst.mode = ZB_APSDE_ADDRMODE_SHORT;
  req.dst.endpoint = APP_ZIGBEE_COLOR_ENDPOINT;
  req.dst.nwkAddr = serverShortAddr;
  req.count = 2;
  req.attr[0] = ZCL_COLOR_ATTR_CURRENT_HUE;
  req.attr[1] = ZCL_COLOR_ATTR_CURRENT_SAT;
  
  eStatus = ZbZclReadReq( stZigbeeAppInfo.ColorClient, &req, APP_ZIGBEE_ColorReadCallback, NULL );
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[COLOR] Read request error (0x%02X)", eStatus );
  }
}

/**
 * @brief  Callback for TempMeas read response
 * @param  read: Read response structure
 * @param  arg: Argument
 * @retval None
 */
static void APP_ZIGBEE_TempMeasReadCallback( const struct ZbZclReadRspT *rsp, void *arg )
{
  if (rsp->status != ZCL_STATUS_SUCCESS) {
    LOG_ERROR_APP( "[TEMP MEAS] Read failed (0x%02X)", rsp->status );
    return;
  }
  
  if (rsp->count > 0 && rsp->attr[0].status == ZCL_STATUS_SUCCESS) {
    int16_t temp = (int16_t)(rsp->attr[0].value[0] | (rsp->attr[0].value[1] << 8));
    iTemperatureCurrent = temp;
    fTemperatureCurrent = (float)iTemperatureCurrent / 100.0f;  /* Convert from centidegrees to degrees */
    uint8_t cTempAP = (uint8_t)(iTemperatureCurrent % 100);
    int16_t iTempBP = (int16_t)(iTemperatureCurrent / 100);
    char szText[10];
    snprintf( szText, sizeof(szText), "%d.%02d", iTempBP, cTempAP );
    LOG_INFO_APP( "[TEMP MEAS] Received TempMeasure : %s C", szText );
    APP_LED_TOGGLE(LED_WORK);
  } else {
    LOG_ERROR_APP( "[TEMP MEAS] Attribute read error" );
  }
}

/**
 * @brief  Callback for Color read response
 * @param  read: Read response structure
 * @param  arg: Argument
 * @retval None
 */
static void APP_ZIGBEE_ColorReadCallback( const struct ZbZclReadRspT *rsp, void *arg )
{
  if (rsp->status != ZCL_STATUS_SUCCESS) {
    LOG_ERROR_APP( "[COLOR] Read failed (0x%02X)", rsp->status );
    return;
  }
  
  for (unsigned int i = 0; i < rsp->count; i++) {
    if (rsp->attr[i].status == ZCL_STATUS_SUCCESS) {
      if (rsp->attr[i].attrId == ZCL_COLOR_ATTR_CURRENT_HUE) {
        uiColorHueCurrent = rsp->attr[i].value[0];
      } else if (rsp->attr[i].attrId == ZCL_COLOR_ATTR_CURRENT_SAT) {
        uiColorSatCurrent = rsp->attr[i].value[0];
      }
    }
  }
  
  LOG_INFO_APP( "[COLOR] Received Color : Hue=%d, Sat=%d", uiColorHueCurrent, uiColorSatCurrent );
  
  uint8_t r, g, b;
  APP_ZIGBEE_HSVtoRGB( uiColorHueCurrent, uiColorSatCurrent, &r, &g, &b );
  uiColorRCurrent = r;
  uiColorGCurrent = g;
  uiColorBCurrent = b;
  LOG_INFO_APP( "[COLOR] RGB equivalent : R=%d, G=%d, B=%d", r, g, b );
  
  /* Trigger the task to display temperature and RGB sequence */
  UTIL_SEQ_SetTask( 1u << CFG_TASK_ZIGBEE_APP_DISPLAY_CYCLE, TASK_ZIGBEE_APP_DISPLAY_CYCLE_PRIORITY );
}

/**
 * @brief  Task to display temperature and RGB sequence
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_DisplayCycleTask( void )
{
  LOG_INFO_APP("E-paper: APP_ZIGBEE_DisplayCycleTask called");
  EPD_Display_Sequence(fTemperatureCurrent, uiColorRCurrent, uiColorGCurrent, uiColorBCurrent);
  LOG_INFO_APP("E-paper: APP_ZIGBEE_DisplayCycleTask completed");
}

/**
 * @brief  Handle On/Off server - On command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOffOn( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF] ON command received" );
  APP_LED_ON(LED_WORK);
  onoff_state = 1;
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, onoff_state );
  return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Handle On/Off server - Off command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOffOff( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF] OFF command received" );
  APP_LED_OFF(LED_WORK);
  onoff_state = 0;
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, onoff_state );
  return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Handle On/Off server - Toggle command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOffToggle( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF] TOGGLE command received" );
  onoff_state = !onoff_state;
  if (onoff_state) {
    APP_LED_ON(LED_WORK);
  } else {
    APP_LED_OFF(LED_WORK);
  }
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, onoff_state );
  return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Handle Second On/Off server - On command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2On( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF 2] ON command received" );
  APP_LED_ON(LED_GREEN);
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, 1 );
  return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Handle Second On/Off server - Off command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2Off( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF 2] OFF command received" );
  APP_LED_OFF(LED_GREEN);
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, 0 );
  return ZCL_STATUS_SUCCESS;
}

/**
 * @brief  Handle Second On/Off server - Toggle command
 * @param  cluster: Cluster pointer
 * @param  srcInfo: Source information
 * @param  arg: Argument
 * @retval ZCL status code
 */
static enum ZclStatusCodeT APP_ZIGBEE_OnOff2Toggle( struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg )
{
  LOG_INFO_APP( "[ON/OFF 2] TOGGLE command received" );
  static uint8_t onoff2_state = 0;
  onoff2_state = !onoff2_state;
  if (onoff2_state) {
    APP_LED_ON(LED_GREEN);
  } else {
    APP_LED_OFF(LED_GREEN);
  }
  /* Update ZCL On/Off attribute */
  ZbZclAttrIntegerWrite( cluster, ZCL_ONOFF_ATTR_ONOFF, onoff2_state );
  return ZCL_STATUS_SUCCESS;
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
