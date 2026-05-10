/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_zigbee_endpoint.c
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

/* USER CODE BEGIN PI */
#include "app_bsp.h"
#include "dht11.h"

/* Used to simulate a Temperature Sensor */
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
#define APP_ZIGBEE_CLUSTER_NAME           "TempMeas Server"

/* MeasTemperature specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_TEMP_MIN               -4000
#define APP_ZIGBEE_TEMP_MAX               12500
#define APP_ZIGBEE_TEMP_TOLERANCE         50
/* USER CODE BEGIN MeasTemperature defines */
/* USER CODE END MeasTemperature defines */

/* Color Sensor specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_COLOR_ENDPOINT         18u
#define APP_ZIGBEE_COLOR_DEVICE_ID        ZCL_DEVICE_EXTENDED_COLOR_LIGHT
#define APP_ZIGBEE_COLOR_CLUSTER_ID       ZCL_CLUSTER_COLOR_CONTROL
#define APP_ZIGBEE_COLOR_CLUSTER_NAME     "Color Control Server"

/* USER CODE BEGIN PD */
#define APP_ZIGBEE_STARTUP_FAIL_DELAY     500u

#define APP_ZIGBEE_TEMP_START             2500                /* +25C   */
#define APP_ZIGBEE_TEMPMEAS_UPDATE_PERIOD (uint32_t)( 500u ) /* 500ms */
#define APP_ZIGBEE_REPORT_MIN_INTERVAL    1u    /* seconds */
#define APP_ZIGBEE_REPORT_MAX_INTERVAL    5u    /* seconds */

#define APP_ZIGBEE_APPLICATION_NAME       APP_ZIGBEE_CLUSTER_NAME
#define APP_ZIGBEE_APPLICATION_OS_NAME    "."

// -- Redefine task to better code read --
#define CFG_TASK_ZIGBEE_APP_SENSOR_READ         CFG_TASK_ZIGBEE_APP1
#define TASK_ZIGBEE_APP_SENSOR_READ_PRIORITY    CFG_SEQ_PRIO_1

/* USER CODE END PD */

// -- Redefine Clusters to better code read --
#define TempMeasServer                    pstZbCluster[0]
#define ColorServer                       pstZbCluster[1]

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static void APP_ZIGBEE_ApplicationTaskInit        ( void );
static void APP_ZIGBEE_TempMeasAttributeUpdate    ( void );
static void APP_ZIGBEE_ColorAttributeUpdate       ( void );
static void APP_ZIGBEE_SendReport                  ( struct ZbZclClusterT *cluster, const char *clusterName );
static void APP_ZIGBEE_HSVtoRGB                   ( uint8_t hue, uint8_t sat, uint8_t *r, uint8_t *g, uint8_t *b );
static void APP_ZIGBEE_TimerUpdateCallback        ( void * arg );

/* Color control callbacks */
static enum ZclStatusCodeT APP_ZIGBEE_ColorCbMoveToHue(struct ZbZclClusterT *cluster,
    struct ZbZclColorClientMoveToHueReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);

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
  stZigbeeAppInfo.eStartupControl = ZbStartTypeJoin;
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
  /* Update default Temperature */
  iTemperatureCurrent = APP_ZIGBEE_TEMP_START;

  /* Update default Color */
  uiColorHueCurrent = 0;  /* Red */
  uiColorSatCurrent = 254;  /* Full saturation */

  /* Display Extended & Short Address */
  LOG_INFO_APP( "Use Short Address : 0x%04X", ZbShortAddress( stZigbeeAppInfo.pstZigbee ) );
  LOG_INFO_APP( "%s ready to work !", APP_ZIGBEE_APPLICATION_NAME );

  /* Check DHT11 sensor availability */
  if (DHT11_IsAvailable())
  {
    LOG_INFO_APP( "[DHT11] Senzor detekován a odpovídá - PŘIPRAVEN" );
  }
  else
  {
    LOG_WARNING_APP( "[DHT11] Senzor NENÍ detekován - použit náhodný generátor" );
  }

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

  /* Add TempMeas Server Cluster */
  LOG_INFO_APP( "Allocating TempMeas Server..." );
  stZigbeeAppInfo.TempMeasServer = ZbZclTempMeasServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, APP_ZIGBEE_TEMP_MIN, APP_ZIGBEE_TEMP_MAX, APP_ZIGBEE_TEMP_TOLERANCE );
  assert( stZigbeeAppInfo.TempMeasServer != NULL );
  LOG_INFO_APP( "TempMeas Server allocated" );
  
  LOG_INFO_APP( "Registering TempMeas Endpoint..." );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.TempMeasServer ) == false )
  {
    LOG_ERROR_APP( "Error during TempMeas Server Endpoint Register." );
  }
  LOG_INFO_APP( "TempMeas Endpoint registered" );

  /* Configure default reporting for the Temperature Measurement cluster */
  {
    enum ZclStatusCodeT eStatus;
    eStatus = ZbZclAttrReportConfigDefault(stZigbeeAppInfo.TempMeasServer,
        ZCL_TEMP_MEAS_ATTR_MEAS_VAL, APP_ZIGBEE_REPORT_MIN_INTERVAL, APP_ZIGBEE_REPORT_MAX_INTERVAL, NULL);
    if ( eStatus != ZCL_STATUS_SUCCESS )
    {
      LOG_ERROR_APP( "TempMeas report config default failed (0x%02X)", eStatus );
    }
    else
    {
      LOG_INFO_APP( "TempMeas report defaults configured" );
    }
  }

  /* Add Color Endpoint */
  LOG_INFO_APP( "Adding Color Endpoint %d...", APP_ZIGBEE_COLOR_ENDPOINT );
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_COLOR_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_COLOR_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  if (stConfig.status != ZB_STATUS_SUCCESS) {
    LOG_ERROR_APP( "Failed to add Color Endpoint (status=0x%02X)", stConfig.status );
  } else {
    LOG_INFO_APP( "Color Endpoint %d added", APP_ZIGBEE_COLOR_ENDPOINT );

    /* Add Color Server Cluster */
    LOG_INFO_APP( "Allocating Color Server..." );
    struct ZbColorClusterConfig colorConfig;
    memset(&colorConfig, 0, sizeof(colorConfig));
    colorConfig.capabilities = (uint8_t)(ZCL_COLOR_CAP_HS | ZCL_COLOR_CAP_XY);
    colorConfig.callbacks.move_to_hue = APP_ZIGBEE_ColorCbMoveToHue;
    
    LOG_INFO_APP( "Color config prepared, capabilities=0x%02X, calling ZbZclColorServerAlloc...", colorConfig.capabilities );
    stZigbeeAppInfo.ColorServer = ZbZclColorServerAlloc( 
      stZigbeeAppInfo.pstZigbee, 
      APP_ZIGBEE_COLOR_ENDPOINT, 
      NULL,           /* onoff_server */
      NULL,           /* attribute_list */
      0,              /* num_attrs */
      &colorConfig,   /* config */
      NULL            /* arg */
    );
    
    if (stZigbeeAppInfo.ColorServer == NULL) {
      LOG_ERROR_APP( "Color Server allocation returned NULL" );
    } else {
      LOG_INFO_APP( "Color Server allocated successfully" );
    
      LOG_INFO_APP( "Registering Color Endpoint..." );
      if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.ColorServer ) == false )
      {
        LOG_ERROR_APP( "Error during Color Server Endpoint Register." );
        stZigbeeAppInfo.ColorServer = NULL;
      }
      else
      {
        LOG_INFO_APP( "Color Endpoint registered" );

        /* Configure default reporting for the Color cluster */
        {
          enum ZclStatusCodeT eStatus;
          eStatus = ZbZclAttrReportConfigDefault(stZigbeeAppInfo.ColorServer,
              ZCL_COLOR_ATTR_CURRENT_HUE, APP_ZIGBEE_REPORT_MIN_INTERVAL, APP_ZIGBEE_REPORT_MAX_INTERVAL, NULL);
          if ( eStatus != ZCL_STATUS_SUCCESS )
          {
            LOG_ERROR_APP( "Color Hue report config default failed (0x%02X)", eStatus );
          }
          else
          {
            LOG_INFO_APP( "Color Hue report defaults configured" );
          }

          eStatus = ZbZclAttrReportConfigDefault(stZigbeeAppInfo.ColorServer,
              ZCL_COLOR_ATTR_CURRENT_SAT, APP_ZIGBEE_REPORT_MIN_INTERVAL, APP_ZIGBEE_REPORT_MAX_INTERVAL, NULL);
          if ( eStatus != ZCL_STATUS_SUCCESS )
          {
            LOG_ERROR_APP( "Color Sat report config default failed (0x%02X)", eStatus );
          }
          else
          {
            LOG_INFO_APP( "Color Sat report defaults configured" );
          }
        }
      }
    }
  }

  /* USER CODE BEGIN APP_ZIGBEE_ConfigEndpoints2 */

  /* USER CODE END APP_ZIGBEE_ConfigEndpoints2 */
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
 * @brief  Print application information to the console
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_PrintApplicationInfo(void)
{
  LOG_INFO_APP( "**********************************************************" );
  LOG_INFO_APP( "Network config : CENTRALIZED ROUTER" );

  /* USER CODE BEGIN APP_ZIGBEE_PrintApplicationInfo1 */
  LOG_INFO_APP( "Application Flashed : Zigbee %s %s", APP_ZIGBEE_APPLICATION_NAME, APP_ZIGBEE_APPLICATION_OS_NAME );

  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo1 */
  LOG_INFO_APP( "Channel used: %d.", APP_ZIGBEE_CHANNEL );

  APP_ZIGBEE_PrintGenericInfo();

  LOG_INFO_APP( "Clusters allocated are:" );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER_NAME, APP_ZIGBEE_ENDPOINT );
  if (stZigbeeAppInfo.ColorServer != NULL) {
    LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_COLOR_CLUSTER_NAME, APP_ZIGBEE_COLOR_ENDPOINT );
  }

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

static void APP_ZIGBEE_SendReport( struct ZbZclClusterT *cluster, const char *clusterName )
{
  if (cluster == NULL) {
    return;
  }

  enum ZclStatusCodeT eStatus;
  eStatus = ZbZclClusterReportsSend(cluster, true, NULL, NULL);
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[REPORT] Failed to send report for %s (0x%02X)", clusterName, eStatus );
  }
  else
  {
    LOG_INFO_APP( "[REPORT] Sent report for %s", clusterName );
  }
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
 * @brief  Write locally Temp Measure attribute 
 * @param  sensor read temp value
 * @retval None
 */
static void APP_ZIGBEE_TempMeasAttributeUpdate( void )
{
  uint8_t             cTempVariation, cTempAP; 
  int16_t             iTempBP;
  enum ZclStatusCodeT eStatus;
  char                szText[10];
  DHT11_DataTypeDef   dht11_data;
  DHT11_StatusTypeDef dht11_status;
  
  /* Try to read from DHT11 sensor first */
  dht11_status = DHT11_Read(&dht11_data);
  if (dht11_status == DHT11_OK)
  {
    /* DHT11 sensor is available and responded successfully */
    iTemperatureCurrent = DHT11_GetTemperatureCC(&dht11_data);
    LOG_INFO_APP( "[TEMP MEAS] DHT11 OK: %d.%02d°C, Humidity: %d.%02d%%", 
                  dht11_data.temperature_int, dht11_data.temperature_dec,
                  dht11_data.humidity_int, dht11_data.humidity_dec );
  }
  else
  {
    /* DHT11 sensor not available, fall back to random generator simulation */
    LOG_WARNING_APP( "[TEMP MEAS] DHT11 FAILED (code %d) - RNG fallback", dht11_status );
    
    /* Simulation of a Temperature Sensor */
    ZIGBEE_PLAT_RngGet( 1, &cTempVariation );
    if ( cTempVariation > 128u )
    { 
      iTemperatureCurrent += APP_ZIGBEE_TEMP_TOLERANCE; 
    }
    else
    { 
      iTemperatureCurrent -= APP_ZIGBEE_TEMP_TOLERANCE; 
    }
  }
  
  /* Verify if Temperature on limits */
  if ( iTemperatureCurrent > APP_ZIGBEE_TEMP_MAX )
  { 
    iTemperatureCurrent = APP_ZIGBEE_TEMP_MAX; 
  }
  else
  {
    if ( iTemperatureCurrent < APP_ZIGBEE_TEMP_MIN )
    { 
      iTemperatureCurrent = APP_ZIGBEE_TEMP_MIN; 
    }
  }
    
  iTempBP = (int16_t)(iTemperatureCurrent / 100);
  cTempAP = (uint8_t)(iTemperatureCurrent % 100);
  snprintf( szText, sizeof(szText), "%d.%02d", iTempBP, cTempAP );
  LOG_INFO_APP( "[TEMP MEAS] Update TempMeasure : %s C", szText );
  APP_LED_TOGGLE(LED_WORK);
  
  eStatus = ZbZclAttrIntegerWrite( stZigbeeAppInfo.TempMeasServer, ZCL_TEMP_MEAS_ATTR_MEAS_VAL, iTemperatureCurrent);
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[TEMP MEAS] Attribute Write error (0x%02X)", eStatus );
  }
  else
  {
    APP_ZIGBEE_SendReport( stZigbeeAppInfo.TempMeasServer, "TempMeas" );
  }

  /* Update Color attributes */
  if (stZigbeeAppInfo.ColorServer != NULL) {
    APP_ZIGBEE_ColorAttributeUpdate();
  }
}


/**
 * @brief  Write locally Color attributes
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ColorAttributeUpdate( void )
{
  if (stZigbeeAppInfo.ColorServer == NULL) {
    return;
  }

  uint8_t             cColorVariation;
  enum ZclStatusCodeT eStatus;

  /* Simulation of a Color Sensor */
  ZIGBEE_PLAT_RngGet( 1, &cColorVariation );
  if ( cColorVariation > 128u )
  {
    if (uiColorHueCurrent < 254) uiColorHueCurrent += 5;
    if (uiColorSatCurrent < 254) uiColorSatCurrent += 1;
  }
  else
  {
    if (uiColorHueCurrent > 0) uiColorHueCurrent -= 5;
    if (uiColorSatCurrent > 0) uiColorSatCurrent -= 1;
  }

  LOG_INFO_APP( "[COLOR] Update Color : Hue=%d, Sat=%d", uiColorHueCurrent, uiColorSatCurrent );

  uint8_t r, g, b;
  APP_ZIGBEE_HSVtoRGB( uiColorHueCurrent, uiColorSatCurrent, &r, &g, &b );
  LOG_INFO_APP( "[COLOR] RGB equivalent : R=%d, G=%d, B=%d", r, g, b );

  eStatus = ZbZclAttrIntegerWrite( stZigbeeAppInfo.ColorServer, ZCL_COLOR_ATTR_CURRENT_HUE, uiColorHueCurrent);
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[COLOR] Attribute Write Hue error (0x%02X)", eStatus );
  }

  eStatus = ZbZclAttrIntegerWrite( stZigbeeAppInfo.ColorServer, ZCL_COLOR_ATTR_CURRENT_SAT, uiColorSatCurrent);
  if ( eStatus != ZCL_STATUS_SUCCESS )
  {
    LOG_ERROR_APP( "[COLOR] Attribute Write Sat error (0x%02X)", eStatus );
  }
  else
  {
    APP_ZIGBEE_SendReport( stZigbeeAppInfo.ColorServer, "Color" );
  }
}

/* Color Server callback handlers */
static enum ZclStatusCodeT APP_ZIGBEE_ColorCbMoveToHue(struct ZbZclClusterT *cluster,
    struct ZbZclColorClientMoveToHueReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* Handle Move To Hue command - for now just return success */
  return ZCL_STATUS_SUCCESS;
}


/* USER CODE END FD_LOCAL_FUNCTIONS */
