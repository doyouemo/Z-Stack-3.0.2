

/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"

#include "zcl_ZEM.h"

/*********************************************************************
 * CONSTANTS
 */

#define ZEM_DEVICE_VERSION     0
#define ZEM_FLAGS              0

#define ZEM_HWVERSION          1
#define ZEM_ZCLVERSION         1

#define ZEM_MAX_MEASURED_VALUE  2700  // 27.00C
#define ZEM_MIN_MEASURED_VALUE  1700  // 17.00C

#define ZEM_MEASURED_VALUE      2000  // 20.00C

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Global attributes
const uint16 zclZEM_clusterRevision_all = 0x0001; 

// Basic Cluster
const uint8 zclZEM_HWRevision = ZEM_HWVERSION;
const uint8 zclZEM_ZCLVersion = ZEM_ZCLVERSION;
const uint8 zclZEM_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8 zclZEM_ModelId[] = { 16, 'T','I','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclZEM_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclZEM_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclZEM_LocationDescription[17];
uint8 zclZEM_PhysicalEnvironment;
uint8 zclZEM_DeviceEnable;

// Identify Cluster
uint16 zclZEM_IdentifyTime;

// Temperature Sensor Cluster
int16 zclZEM_MeasuredValue;
const int16 zclZEM_MinMeasuredValue = ZEM_MIN_MEASURED_VALUE; 
const uint16 zclZEM_MaxMeasuredValue = ZEM_MAX_MEASURED_VALUE;

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */

// NOTE: The attributes listed in the AttrRec must be in ascending order 
// per cluster to allow right function of the Foundation discovery commands

CONST zclAttrRec_t zclZEM_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclZEM_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclZEM_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclZEM_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclZEM_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclZEM_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclZEM_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclZEM_DeviceEnable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_clusterRevision_all
    }
  },
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclZEM_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclZEM_clusterRevision_all
    }
  },

  // *** Temperature Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclZEM_MeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_MinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_MS_TEMPERATURE_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_MaxMeasuredValue
    }
  },

  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclZEM_clusterRevision_all
    }
  },
};

uint8 CONST zclZEM_NumAttributes = ( sizeof(zclZEM_Attrs) / sizeof(zclZEM_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLZEM_MAX_INCLUSTERS       3
const cId_t zclZEM_InClusterList[ZCLZEM_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT
};

#define ZCLZEM_MAX_OUTCLUSTERS       1
const cId_t zclZEM_OutClusterList[ZCLZEM_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};

SimpleDescriptionFormat_t zclZEM_SimpleDesc =
{
  ZEM_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                                 //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_TEMPERATURE_SENSOR,                //  uint16 AppDeviceId[2];
  ZEM_DEVICE_VERSION,            //  int   AppDevVer:4;
  ZEM_FLAGS,                     //  int   AppFlags:4;
  ZCLZEM_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclZEM_InClusterList, //  byte *pAppInClusterList;
  ZCLZEM_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclZEM_OutClusterList //  byte *pAppInClusterList;
};
 
/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zclSampleLight_ResetAttributesToDefaultValues
 *
 * @brief   Reset all writable attributes to their default values.
 *
 * @param   none
 *
 * @return  none
 */
void zclZEM_ResetAttributesToDefaultValues(void)
{
  int i;
  
  zclZEM_LocationDescription[0] = 16;
  for (i = 1; i <= 16; i++)
  {
    zclZEM_LocationDescription[i] = ' ';
  }
  
  zclZEM_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;
  zclZEM_DeviceEnable = DEVICE_ENABLED;
  
#ifdef ZCL_IDENTIFY
  zclZEM_IdentifyTime = 0;
#endif
  
  zclZEM_MeasuredValue = ZEM_MEASURED_VALUE;
  
}

/****************************************************************************
****************************************************************************/


