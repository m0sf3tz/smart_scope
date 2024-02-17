#pragma once

#define RADAR_MQ_PATH "/mq_radar"

#define VIRTUAL_UART_PORTS (2)
enum port_e { cfg_port_e, data_port_e };

// From TI, TLV tag
typedef enum MmwDemo_output_message_type_e
{
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_STATS,
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO,
    MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP,
    MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS,
    MMWDEMO_OUTPUT_MSG_MAX
} MmwDemo_output_message_type;

// Misc TLVs
#define MMWDEMO_OUTPUT_MSG_SPHERICAL_POINTS 1000
#define MMWDEMO_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST 1010
#define MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX   1011
#define MMWDEMO_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT  1012

typedef struct DPIF_PointCloudSpherical_t
{
    /*! @brief     Range in meters */
    float range;

    /*! @brief     Azimuth angle in degrees in the range [-90,90],
     *             where positive angle represents the right hand side as viewed
     *             from the sensor towards the scene and negative angle
     *             represents left hand side */
    float azimuthAngle;

    /*! @brief     Elevation angle in degrees in the range [-90,90],
                   where positive angle represents above the sensor and negative
     *             below the sensor */
    float elevAngle;

    /*! @brief  Doppler velocity estimate in m/s. Positive velocity means target
     *          is moving away from the sensor and negative velocity means target
     *          is moving towards the sensor. */
    float velocity;
} DPIF_PointCloudSpherical;


// From TI, TLV structure for point cloud
typedef struct DPIF_PointCloudCartesian_t
{
    float  x;
    float  y;
    float  z;
    float  velocity;
} DPIF_PointCloudCartesian;

// From TI, TLV structure for side info of points
typedef struct DPIF_PointCloudSideInfo_t
{
    int16_t  snr;
    int16_t  noise;
} DPIF_PointCloudSideInfo;

typedef struct PointCloudMetaData_t
{
    uint32_t frameNumber;    // From IWR
    uint32_t timeCpuCycles;  // From IWR
    uint32_t points;         // From IWR

    uint32_t seconds;        // From TLV parser
    uint32_t nanoseconds;    // From TLV parser
} PointCloudMetaData;

// This is packed since it gets sent over the wire to python
// This is a single point in a point cloud stored in 
// spherical co-ordinates as well as side info for that 
// single point 
typedef struct PointCloudSphericalAndSnr_t
{
  DPIF_PointCloudSpherical sphere;  
  DPIF_PointCloudSideInfo side;
} __attribute__((packed)) SphericalPointAndSnr;

// This number is defined as such,
// The default size of a message queue is 8196 on our OS
// the size of packed PointCloudCartesianAndSnr is 24 bytes
// This means we can fit about ~325 points in a single message
#define MAX_CLOUD_POINTS (325)
typedef struct PointCloudWireFormatSpherical_t 
{
  PointCloudMetaData   meta_data;
  SphericalPointAndSnr points[MAX_CLOUD_POINTS];
} __attribute__((packed)) PointCloudSpherical;
