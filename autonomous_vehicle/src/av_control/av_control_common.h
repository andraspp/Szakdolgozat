#ifndef AV_CONTROL_COMMON_H
#define AV_CONTROL_COMMON_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <sstream>
#include <vector>

#define     True                (1)
#define     False               (0)
#define     AvFrontAngle        (320)
#define     AvFrontAngleOffset  (5)
#define     AvMaxObjects        (10)
#define     AvTmpAngleCorr      (0)

/* Object range thresholds */
#define     AvProxyFarThr       (7)
#define     AvProxyCloseThr     (4)
#define     AvProxyDngrThr      (2)
#define     AvProxyHyst         (0.1)

/* DEBUG INFO handling */
#define AvDebugSpeedInfoEnable          (0x01)
#define AvDebugObjectsStateInfoEnable   (0x02)
#define AvDebugStoredObjectsInfoEnable  (0x04)
#define AvDebugFrontDetInfoEnable       (0x08)

#define AvDebugConfig                   (AvDebugSpeedInfoEnable | AvDebugObjectsStateInfoEnable | AvDebugStoredObjectsInfoEnable | AvDebugFrontDetInfoEnable)

typedef enum Av_proximity_level_e {
    AvProximityFar          = 0,
    AvProximityClose        = 2,
    AvProximityDanger       = 3,
    AvProximityUnknown      = 10
} Av_proximity_level_t;

typedef struct Av_object_s {
    signed int              Av_obj_id;
    signed int              Av_scan_low_point;
    signed int              Av_scan_high_point;
    double                  Av_object_range_min;
    unsigned int            Av_object_in_front;
    Av_proximity_level_t    Av_object_proximity;
} Av_objects_t;

typedef struct Av_orientation_s {
   float                     pose_x;
   float                     pose_y;
   float                     pose_z;
   geometry_msgs::Quaternion orient;
} Av_orientation_t;

Av_objects_t        Av_object_container[AvMaxObjects];
unsigned int        Av_num_of_objects;
Av_orientation_t    Av_orientation;

void AV_INIT(void);
void AV_SENSING_INIT(void);
void AV_PERCEPTION_INIT(void);
void AV_PLANNING_INIT(void);
void AV_CONTROL_INIT(void);

void AV_SENSING(void);
void AV_PERCEPTION(void);
void AV_PLANNING(void);
void AV_CONTROL(void);

#endif
