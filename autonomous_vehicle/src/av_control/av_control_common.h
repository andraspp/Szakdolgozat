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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include "av_params.h"

#define     True                (1)
#define     False               (0)

/* DEBUG INFO handling */
#define AvDebugSpeedInfoEnable          (0x01)
#define AvDebugObjectsStateInfoEnable   (0x02)
#define AvDebugStoredObjectsInfoEnable  (0x04)
#define AvDebugFrontDetInfoEnable       (0x08)

//#define AvDebugConfig                   (AvDebugSpeedInfoEnable | AvDebugObjectsStateInfoEnable | AvDebugStoredObjectsInfoEnable | AvDebugFrontDetInfoEnable)
#define AvDebugConfig                   (0x0)

using namespace cv;
using namespace std;
using namespace cv_bridge;

typedef enum Av_proximity_level_e {
    AvProximityUnknown      = 10,
    AvProximityFar          = 3,
    AvProximityClose        = 2,
    AvProximityDanger       = 1
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

extern Av_objects_t        Av_object_container[AvMaxObjects];
extern unsigned int        Av_num_of_objects;
extern Av_orientation_t    Av_orientation;
extern unsigned int        Av_scan_low_points_LL[AvMaxObjects];
extern unsigned int        Av_scan_high_points_LL[AvMaxObjects];
extern bool                Av_StopSignDet, Av_CollisionWarning, Av_SideWayCollWarning;
extern double              Av_Arbitrated_Target_Yaw, Av_Arbitrated_Target_Speed, Av_Lane_Target_Yaw, Av_Lane_Target_Speed;


void AV_INIT(void);
void AV_SENSING_INIT(void);
void AV_PERCEPTION_INIT(void);
void AV_PLANNING_INIT(void);
void AV_CONTROL_INIT(void);

void AV_SENSING(void);
void AV_PERCEPTION(void);
void AV_PLANNING(void);
void AV_CONTROL(void);

void AV_DETECT_STOP_SIGN(Mat image);
void AV_DETECT_LANE(Mat image);

void AV_LINE_FOLLOW(Moments mom, int img_height, int img_width);

void AV_SET_VELO(float lin_vel, float ang_vel);

void AV_SENSING_IMAGE_CALLBACK(const sensor_msgs::Image::ConstPtr& img_scan);
void AV_SENSING_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan);
void AV_SENSING_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom);

#endif
