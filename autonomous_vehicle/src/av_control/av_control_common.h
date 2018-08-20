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

#define     True                (1)
#define     False               (0)
#define     AvFrontAngle        (320)
#define     AvFrontAngleOffset  (15)
#define     AvMaxObjects        (5)
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

//#define AvDebugConfig                   (AvDebugSpeedInfoEnable | AvDebugObjectsStateInfoEnable | AvDebugStoredObjectsInfoEnable | AvDebugFrontDetInfoEnable)
#define AvDebugConfig                   (0x0)

using namespace cv;
using namespace std;
using namespace cv_bridge;

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

extern Av_objects_t        Av_object_container[AvMaxObjects];
extern unsigned int        Av_num_of_objects;
extern Av_orientation_t    Av_orientation;
extern Mat                 Av_img_hsv, Av_img_threshold;
extern bool                Av_StopSignDet;


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

void AV_SENSING_IMAGE_CALLBACK(const sensor_msgs::Image::ConstPtr& img_scan);
void AV_SENSING_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan);
void AV_SENSING_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom);

#endif
