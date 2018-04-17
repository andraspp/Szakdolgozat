#ifndef AV_CONTROL_COMMON_H
#define AV_CONTROL_COMMON_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include <vector>

#define     AvTrue              (1)
#define     AvFalse             (0)
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

sensor_msgs::PointCloud2             cloud_msg;
const sensor_msgs::PointCloud2ConstPtr& cloud_msg_ptr = boost::make_shared<sensor_msgs::PointCloud2>(cloud_msg);
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;


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
    double                  Av_object_range_avg;
    unsigned int            Av_object_in_front;
    Av_proximity_level_t    Av_object_proximity;
} Av_objects_t;

typedef struct Av_orientation_s {
    float z; // sin (theta / 2)
    float w; // cos (theta / 2)

    Av_orientation_s(void) : w(0.0f), z(0.0f) {}
    Av_orientation_s(float _w, float _z) : w(_w), z(_z) {}
} Av_orientation_t;

Av_objects_t        Av_object_container[AvMaxObjects];

bool                Av_scan_detection;
bool                Av_scan_detection_LL;
unsigned int        Av_num_of_objects;
Av_orientation_t    Av_robot_orientation;


ros::Publisher  Av_cmd_vel_pub;


void AV_INIT(void);
void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan);
void AV_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom);

void AV_SET_VELO(float lin_vel, float ang_vel);

void AV_SET_OBJECT_PROXIMITY(void);
void AV_DETECT_FRONT_OBJECT (void);
void AV_CLEAR_OBJECTS_ARRAY(void);
int AV_NUM_OF_OBJECTS(void);
bool AV_OBJECT_HAS_ELEMENTS(void);
void AV_COLLISION_AVOIDANCE(void);

#endif