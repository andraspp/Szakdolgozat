#ifndef AV_CONTROL_COMMON_H
#define AV_CONTROL_COMMON_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <sstream>

sensor_msgs::PointCloud         cloud;

void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan);

#endif