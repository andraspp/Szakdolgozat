/*
 * av_control.h
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#ifndef SRC_AV_CONTROL_AV_CONTROL_H_
#define SRC_AV_CONTROL_AV_CONTROL_H_

#include "av_control_common.h"
#include <geometry_msgs/Twist.h>

ros::Publisher  Av_cmd_vel_pub;

double Av_current_speed;
double Av_current_yaw;


#endif /* SRC_AV_CONTROL_AV_CONTROL_H_ */
