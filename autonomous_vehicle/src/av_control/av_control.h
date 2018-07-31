/*
 * av_control.h
 *
 *  Created on: 2018. júl. 4.
 *      Author: pappa3
 */

#ifndef SRC_AV_CONTROL_AV_CONTROL_H_
#define SRC_AV_CONTROL_AV_CONTROL_H_

#include "av_control_common.h"
#include <geometry_msgs/Twist.h>

ros::Publisher  Av_cmd_vel_pub;

void AV_SET_VELO(float lin_vel, float ang_vel);
void AV_COLLISION_AVOIDANCE(void);

#endif /* SRC_AV_CONTROL_AV_CONTROL_H_ */
