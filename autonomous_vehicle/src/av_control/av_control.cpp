/*
 * av_control.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_control.h"

void AV_CONTROL()
{
    ROS_INFO("Entered AV_CONTROL");
    ros::NodeHandle av_ctrl_nh;

    Av_cmd_vel_pub = av_ctrl_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    AV_SET_VELO(0.0, 2.0);

    AV_COLLISION_AVOIDANCE();
}


void AV_COLLISION_AVOIDANCE()
{
    for(int idx; idx < Av_num_of_objects; idx++)
    {
        if(   (Av_object_container[idx].Av_object_in_front  == True)
           && (Av_object_container[idx].Av_object_proximity == AvProximityDanger)
        )
        {
            /* stop to avoid collision */
            AV_SET_VELO(0.0, 0.0);
        }
        else if(   (Av_object_container[idx].Av_object_in_front == True)
               && (Av_object_container[idx].Av_object_proximity == AvProximityClose)
        )
        {
            /* slow down and turn to avoid collision */
            AV_SET_VELO(0.25, 0.5);
        }
        /*else if(   (Av_object_container[idx].Av_object_in_front  == False)
                && (Av_object_container[idx].Av_object_proximity == AvProximityDanger)
        )
        {
            if(Av_object_container[idx].Av_scan_low_point > 320)
            {

            }
        }*/
        else
        {
            /* no change in course necessary */
            AV_SET_VELO(0.0, 1.0);
        }
    }
}

void AV_SET_VELO(float lin_vel, float ang_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    #if((AvDebugConfig & AvDebugSpeedInfoEnable) > 0)
    //ROS_INFO(" ");
    //ROS_INFO("Robot ang speed: %1.8f Robot lin speed: %1.8f", msg.angular.z, msg.linear.x);
    #endif /* AvDebugSpeedInfoEnable */

    Av_cmd_vel_pub.publish(msg);
}
