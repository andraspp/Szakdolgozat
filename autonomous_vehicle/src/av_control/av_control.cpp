/*
 * av_control.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_control.h"

void AV_CONTROL()
{
    ROS_INFO("-- Entered AV_CONTROL");
    //ROS_INFO("Entered AV_CONTROL");
    ros::NodeHandle av_ctrl_nh;

    Av_cmd_vel_pub = av_ctrl_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    /* Ramp to target speed */
    if(Av_Arbitrated_Target_Speed == Av_current_speed)
    {
        /* nothing to do */
    }
    else if((Av_Arbitrated_Target_Speed - Av_current_speed) > 0) /* need to accelerate*/
    {
        if((Av_Arbitrated_Target_Speed - Av_current_speed) < AvSpeedGrad)
        {
            Av_current_speed = Av_Arbitrated_Target_Speed;
        }
        else
        {
            Av_current_speed += AvSpeedGrad;
        }
    }
    else if((Av_Arbitrated_Target_Speed - Av_current_speed) < 0) /* need to decelerate*/
    {
        if((Av_current_speed - Av_Arbitrated_Target_Speed) < AvSpeedGrad)
        {
            Av_current_speed = Av_Arbitrated_Target_Speed;
        }
        else
        {
            Av_current_speed -= AvSpeedGrad;
        }
    }
    else
    {}

    Av_current_yaw = Av_Arbitrated_Target_Yaw;

    AV_SET_VELO(Av_current_yaw, Av_current_speed);
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
