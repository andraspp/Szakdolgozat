/*
 * av_planning.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_planning.h"

void AV_PLANNING_INIT()
{
    Av_CollisionWarning = false;
    Av_StopSignDet = false;
    Av_Lane_Target_Yaw = 0.0;
    Av_Arbitrated_Target_Speed = 0.0;
    Av_Arbitrated_Target_Yaw = 0.0;
}

void AV_PLANNING()
{
   /* Arbitration */
    ROS_INFO("-- Entered AV_PLANNING");

    if(   (Av_StopSignDet == true)
       || (Av_CollisionWarning == true)
      )
    {
        Av_Arbitrated_Target_Yaw = 0.0;
        Av_Arbitrated_Target_Speed = 0.0;
    }
    else
    {
        Av_Arbitrated_Target_Yaw = Av_Lane_Target_Yaw;
        Av_Arbitrated_Target_Speed = Av_Lane_Target_Speed;
    }
   
   /* Nothing specific yet */
}

void AV_LINE_FOLLOW(Moments mom, int img_height, int img_width)
{
    double yaw, throttle;
    double x, y;

    double d_M01 =  mom.m01;
    double d_M10 = mom.m10;
    double d_Area = mom.m00;

    if(d_Area != 0)
    {
        x = d_M10/d_Area;
        y = d_M01/d_Area;
    }
    else
    {
        x = img_width/2;
        y = img_height/2;
    }
    ROS_INFO("X, Y: %f, %f", x, y);
    ROS_INFO("Width, Height: %d, %d", img_width, img_height);

    //circle(img_extract, Point(x, y), 2, Scalar( 0, 0, 255 ), 1, LINE_8, 0);

    //namedWindow("Image window",1);
    //imshow("Image window", img_extract);
    //waitKey(0);

    yaw = ((x - img_width/2) / 10) * (-1);
    ROS_INFO("center = %d, point = %f, yaw = %f", img_width/2, x, yaw);

    throttle = 1.2;

    if (yaw > 1.9)
    {
        yaw = 1.9;
    }
    else if (yaw < (-1.9))
    {
        yaw = -1.9;
    }
    else
    {}

    Av_Lane_Target_Speed = throttle;
    Av_Lane_Target_Yaw = yaw;
  
}
