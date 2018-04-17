#include "av_control_common.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "av_sensorics");
    

    // Exit if no ROS
    if (!ros::isInitialized())
    {
        std::cout << "Not loading plugin since ROS hasn't been "
            << "properly initialized.  Try starting gazebo with ros plugin:\n" << std::endl
            << "  gazebo -s libgazebo_ros_api_plugin.so\n" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "ROS has been properly initialized :-) " << std::endl;
        ros::NodeHandle av_sensorics_nh_;

        //ros::Subscriber AV_ODOM_CHECK  = av_sensorics_nh_.subscribe<sensor_msgs::LaserScan>("/av_robot/odom",sizeof(nav_msgs::Odometry),&AV_ODOMETRY_CALLBACK);
        ros::topic::waitForMessage<nav_msgs::Odometry>("/av_robot/odom");
        ros::Subscriber AV_LIDAR_CHECK = av_sensorics_nh_.subscribe<sensor_msgs::LaserScan>("/av_robot/laser/scan",10,&AV_SENSORICS_LIDAR_CALLBACK);
        
        Av_cmd_vel_pub = av_sensorics_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        AV_INIT();

        AV_SET_VELO(0.0, 2.0);
        ros::Rate loop_rate(10);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }



    return 0;
}

void AV_INIT()
{
    AV_CLEAR_OBJECTS_ARRAY();
    AV_OBJECT_HAS_ELEMENTS();
    AV_SET_VELO(0.0, 0.0);
}

void AV_CLEAR_OBJECTS_ARRAY()
{
    for(int idx=0; idx < AvMaxObjects; idx++)
    {
        Av_object_container[idx].Av_obj_id             = -1;
        Av_object_container[idx].Av_scan_low_point     = -1;
        Av_object_container[idx].Av_scan_high_point    = -1;
        Av_object_container[idx].Av_object_range_avg   = -1.00;
        Av_object_container[idx].Av_object_in_front    = AvFalse;
        Av_object_container[idx].Av_object_proximity   = AvProximityFar;
    }
}

bool AV_OBJECT_HAS_ELEMENTS()
{
    bool ret_val;

    if (Av_object_container[0].Av_obj_id != -1)
    {
        ret_val = AvTrue;
    }
    else
    {
        ret_val = AvFalse;
    }

    ROS_INFO("AV_OBJECT_HAS_ELEMENTS return value: %d", ret_val);
    return ret_val;
}

int AV_NUM_OF_OBJECTS()
{
    int oidx;

    oidx = 0;

    while(Av_object_container[oidx].Av_obj_id != -1)
    {
        oidx++;
    }

    return oidx;
}

void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laser_geometry::LaserProjection projector;
    float sum;
    unsigned int cnt;
    bool object_detected;
    signed int num_of_objects;

    try
    {
        sum = 0;
        cnt = 0;
        object_detected = AvFalse;
        num_of_objects = -1;

        AV_CLEAR_OBJECTS_ARRAY();

        for (int scan_idx=0; scan_idx < 640; scan_idx++)
        {
            if(   (scan->ranges[scan_idx] <= scan->range_max)
               && (scan->ranges[scan_idx] >= scan->range_min)
               && (num_of_objects         <  AvMaxObjects)
            )
            {
                object_detected = AvTrue;
                Av_scan_detection = AvTrue;

                if(Av_scan_detection_LL == AvFalse)
                {
                    ++num_of_objects;
                    Av_object_container[num_of_objects].Av_scan_low_point = scan_idx;
                    Av_object_container[num_of_objects].Av_obj_id = num_of_objects;
                }

                projector.projectLaser(*scan, cloud_msg,-1.0, 0x03);

                sum = (float)(sum + scan->ranges[scan_idx]);
                cnt++;
            }
            else
            {
                Av_scan_detection = AvFalse;
                if(Av_scan_detection_LL == AvTrue)
                {
                    Av_object_container[num_of_objects].Av_scan_high_point = scan_idx - 1;
                    if(cnt != 0)
                    {
                        Av_object_container[num_of_objects].Av_object_range_avg = (double)sum/cnt;
                    } 
                }
            }
            
            Av_scan_detection_LL = Av_scan_detection;
        }

        if(object_detected == AvFalse) /* No object within range*/
        {
            AV_CLEAR_OBJECTS_ARRAY();
        }

        Av_num_of_objects = (unsigned char)AV_NUM_OF_OBJECTS();
        #if((AvDebugConfig & AvDebugStoredObjectsInfoEnable) > 0)
        ROS_INFO("temp num of objects: %d, stored num of objects: %d", num_of_objects, Av_num_of_objects);
        ROS_INFO("Object 0's ID: %d", Av_object_container[0].Av_obj_id);
        #endif /* AvDebugStoredObjectsInfoEnable */
        

        if(Av_num_of_objects > 0) /* There are tracked objects */
        {
            AV_SET_OBJECT_PROXIMITY();  /* Set their proximity state */
            AV_DETECT_FRONT_OBJECT();   /* Detect those that are in front of the AV */

            #if((AvDebugConfig & AvDebugObjectsStateInfoEnable) > 0)
            for(int idx; idx < Av_num_of_objects; idx++)
            {
                ROS_INFO(" ");
                ROS_INFO("Object No: %d", Av_object_container[idx].Av_obj_id);
                ROS_INFO("Object Start Angle: %d  Object End Angle: %d", Av_object_container[idx].Av_scan_low_point, Av_object_container[idx].Av_scan_high_point);
                ROS_INFO("Object average range: %1.8f", Av_object_container[idx].Av_object_range_avg);
                ROS_INFO("Object proximity state: %d", Av_object_container[idx].Av_object_proximity);
                ROS_INFO("Object front?: %d", Av_object_container[idx].Av_object_in_front);
            }
            #endif /* AvDebugObjectsStateInfoEnable */
            AV_COLLISION_AVOIDANCE();
            
        }
        else
        {
            /* proceed with normal speed */
            AV_SET_VELO(0.0, 1.0);
        }
        
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
}

void AV_SET_OBJECT_PROXIMITY()
{
    for(int idx; idx < Av_num_of_objects; idx++)
    {
        if(Av_object_container[idx].Av_object_range_avg >= AvProxyFarThr) /* If range is really far, don't bother with state check */
        {
            Av_object_container[idx].Av_object_proximity = AvProximityFar;
        }
        else
        {
            switch(Av_object_container[idx].Av_object_proximity)
            {
            case AvProximityFar:
                if (Av_object_container[idx].Av_object_range_avg >= AvProxyCloseThr)
                {
                    continue; /* Stay in Far state */
                }
                else if( Av_object_container[idx].Av_object_range_avg < AvProxyDngrThr)
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityDanger;
                }
                else if( Av_object_container[idx].Av_object_range_avg < AvProxyCloseThr)
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityClose;
                }
                else
                {
                    /* Stay in Far state */
                }
                break;
            case AvProximityClose:
                if (   (Av_object_container[idx].Av_object_range_avg >= AvProxyDngrThr) 
                    && (Av_object_container[idx].Av_object_range_avg <= AvProxyCloseThr)
                )
                {
                    continue; /* Stay in Close state */
                }
                else if( Av_object_container[idx].Av_object_range_avg < (AvProxyDngrThr - AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityDanger;
                }
                else if(Av_object_container[idx].Av_object_range_avg > (AvProxyCloseThr + AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else
                {
                    /* Stay in Close state */
                }
                break;
            case AvProximityDanger:
                if (Av_object_container[idx].Av_object_range_avg <= AvProxyDngrThr) 
                {
                    continue; /* Stay in Close state */
                }
                else if(Av_object_container[idx].Av_object_range_avg > (AvProxyCloseThr + AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else if(Av_object_container[idx].Av_object_range_avg > (AvProxyDngrThr + AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityClose;
                }
                else
                {
                    /* Stay in Close state */
                }
                break;
            case AvProximityUnknown:
            default:
                if( Av_object_container[idx].Av_object_range_avg > AvProxyCloseThr )
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else if( Av_object_container[idx].Av_object_range_avg > AvProxyDngrThr )
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityClose;
                }
                else
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityDanger;
                }
                break;
            }
        }
    }
}

void AV_DETECT_FRONT_OBJECT()
{
    #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
    ROS_INFO("Front Det entered");
    #endif /* AvDebugFrontDetInfoEnable */

    for(int idx; idx < Av_num_of_objects; idx++)
    {
        if(   ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_low_point  <= (AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr)) /*TODO: implement turn angle correction*/
           || ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_high_point >= (AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr))
           || (   ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) <= Av_object_container[idx].Av_scan_high_point) 
               && ((AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_low_point)
              )
          )
        {
            #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
            ROS_INFO("Object ID: %d detected to the front!!", Av_object_container[idx].Av_obj_id);
            #endif /* AvDebugFrontDetInfoEnable */
            Av_object_container[idx].Av_object_in_front = AvTrue;
        }
        else
        {
            #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
            ROS_INFO("Object ID: %d is NOT in front!!", Av_object_container[idx].Av_obj_id);
            #endif /* AvDebugFrontDetInfoEnable */
            Av_object_container[idx].Av_object_in_front = AvFalse;
        }

    }
}

void AV_SET_VELO(float lin_vel, float ang_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = lin_vel;
    msg.angular.z = ang_vel;

    #if((AvDebugConfig & AvDebugSpeedInfoEnable) > 0)
    ROS_INFO(" ");
    ROS_INFO("Robot ang speed: %1.8f Robot lin speed: %1.8f", msg.angular.z, msg.linear.x);
    #endif /* AvDebugSpeedInfoEnable */

    Av_cmd_vel_pub.publish(msg);
}

void AV_COLLISION_AVOIDANCE()
{
    for(int idx; idx < Av_num_of_objects; idx++)
    {
        if(   (Av_object_container[idx].Av_object_in_front  == AvTrue)
           && (Av_object_container[idx].Av_object_proximity == AvProximityDanger)
        )
        {
            /* stop to avoid collision */
            AV_SET_VELO(0.0, 0.0);
        }
        else if(   (Av_object_container[idx].Av_object_in_front == AvTrue)
               && (Av_object_container[idx].Av_object_proximity == AvProximityClose)
        )
        {
            /* slow down and turn to avoid collision */
            AV_SET_VELO(0.25, 0.5);
        }
        /*else if(   (Av_object_container[idx].Av_object_in_front  == AvFalse)
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

/*
void AV_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom)
{
    Av_robot_orientation.z = odom->pose.pose.orientation.z;
    Av_robot_orientation.w = odom->pose.pose.orientation.w;
}*/