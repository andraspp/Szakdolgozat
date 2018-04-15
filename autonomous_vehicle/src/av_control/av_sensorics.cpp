#include "av_control_common.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "av_sensorics");

    DEBUG_INFO = 0;
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

        ros::Subscriber AV_LIDAR_CHECK = av_sensorics_nh_.subscribe<sensor_msgs::LaserScan>("/av_robot/laser/scan",10,&AV_SENSORICS_LIDAR_CALLBACK);
        
        ros::Rate loop_rate(10);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }



    return 0;
}

void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    tf::TransformBroadcaster broadcaster;
    laser_geometry::LaserProjection projector;

    try
    {
        for (int scan_idx=0; scan_idx < 640; scan_idx++)
        {
            if(   (scan->ranges[scan_idx] <= scan->range_max)
               && (scan->ranges[scan_idx] >= scan->range_min)
            )
            {
                Av_scan_detection = AvTrue;

                if(scan_idx <= Av_scan_low_point)
                {
                    Av_scan_low_point = scan_idx;
                }

                projector.projectLaser(*scan, cloud_msg,-1.0, 0x03);
                std::cout << std::endl;
                std::cout << Av_scan_low_point << " " << Av_scan_high_point << std::endl;
                std::cout << scan->ranges[scan_idx] << std::endl;

                
            }
            else
            {
                Av_scan_detection = AvFalse;
                if(Av_scan_detection_LL == AvTrue)
                {
                    Av_scan_high_point = scan_idx - 1;
                }
            }
            
            Av_scan_detection_LL = Av_scan_detection;
        }
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
}