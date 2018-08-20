#include "av_control_common.h"

Av_objects_t        Av_object_container[AvMaxObjects];
unsigned int        Av_num_of_objects;
Av_orientation_t    Av_orientation;
Mat                 Av_img_hsv, Av_img_threshold;
bool                Av_StopSignDet;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "av_main");

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
        ros::NodeHandle av_nh;

        ros::Subscriber AV_LIDAR_CHECK = av_nh.subscribe<sensor_msgs::LaserScan>("/av_robot/laser/scan",10,&AV_SENSING_LIDAR_CALLBACK);
        ros::Subscriber AV_IMG_CHECK   = av_nh.subscribe<sensor_msgs::Image>("/camera_front/image_raw",sizeof(sensor_msgs::Image),&AV_SENSING_IMAGE_CALLBACK);
        ros::Subscriber AV_ODOM_CHECK  = av_nh.subscribe<nav_msgs::Odometry>("/av_robot/odom",sizeof(nav_msgs::Odometry),&AV_SENSING_ODOMETRY_CALLBACK);
        
        
        AV_INIT();

        ros::Rate loop_rate(10);
        while(ros::ok())
        {
           AV_SENSING();
           AV_PERCEPTION();
           AV_PLANNING();
           AV_CONTROL();
           ros::spinOnce();
           loop_rate.sleep();
        }
    }

    return 0;
}

void AV_INIT()
{
   Av_StopSignDet = False;
   AV_SENSING_INIT();
}

