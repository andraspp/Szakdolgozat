#include "av_control_common.h"

Av_objects_t        Av_object_container[AvMaxObjects];
unsigned int        Av_num_of_objects;
Av_orientation_t    Av_orientation;
bool                Av_StopSignDet, Av_CollisionWarning;
double              Av_Arbitrated_Target_Yaw, Av_Arbitrated_Target_Speed, Av_Lane_Target_Yaw, Av_Lane_Target_Speed;


int main(int argc, char **argv)
{
    double main_clock = 0;

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
            ROS_INFO("-- MAIN CLOCK: %f", main_clock);

            AV_SENSING();
            AV_PERCEPTION();
            AV_PLANNING();
            AV_CONTROL();
            ros::spinOnce();
            loop_rate.sleep();
            main_clock++;

            ROS_INFO(" Object Info");
            ROS_INFO(" Num of objects PERC: %d", Av_num_of_objects);
            ROS_INFO("   >> ID: %d. LP: %d, HP: %d, MINR: %lf, Fr: %d, Prox: %d", Av_object_container[0].Av_obj_id, Av_object_container[0].Av_scan_low_point, Av_object_container[0].Av_scan_high_point, Av_object_container[0].Av_object_range_min, Av_object_container[0].Av_object_in_front, Av_object_container[0].Av_object_proximity);
            ROS_INFO("   >> ID: %d. LP: %d, HP: %d, MINR: %lf, Fr: %d, Prox: %d \n", Av_object_container[1].Av_obj_id, Av_object_container[1].Av_scan_low_point, Av_object_container[1].Av_scan_high_point, Av_object_container[1].Av_object_range_min, Av_object_container[1].Av_object_in_front, Av_object_container[1].Av_object_proximity);
        }
    }

    return 0;
}

void AV_INIT()
{
   AV_SENSING_INIT();
   AV_PERCEPTION_INIT();
   AV_PLANNING_INIT();
}

