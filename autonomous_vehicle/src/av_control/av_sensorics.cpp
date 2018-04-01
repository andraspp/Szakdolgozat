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
        ros::Publisher  av_cloud_pub;

        ros::Subscriber AV_LIDAR_CHECK = av_sensorics_nh_.subscribe<sensor_msgs::LaserScan>("/av_robot/laser/scan",10,&AV_SENSORICS_LIDAR_CALLBACK);
        av_cloud_pub = av_sensorics_nh_.advertise<sensor_msgs::PointCloud>("/av_robot/cloud",1);
        
        ros::Rate loop_rate(10);
        while(ros::ok())
        {
            av_cloud_pub.publish(cloud); 
            ros::spinOnce();
            loop_rate.sleep();
        }
    }



    return 0;
}

void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    laser_geometry::LaserProjection projector;

    try
    {
        projector.projectLaser(*scan, cloud,-1.0, 0x03);
        std::cout << "LIDAR info received :o! " << std::endl;
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
}