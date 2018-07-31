#include "av_control_common.h"


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
   AV_SENSING_INIT();
}

