#include "av_control_common.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "av_sensorics");
    AV_INIT();

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
            AV_DETECT_FRONT_OBJECT();
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
}

void AV_CLEAR_OBJECTS_ARRAY()
{
    for(int oidx=0; oidx < AvMaxObjects; oidx++)
    {
        Av_object_container[oidx].Av_obj_id      = -1;
        Av_object_container[oidx].Av_scan_low_point = -1;
        Av_object_container[oidx].Av_scan_high_point = -1;
        Av_object_container[oidx].Av_object_range_avg = -1.00;
    }
}

bool AV_OBJECT_HAS_ELEMENTS()
{
    if (Av_object_container[0].Av_obj_id != -1)
    {
        return AvTrue;
    }
    else
    {
        return AvFalse;
    }
}

unsigned int AV_NUM_OF_OBJECTS()
{
    unsigned int oidx;

    oidx = 0;

    while(Av_object_container[oidx].Av_obj_id != -1)
    {
        oidx++;
    }

    return oidx;
}

void AV_DETECT_FRONT_OBJECT()
{
    
}

void AV_SENSORICS_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    tf::TransformBroadcaster broadcaster;
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
        std::cout << std::endl;
        std::cout << "temp num of objects: " << num_of_objects << " stored num of objects: " << Av_num_of_objects << std::endl;
        

        if(AV_OBJECT_HAS_ELEMENTS() == AvTrue)
        {
            for(int idx; idx < Av_num_of_objects; idx++)
            {
                std::cout << std::endl;
                std::cout << "Object No: " << Av_object_container[idx].Av_obj_id << std::endl;
                std::cout << "Object Start Angle: " << Av_object_container[idx].Av_scan_low_point << " Object End Angle: " << Av_object_container[idx].Av_scan_high_point << std::endl;
                std::cout << "Object average range: " << Av_object_container[idx].Av_object_range_avg << std::endl;
            }
        }
        
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
}