/*
 * av_sensing.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */
#include "av_sensing.h"


void AV_SENSING_INIT(void)
{
    AV_CLEAR_OBJ_ARRAY();
    AV_CLEAR_TMP_ARRAY();
    Av_object_detected      = False;
    Av_num_of_tmp           = 0;
    Av_num_of_tmp_LL        = 0;
    Av_scan_detection       = False;
    Av_scan_detection_LL    = False;
    Av_orientation.pose_x = 0;
    Av_orientation.pose_y = 0;
    Av_orientation.pose_z = 0;
}

void AV_SENSING(void)
{
    bool          tmp_array_nonempty;
    
    ROS_INFO("-- Entered AV_SENSING");
    //ros::NodeHandle av_sens_nh;
    
   
    /***********************************************/
    /*       UPDATE OBJECTS ARRAY FOR PROCESSING   */
    /***********************************************/
    Av_num_of_tmp_LL = Av_num_of_tmp;
    tmp_array_nonempty = (bool)AV_TMP_HAS_ELEMENTS();
    Av_num_of_tmp = (unsigned char)AV_NUM_OF_TMP_OBJECTS();

    /* Number of elements in array changed, updated entire table*/
    if(   (tmp_array_nonempty == True)
        && (Av_num_of_tmp     < AvMaxObjects)
        && (Av_num_of_tmp_LL != Av_num_of_tmp)
        )
    {
        for (int idx=0; idx < AvMaxObjects; idx++)
        {
            Av_object_container[idx].Av_obj_id             = Av_tmp_container[idx].Av_obj_id;
            Av_object_container[idx].Av_scan_low_point     = Av_tmp_container[idx].Av_scan_low_point;
            Av_object_container[idx].Av_scan_high_point    = Av_tmp_container[idx].Av_scan_high_point;
            Av_object_container[idx].Av_object_range_min   = Av_tmp_container[idx].Av_object_range_min;
            Av_object_container[idx].Av_object_in_front    = Av_tmp_container[idx].Av_object_in_front;
            Av_object_container[idx].Av_object_proximity   = Av_tmp_container[idx].Av_object_proximity;
        }
    }
    /* No change in number of elements, just update angles and ranges */
    else if(   (tmp_array_nonempty == True)
            && (Av_num_of_tmp      <  AvMaxObjects)
            && (Av_num_of_tmp_LL   == Av_num_of_tmp)
            )
    {
        for (int idx=0; idx < AvMaxObjects; idx++)
        {
            Av_scan_low_points_LL[idx] = Av_object_container[idx].Av_scan_low_point;
            Av_scan_high_points_LL[idx] = Av_object_container[idx].Av_scan_high_point;
            Av_object_container[idx].Av_scan_low_point     = Av_tmp_container[idx].Av_scan_low_point;
            Av_object_container[idx].Av_scan_high_point    = Av_tmp_container[idx].Av_scan_high_point;
            Av_object_container[idx].Av_object_range_min   = Av_tmp_container[idx].Av_object_range_min;
        }
    }
    /* No objects detected, clear array */
    else if (tmp_array_nonempty == False)
    {
        AV_CLEAR_OBJ_ARRAY();
    }
    else
    {
        /* Do nothing */
    }
    Av_num_of_objects = AV_NUM_OF_OBJECTS();

}


/***********************************************/
/*       LIDAR SIG PROCESSING                  */
/***********************************************/
void AV_SENSING_LIDAR_CALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO(" !! Entered AV_SENSING_LIDAR_CALLBACK");
    bool object_detected;
    signed int num_of_objects;

    object_detected = False;
    num_of_objects = -1;

    AV_CLEAR_TMP_ARRAY();

    for (int scan_idx=0; scan_idx < 640; scan_idx++)
    {
        if(   (scan->ranges[scan_idx] <= scan->range_max)
            && (scan->ranges[scan_idx] >= scan->range_min)
            && (num_of_objects         <  AvMaxObjects)
        )
        {
            object_detected = True;
            Av_scan_detection = True;

            if(Av_scan_detection_LL == False)
            {
                ++num_of_objects;
                Av_tmp_container[num_of_objects].Av_scan_low_point              = scan_idx;
                Av_tmp_container[num_of_objects].Av_obj_id                      = num_of_objects;
                Av_tmp_container[num_of_objects].Av_object_proximity            = AvProximityUnknown;
                Av_tmp_container[num_of_objects].Av_object_in_front             = False;
                Av_tmp_container[num_of_objects].Av_object_range_min            = ((scan->range_max) + 1); /* initialize with large number */
            }

            if(   (Av_tmp_container[num_of_objects].Av_object_range_min > (scan->ranges[scan_idx]))
               && (scan->ranges[scan_idx] > 0)
              )
            {
                Av_tmp_container[num_of_objects].Av_object_range_min = scan->ranges[scan_idx];
            }

        }
        else
        {
            Av_scan_detection = False;
            if(Av_scan_detection_LL == True)
            {
                Av_tmp_container[num_of_objects].Av_scan_high_point = scan_idx - 1;
            }
        }

        Av_scan_detection_LL = Av_scan_detection;
    }

    if(object_detected == False) /* No object within range*/
    {
        AV_CLEAR_OBJ_ARRAY();
    }
}

/***********************************************/
/*        ODOM SIG PROCESSING                  */
/***********************************************/
void AV_SENSING_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom)
{
    //ROS_INFO(" !! Entered AV_SENSING_ODOMETRY_CALLBACK");
    Av_orientation.pose_x = odom->pose.pose.position.x;
    Av_orientation.pose_y = odom->pose.pose.position.y;
    Av_orientation.pose_z = odom->pose.pose.position.z;
    Av_orientation.orient = odom->pose.pose.orientation;
}

/***********************************************/
/*        CAMERA IMAGE PROCESSING              */
/***********************************************/
void AV_SENSING_IMAGE_CALLBACK(const sensor_msgs::Image::ConstPtr& img_scan)
{
    ROS_INFO(" !! Entered AV_SENSING_IMAGE_CALLBACK");
    /* convert to openCV somehow */
    try
    {
        Av_cv_ptr = cv_bridge::toCvCopy(img_scan, sensor_msgs::image_encodings::BGR8);

        AV_DETECT_STOP_SIGN(Av_cv_ptr->image);
        AV_DETECT_LANE(Av_cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
   
}

/***********************************************/
/*        TMP ARRAY FUNCTIONS                  */
/***********************************************/
void AV_CLEAR_TMP_ARRAY()
{
    for(int idx=0; idx < AvMaxObjects; idx++)
    {
        Av_tmp_container[idx].Av_obj_id             = -1;
        Av_tmp_container[idx].Av_scan_low_point     = -1;
        Av_tmp_container[idx].Av_scan_high_point    = -1;
        Av_tmp_container[idx].Av_object_range_min   = -1.00;
        Av_tmp_container[idx].Av_object_in_front    = False;
        Av_tmp_container[idx].Av_object_proximity   = AvProximityUnknown;
    }
}

void AV_CLEAR_OBJ_ARRAY()
{
    for(int idx=0; idx < AvMaxObjects; idx++)
    {
        Av_object_container[idx].Av_obj_id             = -1;
        Av_object_container[idx].Av_scan_low_point     = -1;
        Av_object_container[idx].Av_scan_high_point    = -1;
        Av_object_container[idx].Av_object_range_min   = -1.00;
        Av_object_container[idx].Av_object_in_front    = False;
        Av_object_container[idx].Av_object_proximity   = AvProximityUnknown;
    }
}

bool AV_TMP_HAS_ELEMENTS()
{
    bool ret_val;

    if (Av_tmp_container[0].Av_obj_id != -1)
    {
        ret_val = True;
    }
    else
    {
        ret_val = False;
    }

    //ROS_INFO("AV_TMP_HAS_ELEMENTS return value: %d", ret_val);
    return ret_val;
}

unsigned int AV_NUM_OF_TMP_OBJECTS()
{
    int idx;

    idx = 0;

    while(Av_tmp_container[idx].Av_obj_id != -1)
    {
        idx++;
    }

    return idx;
}

unsigned int AV_NUM_OF_OBJECTS()
{
    int idx;

    idx = 0;

    while(Av_object_container[idx].Av_obj_id != -1)
    {
        idx++;
    }

    return idx;
}


