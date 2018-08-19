/*
 * av_sensing.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */
#include "av_sensing.h"

using namespace cv;
using namespace std;

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
    
    //ROS_INFO("Entered AV_SENSING");
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
    ROS_INFO("Entered AV_SENSING_LIDAR_CALLBACK");
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

    #if((AvDebugConfig & AvDebugStoredObjectsInfoEnable) > 0)
    ROS_INFO("temp num of objects: %d, stored num of objects: %d", num_of_objects, Av_num_of_objects);
    ROS_INFO("Object 0's ID: %d", Av_tmp_container[0].Av_obj_id);
    #endif /* AvDebugStoredObjectsInfoEnable */

}

/***********************************************/
/*        ODOM SIG PROCESSING                  */
/***********************************************/
void AV_SENSING_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom)
{
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
    Scalar Av_lower_red(160,175,0);
    Scalar Av_upper_red(179,255,255);
    Scalar box;
    RotatedRect rect;

    Av_StopSignDet = 0;

    ROS_INFO("Entered AV_SENSING_IMAGE_CALLBACK");
    /* convert to openCV somehow */
    try
    {
        Av_cv_ptr = cv_bridge::toCvCopy(img_scan, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    try
    {
        //cvtColor(Av_cv_ptr->image, Av_img_gray, cv::COLOR_BGR2GRAY);
        cvtColor(Av_cv_ptr->image, Av_img_hsv,  cv::COLOR_BGR2HSV);

        inRange(Av_img_hsv, Scalar(Av_lower_red[0], Av_lower_red[1], Av_lower_red[2]), Scalar(Av_upper_red[0], Av_upper_red[1], Av_upper_red[2]), mask); //Threshold the image
            
        //morphological opening (remove small objects from the foreground)
        erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Moments oMoments = moments(mask);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 30000)
        {
            Av_StopSignDet = 1;
        }

        //morphologyEx(mask, mask, MORPH_OPEN, Mat::ones(100, 100, CV_8U), Point(-1,-1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
        //morphologyEx(mask, mask, MORPH_CLOSE, Mat::ones(100, 100, CV_8U), Point(-1,-1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
        //findContours(mask, Av_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    }
    catch( Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }

    ROS_INFO("Stop sign detected: %d", Av_StopSignDet);
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


