/*
 * av_perception.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_perception.h"

void AV_PERCEPTION_INIT(void)
{
    Av_CollisionWarning = false;
    Av_StopSignDet      = false;
}

void AV_PERCEPTION(void)
{
    ROS_INFO("-- Entered AV_PERCEPTION");
    if(Av_num_of_objects > 0)
    {
        AV_SET_OBJECT_PROXIMITY();  /* Set their proximity state */
        AV_DETECT_FRONT_OBJECT();   /* Detect those that are in front of the AV */
        AV_COLLISION_MONITORING();
    }
    else
    {
        /* Nothing to do */
    }
}


void AV_SET_OBJECT_PROXIMITY()
{
    int idx;

    for(idx = 0; idx < Av_num_of_objects; idx++)
    {
        if(Av_object_container[idx].Av_object_range_min >= AvProxyFarThr) /* If range is really far, don't bother with state check */
        {
            Av_object_container[idx].Av_object_proximity = AvProximityFar;
        }
        else
        {
            switch(Av_object_container[idx].Av_object_proximity)
            {
            case AvProximityFar:
                if (Av_object_container[idx].Av_object_range_min >= AvProxyCloseThr)
                {
                    continue; /* Stay in Far state */
                }
                else if( Av_object_container[idx].Av_object_range_min < AvProxyDngrThr)
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityDanger;
                }
                else if( Av_object_container[idx].Av_object_range_min < AvProxyCloseThr)
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityClose;
                }
                else
                {
                    /* Stay in Far state */
                }
                break;
            case AvProximityClose:
                if (   (Av_object_container[idx].Av_object_range_min >= AvProxyDngrThr)
                    && (Av_object_container[idx].Av_object_range_min <= AvProxyCloseThr)
                )
                {
                    continue; /* Stay in Close state */
                }
                else if( Av_object_container[idx].Av_object_range_min < (AvProxyDngrThr - AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityDanger;
                }
                else if(Av_object_container[idx].Av_object_range_min > (AvProxyCloseThr + AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else
                {
                    /* Stay in Close state */
                }
                break;
            case AvProximityDanger:
                if (Av_object_container[idx].Av_object_range_min <= AvProxyDngrThr)
                {
                    continue; /* Stay in Close state */
                }
                else if(Av_object_container[idx].Av_object_range_min > (AvProxyCloseThr + AvProxyHyst))
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else if(Av_object_container[idx].Av_object_range_min > (AvProxyDngrThr + AvProxyHyst))
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
                if( Av_object_container[idx].Av_object_range_min > AvProxyCloseThr )
                {
                    Av_object_container[idx].Av_object_proximity = AvProximityFar;
                }
                else if( Av_object_container[idx].Av_object_range_min > AvProxyDngrThr )
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
    char idx;
    #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
    ROS_INFO("Front Det entered");
    #endif /* AvDebugFrontDetInfoEnable */
    AvTmpAngleCorr = ((-1)*(Av_Arbitrated_Target_Yaw));

    AvFrontCheckHighMark = (AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr);
    AvFrontCheckLowMark = (AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr);
    ROS_INFO("HighMark: %d, LowMark: %d", AvFrontCheckHighMark, AvFrontCheckLowMark);

    for(idx = 0; idx < Av_num_of_objects; idx++)
    {
        if(   ((AvFrontCheckHighMark <= Av_object_container[idx].Av_scan_high_point) && (AvFrontCheckLowMark >= Av_object_container[idx].Av_scan_low_point)) /*TODO: implement turn angle correction*/
           || ((AvFrontCheckHighMark >= Av_object_container[idx].Av_scan_high_point) && (AvFrontCheckLowMark <= Av_object_container[idx].Av_scan_low_point))
           || ((AvFrontCheckHighMark >= Av_object_container[idx].Av_scan_high_point) && (AvFrontCheckLowMark <= Av_object_container[idx].Av_scan_high_point))
           || ((AvFrontCheckHighMark >= Av_object_container[idx].Av_scan_low_point)  && (AvFrontCheckLowMark <= Av_object_container[idx].Av_scan_low_point))
          )
        {
            #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
            ROS_INFO("Object ID: %d detected to the front!!", Av_object_container[idx].Av_obj_id);
            #endif /* AvDebugFrontDetInfoEnable */
            Av_object_container[idx].Av_object_in_front = True;
        }
        else
        {
            #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
            ROS_INFO("Object ID: %d is NOT in front!!", Av_object_container[idx].Av_obj_id);
            #endif /* AvDebugFrontDetInfoEnable */
            Av_object_container[idx].Av_object_in_front = False;
        }

    }
}

void AV_COLLISION_MONITORING()
{
    int idx;
    Av_CollisionWarning = False;

    for(idx = 0; idx < Av_num_of_objects; idx++)
    {
        if(   (Av_object_container[idx].Av_object_in_front == True)
           && (   (Av_object_container[idx].Av_object_proximity == AvProximityClose)
               || (Av_object_container[idx].Av_object_proximity == AvProximityDanger)
               || (Av_object_container[idx].Av_object_range_min < 5.0)
              )
          )
        {
            Av_CollisionWarning = True;
        }
    }
    ROS_INFO(" >> Collision warning: %d", Av_CollisionWarning);
}

void AV_DETECT_STOP_SIGN(Mat image)
{
    Scalar Av_lower_red(160,175,0);
    Scalar Av_upper_red(179,255,255);
    Mat img_hsv;
    Mat img_thr;
    
    Av_StopSignDet = False;

    try
    {
        cvtColor(image, img_hsv,  cv::COLOR_BGR2HSV);
        inRange(img_hsv, Scalar(Av_lower_red[0], Av_lower_red[1], Av_lower_red[2]), Scalar(Av_upper_red[0], Av_upper_red[1], Av_upper_red[2]), img_thr); //Threshold the image
            
        //morphological opening (remove small objects from the foreground)
        erode(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Moments oMoments = moments(img_thr);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 120000)
        {
            Av_StopSignDet = True;
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }

    ROS_INFO(" >> Stop sign detected: %d", Av_StopSignDet);
}

void AV_DETECT_LANE(Mat image)
{
    Scalar lower_black(0,0,0);
    Scalar upper_black(180,255,30);
    Scalar lower_yellow(20, 100, 100);
    Scalar upper_yellow(30, 255, 255);
    Scalar lower_white(0,0,200);
    Scalar upper_white(0,0,255);
    Mat img_hsv, img_gray;
    Mat img_thr_road, img_thr_lane, img_thr_wht;
    Mat img_band, img_gauss_gray, img_something;
    Mat img_canny;

    int img_height, img_width;

    vector<Vec4i> lines;

    try
    {
        img_width = image.size().width;
        img_height = image.size().height;

        cvtColor(image, img_hsv, COLOR_BGR2HSV);
        inRange(img_hsv, lower_black, upper_black, img_thr_road); //Threshold the image
        Moments mom = moments(img_thr_road, false);

        AV_LINE_FOLLOW(mom, img_height, img_width);  
        

        cvtColor(image, img_gray, COLOR_BGR2GRAY);
        inRange(img_hsv, lower_yellow, upper_yellow, img_thr_lane);
        inRange(img_gray, lower_white, upper_white, img_thr_wht);
        bitwise_or(img_thr_wht, img_thr_lane, img_something, noArray());
        bitwise_and(img_gray, img_something ,img_band, noArray());
        GaussianBlur(img_band, img_gauss_gray, Size(5,5), 3, 3, BORDER_DEFAULT);		

        Canny( img_gauss_gray, img_canny, 50, 150, 3 );
        HoughLinesP(img_canny, lines, 1, CV_PI/180, 80, 30, 10 );

    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}