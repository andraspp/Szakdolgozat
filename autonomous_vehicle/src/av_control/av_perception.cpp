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
    double dM10, dM01, dArea;
    
    Av_StopSignDet = False;

    try
    {
        cvtColor(image, img_hsv,  cv::COLOR_BGR2HSV);
        inRange(img_hsv, Av_lower_red, Av_upper_red, img_thr); //Threshold the image
            
        //morphological opening (remove small objects from the foreground)
        /*erode(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );*/ 

        //morphological closing (fill small holes in the foreground)
        /*dilate(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(img_thr, img_thr, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );*/

        Moments oMoments = moments(img_thr);

        //bitwise_and(image,img_thr,image,noArray());
        namedWindow("StopSign",1);
        imshow("StopSign", img_thr);
        waitKey(1);

        dM01 = oMoments.m01;
        dM10 = oMoments.m10;
        dArea = oMoments.m00;

        if (dArea > AvStopSignAreaThreshold)
        {
            Av_StopSignDet = True;
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }

    ROS_INFO(" >> StopSign area: %f, Stop sign detected: %d", dArea, Av_StopSignDet);
}

void AV_DETECT_LANE(Mat image)
{
    Scalar lower_black(0,0,0);
    Scalar upper_black(180,255,30);
    Scalar lower_white(0,25,0);
    Scalar upper_white(0,100,0);
    Mat img_hsv, img_hsl;
    Mat mask_b, mask_w;
    Mat img_canny, img_lines, img_weighted;

    int img_height, img_width;
    int i;
    double avg;

    vector<Vec4i> lines;
    Vec4i v;

    try
    {
        img_width = image.size().width;
        img_height = image.size().height;

        cvtColor(image, img_hsv, COLOR_BGR2HSV);
        inRange(img_hsv, lower_black, upper_black, mask_b); //Threshold the image
 
        

        cvtColor(image, img_hsl, COLOR_BGR2HLS);
        inRange(img_hsl, lower_white, upper_white, mask_w);
        //bitwise_and(image, image, rgb_w, mask_w);
        //cvtColor(rgb_w, rgb_w, COLOR_BGR2GRAY);
        //threshold(rgb_w, rgb_w, 20,255,THRESH_BINARY );
       	

        bitwise_or(mask_w, mask_b, mask_b);

        Moments mom_b = moments(mask_b, false);

        AV_LINE_FOLLOW(mom_b, img_height, img_width); 

        GaussianBlur(mask_w, mask_w, Size(3,3), 3, 3, BORDER_DEFAULT);
        Canny( mask_w, img_canny, 50, 150, 3 );

        HoughLinesP(img_canny, lines, AvHoughRho, AvHoughTheta, AvHoughThreshold, AvHoughMinLineLength, AvHoughMaxLineGap );

        avg = 0;
        for (i = 0; i < lines.size(); i++)
        {
            v = lines[i];
            line(image, Point(v[0],v[1]), Point(v[2],v[3]), Scalar(0,255,0), 2, LINE_8, 0 );
            if((v[1]-v[0]) == 0)
            {
                slope = 999;
            }
            else
            {
                slope = ((v[3] - v[2])/(v[1] - v[0]));
                
                if(avg == 0)
                {
                    avg += slope;
                }
                else
                {
                    
                }
            }
            
        }


        /**namedWindow("Mask White",1);
        imshow("Mask White", mask_w);
        waitKey(1);*/

        namedWindow("Canny",1);
        imshow("Canny", img_canny);
        waitKey(1);

        namedWindow("Weighted",1);
        imshow("Weighted", image);
        waitKey(1);  

    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}