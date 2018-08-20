/*
 * av_perception.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_perception.h"


void AV_PERCEPTION(void)
{
    //ROS_INFO("Entered AV_PERCEPTION");
    if(Av_num_of_objects > 0)
    {
        AV_SET_OBJECT_PROXIMITY();  /* Set their proximity state */
        AV_DETECT_FRONT_OBJECT();   /* Detect those that are in front of the AV */
    }
    else
    {
        /* Nothing to do */
    }

    ROS_INFO("Object Info @ PERCEPTION");
    ROS_INFO("Num of objects PERC: %d", Av_num_of_objects);
    ROS_INFO("ID: %d. LP: %d, HP: %d, MINR: %lf, Fr: %d, Prox: %d", Av_object_container[0].Av_obj_id, Av_object_container[0].Av_scan_low_point, Av_object_container[0].Av_scan_high_point, Av_object_container[0].Av_object_range_min, Av_object_container[0].Av_object_in_front, Av_object_container[0].Av_object_proximity);
    ROS_INFO("ID: %d. LP: %d, HP: %d, MINR: %lf, Fr: %d, Prox: %d", Av_object_container[1].Av_obj_id, Av_object_container[1].Av_scan_low_point, Av_object_container[1].Av_scan_high_point, Av_object_container[1].Av_object_range_min, Av_object_container[1].Av_object_in_front, Av_object_container[1].Av_object_proximity);
}


void AV_SET_OBJECT_PROXIMITY()
{
    for(int idx; idx < Av_num_of_objects; idx++)
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

void AV_DETECT_STOP_SIGN(Mat image)
{
    Scalar Av_lower_red(160,175,0);
    Scalar Av_upper_red(179,255,255);
    
    Av_StopSignDet = False;

    try
    {
        inRange(image, Scalar(Av_lower_red[0], Av_lower_red[1], Av_lower_red[2]), Scalar(Av_upper_red[0], Av_upper_red[1], Av_upper_red[2]), Av_img_threshold); //Threshold the image
            
        //morphological opening (remove small objects from the foreground)
        erode(Av_img_threshold, Av_img_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(Av_img_threshold, Av_img_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        //morphological closing (fill small holes in the foreground)
        dilate(Av_img_threshold, Av_img_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(Av_img_threshold, Av_img_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Moments oMoments = moments(Av_img_threshold);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        if (dArea > 30000)
        {
            Av_StopSignDet = True;
        }
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }

    ROS_INFO("Stop sign detected: %d", Av_StopSignDet);
}