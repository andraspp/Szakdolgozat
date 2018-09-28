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
    int low_angle_diff, high_angle_diff;

    Av_CollisionWarning = False;
    Av_SideWayCollWarning = False;

    for(idx = 0; idx < Av_num_of_objects; idx++)
    {
        if (   (Av_object_container[idx].Av_object_in_front == True)
            && (   (Av_object_container[idx].Av_object_proximity == AvProximityClose)
                || (Av_object_container[idx].Av_object_proximity == AvProximityDanger)
                || (Av_object_container[idx].Av_object_range_min < 5.0)
               )
           )
        {
            Av_CollisionWarning = True;
        }

        low_angle_diff = Av_object_container[idx].Av_scan_low_point - Av_scan_low_points_LL[idx];
        high_angle_diff = Av_object_container[idx].Av_scan_high_point - Av_scan_high_points_LL[idx];

        if (   (Av_scan_low_points_LL[idx] > AvCenterLine)
            && (low_angle_diff <= 0)
            && (high_angle_diff <= 0)
            && (Av_Lane_Target_Yaw < 0.1)
           )
        {
            Av_SideWayCollWarning = True;
        }
        else if (   (Av_scan_high_points_LL[idx] < AvCenterLine)
                 && (low_angle_diff >= 0)
                 && (high_angle_diff >= 0)
                 && (Av_Lane_Target_Yaw < 0.1)
                )
        {
            Av_SideWayCollWarning = True;
        }
        else
        {
            
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
    double temp_thr;
    
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

        if ( Av_Lane_Target_Speed == 0.0 )
        {
            temp_thr = (double)(AvStopSignAreaThreshold * 8.0);
        }
        else if ( Av_Lane_Target_Speed <= 2.0 )
        {
            temp_thr = (double)(AvStopSignAreaThreshold * 7.0);
        }
        else if ( Av_Lane_Target_Speed <= 3.0 )
        {
            temp_thr = (double)(AvStopSignAreaThreshold * 6.0);
        }
        else if ( Av_Lane_Target_Speed <= 4.0 )
        {
            temp_thr = (double)(AvStopSignAreaThreshold * 5.0);
        }
        else
        {
            temp_thr = (double)(AvStopSignAreaThreshold * 4.0);
        }

        if (dArea > temp_thr)
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

    int img_height, img_width, img_center;
    int x1, x2, y1, y2;
    double avg, slope, poly_left, pply_right;

    vector<Vec4i> lines;
    vector<double> slopes;
    vector<Vec4i> left_lines, right_lines;
    vector<Point> right_pts;
    vector<Point> left_pts;
    Vec4d right_line;
    Vec4d left_line;
    double right_m, left_m;
    Point right_b, left_b;
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
        img_center = static_cast<double>((image.cols / 2));

        for (int i = 0; i < lines.size(); i++)
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

                x1 = v[0];
                x2 = v[1];
                y1 = v[2];
                y2 = v[3];
                
                if( fabs(slope) < 0.5 )
                    continue;
                
                slopes.push_back(slope);
                

                if(   ( slope < 0 )
                   && ( img_center > x1 )
                   && ( img_center > x2 )
                  )
                {
                    left_lines.push_back(Vec4i(x1,x2,y1,y2));
                }
                else if (   ( slope > 0 )
                         && ( img_center < x1 )
                         && ( img_center < x2 )
                        )
                {
                    right_lines.push_back(Vec4i(x1,x2,y1,y2));
                }
                else
                {

                }
            }
        }

        for(int i = 0; i < left_lines.size(); i++)
        {
            Point start, end;
            Vec4i v = left_lines[i];

            start = Point(v[0], v[1]);
            end = Point(v[2], v[3]);

            left_pts.push_back(start);
            left_pts.push_back(end);
        }

        for(int i = 0; i < right_lines.size(); i++)
        {
            Point start, end;
            Vec4i c = right_lines[i];

            start = Point(c[0], c[1]);
            end = Point(c[2], c[3]);

            right_pts.push_back(start);
            right_pts.push_back(end);
        }

        if (left_pts.size() > 0) 
        {
            // The left line is formed here
            fitLine(left_pts, left_line, CV_DIST_L2, 0, 0.01, 0.01);
            left_m = left_line[1] / left_line[0];
            left_b = Point(left_line[2], left_line[3]);
        }

        if (right_pts.size() > 0) 
        {
            // The right line is formed here
            fitLine(right_pts, right_line, CV_DIST_L2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];
            right_b = cv::Point(right_line[2], right_line[3]);
        }

        int ini_y = image.rows;
        int fin_y = 470;

        double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
        double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

        double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
        double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

        Mat output;
        vector<Point> poly_points;

        // Create the transparent polygon for a better visualization of the lane
        image.copyTo(output);
        poly_points.push_back(Point(left_ini_x,ini_y));
        poly_points.push_back(Point(right_ini_x,ini_y));
        poly_points.push_back(Point(right_fin_x,ini_y));
        poly_points.push_back(Point(left_fin_x,ini_y));
        fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), CV_AA, 0);
        addWeighted(output, 0.3, image, 1.0 - 0.3, 0, image);

        // Plot both lines of the lane boundary
        line(image, Point(right_ini_x,ini_y), Point(right_fin_x,ini_y), cv::Scalar(0, 255, 255), 5, CV_AA);
        line(image, Point(left_ini_x,ini_y), Point(left_fin_x,ini_y), cv::Scalar(0, 255, 255), 5, CV_AA);

        // Show the final output image
        namedWindow("Lane", CV_WINDOW_AUTOSIZE);
        imshow("Lane", image);

        /**namedWindow("Mask White",1);
        imshow("Mask White", mask_w);
        waitKey(1);*/

        namedWindow("Canny",1);
        imshow("Canny", img_canny);
        waitKey(1);

    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}