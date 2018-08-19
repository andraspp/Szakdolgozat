/*
 * av_sensing.h
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#ifndef SRC_AV_CONTROL_AV_SENSING_H_
#define SRC_AV_CONTROL_AV_SENSING_H_

#include "av_control_common.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>


bool                          Av_object_detected;
unsigned int                  Av_num_of_tmp;
unsigned int                  Av_num_of_tmp_LL;
unsigned char                 Av_StopSignDet;
bool                          Av_scan_detection;
bool                          Av_scan_detection_LL;
Av_objects_t                  Av_tmp_container[AvMaxObjects];
cv_bridge::CvImagePtr         Av_cv_ptr;
cv::Mat                       Av_img_gray, Av_img_blurred, Av_img_edged, Av_img_hsv;
cv::Mat                       mask, temp_img;
std::vector<cv::Vec4i>            Av_hierarchy;
std::vector<std::vector<cv::Point> >   Av_contours;


void AV_CLEAR_TMP_ARRAY(void);
void AV_CLEAR_OBJ_ARRAY(void);

unsigned int  AV_NUM_OF_TMP_OBJECTS(void);
unsigned int  AV_NUM_OF_OBJECTS(void);
bool AV_TMP_HAS_ELEMENTS(void);

void AV_SENSORICS_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom);

#endif /* SRC_AV_CONTROL_AV_SENSING_H_ */
