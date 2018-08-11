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



bool                          Av_object_detected;
unsigned int                  Av_num_of_tmp;
unsigned int                  Av_num_of_tmp_LL;
bool                          Av_scan_detection;
bool                          Av_scan_detection_LL;
Av_objects_t                  Av_tmp_container[AvMaxObjects];
cv_bridge::CvImagePtr         Av_cv_ptr;
cv::Mat                       FrontCamImageIHLS;

// Maximum between three values
inline float get_maximum(const float& r, const float& g, const float& b) 
{ 
    return (r >= g) ? ((r >= b) ? r : b) : ((g >= b) ? g : b); 
}

// Minimum between three values
inline float get_minimum(const float& r, const float& g, const float& b) 
{ 
    return (r <= g) ? ((r <= b) ? r : b) : ((g <= b) ? g : b); 
}


// Theta computation
inline float retrieve_theta(const float& r, const float& g, const float& b) { return acos((r - (g * 0.5) - (b * 0.5)) / sqrtf((r * r) + (g * g) + (b * b) - (r * g) - (r * b) - (g * b))); }
// Hue computation -- H = θ if B <= G -- H = 2 * pi − θ if B > G
inline float retrieve_normalised_hue(const float& r, const float& g, const float& b) { return (b <= g) ? (retrieve_theta(r, g, b) * 255.f / (2.f * M_PI)) : (((2.f * M_PI) - retrieve_theta(r, g, b)) * 255.f / (2.f * M_PI)); }
// Luminance computation -- L = 0.210R + 0.715G + 0.072B
inline float retrieve_luminance(const float& r, const float& g, const float& b) { return (0.210f * r) + (0.715f * g) + (0.072f * b); }
// Saturation computation -- S = max(R, G, B) − min(R, G, B)
inline float retrieve_saturation(const float& r, const float& g, const float& b) { return (get_maximum(r, g, b) - get_minimum(r, g, b)); }

void AV_CLEAR_TMP_ARRAY(void);
void AV_CLEAR_OBJ_ARRAY(void);

unsigned int  AV_NUM_OF_TMP_OBJECTS(void);
unsigned int  AV_NUM_OF_OBJECTS(void);
bool AV_TMP_HAS_ELEMENTS(void);

void AV_SENSORICS_ODOMETRY_CALLBACK(const nav_msgs::Odometry::ConstPtr& odom);

#endif /* SRC_AV_CONTROL_AV_SENSING_H_ */
