/*
 * av_perception.h
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#ifndef SRC_AV_CONTROL_AV_PERCEPTION_H_
#define SRC_AV_CONTROL_AV_PERCEPTION_H_

#include "av_control_common.h"
#include "tgmath.h"
#include "Polynomial.h"

int AvFrontCheckHighMark;
int AvFrontCheckLowMark;
int AvTmpAngleCorr;

void AV_SET_OBJECT_PROXIMITY(void);
void AV_DETECT_FRONT_OBJECT (void);
void AV_COLLISION_MONITORING(void);

#endif /* SRC_AV_CONTROL_AV_PERCEPTION_H_ */
