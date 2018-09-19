#ifndef AV_PARAMS_H
#define AV_PARAMS_H

/* AV_SENSING parameters*/
#define AvFrontAngle                    (320)
#define AvFrontAngleOffset              (25)
#define AvMaxObjects                    (5)

/* AV_PERCEPTION parameters*/
#define AvStopSignAreaThreshold         (2500)
#define AvProxyFarThr                   (8)
#define AvProxyCloseThr                 (6)
#define AvProxyDngrThr                  (3)
#define AvProxyHyst                     (0.1)
#define AvHoughRho                      (1)
#define AvHoughTheta                    (CV_PI/180)
#define AvHoughThreshold                (15)
#define AvHoughMinLineLength            (10)
#define AvHoughMaxLineGap               (60)

/* AV_PLANNING parameters*/
#define AvThrottleBaseValue             (6.0)
#define AvYawLimit                      (6.0)
#define AvThrYawDiff                    (2.5)

/* AV_CONTROL parameters*/
#define AvSpeedGrad                     (0.3)
#define AvHardStopSpeedGrad             (0.7)

#endif