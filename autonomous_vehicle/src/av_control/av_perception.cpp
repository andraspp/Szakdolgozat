/*
 * av_perception.c
 *
 *  Created on: 2018. j�l. 4.
 *      Author: pappa3
 */

#include "av_perception.h"


void AV_PERCEPTION(void)
{
    ROS_INFO("Entered AV_PERCEPTION");
    if(Av_num_of_objects > 0)
    {
        AV_SET_OBJECT_PROXIMITY();  /* Set their proximity state */
        AV_DETECT_FRONT_OBJECT();   /* Detect those that are in front of the AV */
    }
    else
    {
        /* Nothing to do */
    }
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
    #if((AvDebugConfig & AvDebugFrontDetInfoEnable) > 0)
    ROS_INFO("Front Det entered");
    #endif /* AvDebugFrontDetInfoEnable */

    for(int idx; idx < Av_num_of_objects; idx++)
    {
        if(   ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_low_point  <= (AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr)) /*TODO: implement turn angle correction*/
           || ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_high_point >= (AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr))
           || (   ((AvFrontAngle + AvFrontAngleOffset + AvTmpAngleCorr) <= Av_object_container[idx].Av_scan_high_point)
               && ((AvFrontAngle - AvFrontAngleOffset + AvTmpAngleCorr) >= Av_object_container[idx].Av_scan_low_point)
              )
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
