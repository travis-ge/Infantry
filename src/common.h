//
// Created by quanyi on 2021/12/27.
//

#ifndef CAM_POSE_CALIB_COMMON_H
#define CAM_POSE_CALIB_COMMON_H
#include <iostream>
#include <chrono>
#include <vector>
#include "camera_api.h"


#ifdef PATH
#define PROJECT_DIR PATH
#else
#define PROJECT_DIR ""
#endif
/**
 *
 * @param pFrame
 */
void Frame_0_ProcessRGB(GX_FRAME_CALLBACK_PARAM *pFrame);
void Camera_offline_deal(void *pUserParams);
extern double current_yaw ;
enum STATE{
    STATE_AUTOAIM = 1,
    STATE_BUFF = 2,
    STATE_SENTRY = 3,
    STATE_TOP = 4,
    STATE_RECORD = 5
};
struct Send{
    double pitch;
    double yaw;
    double dis;
};
STATE getMode(char infor);

/** =============================================**/
//#define WRITE_IMG

/**
 * PARAMS FILE SELECT
 * note: PC of infantry 4 has moved to infantry 5 (2022.7.30)
 */
/**==============  3 ================**/
//#define cameraParam PROJECT_DIR"/config/cameraParams_156.xml"
//#define Param PROJECT_DIR"/config/params3.yml"
//#define cameraSN "KE0200120156"
/**==============  4 ================**/
//#define cameraParam PROJECT_DIR"/config/cameraParams_061.xml"
//#define Param PROJECT_DIR"/config/params4.yml"
//#define cameraSN "KE0200100061"
/**==============  5 ================**/
#define cameraParam PROJECT_DIR"/config/cameraParams_157.xml"
#define Param PROJECT_DIR"/config/params5.yml"
#define cameraSN "KE0200120157"
/**==============  6 ================**/
//#define cameraParam PROJECT_DIR"/config/cameraParams_332.xml"
//#define Param PROJECT_DIR"/config/params6.yml"
//#define cameraSN "KE0220060332"
/**==============  End  ================**/
#endif //CAM_POSE_CALIB_COMMON_H

