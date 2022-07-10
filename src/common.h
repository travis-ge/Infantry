//
// Created by quanyi on 2021/12/27.
//

#ifndef CAM_POSE_CALIB_COMMON_H
#define CAM_POSE_CALIB_COMMON_H
#include "camera_api.h"
#include "chrono"

#ifdef PATH
#define PROJECT_DIR PATH
#else
#define PROJECT_DIR ""
#endif

enum STATE{
    STATE_AUTOAIM = 1,
    STATE_BUFF = 2,
    STATE_SENTRY = 3,
    STATE_TOP = 4,
    STATE_RECORD = 5
};

STATE getMode(char infor);
void Frame_0_ProcessRGB(GX_FRAME_CALLBACK_PARAM *pFrame);
#endif //CAM_POSE_CALIB_COMMON_H

