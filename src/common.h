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
/**
 *
 */

#define INFANTRY6
/****/
#ifdef INFANTRY3
#define
#elifdef INFANTRY4
#define
#elifdef INFANTRY5
#define
#elifdef INFANTRY6
#define cameraParam PROJECT_DIR"/config/cameraParams_332.xml"
#define Param PROJECT_DIR"/config/params6.yml"
#endif
/**
 *
 */
enum STATE{
    STATE_AUTOAIM = 1,
    STATE_BUFF = 2,
    STATE_SENTRY = 3,
    STATE_TOP = 4,
    STATE_RECORD = 5
};


STATE getMode(char infor);

/** =============================================**/
//#define SHOW_DST
//#define WRITE_IMG
#endif //CAM_POSE_CALIB_COMMON_H

