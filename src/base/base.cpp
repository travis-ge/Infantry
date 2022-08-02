//
// Created by quanyi on 2022/4/2.
//

#include "base.h"
#include "serial_port.h"
#include "common.h"

using namespace cv;
extern SerialPort port;
extern Ptz_infor stm;
extern std::mutex mtx_port;
AngleSolver::AngleSolver() {
    FileStorage fs(Param, cv::FileStorage::READ);
    fs["base"]["simple_pitch_diff"] >> simple_pitch_diff;
    fs["base"]["simple_yaw_diff"] >> simple_yaw_diff;
    fs["base"]["sentry_pitch_diff"] >> sentry_pitch_diff;
    fs["base"]["sentry_yaw_diff"] >> sentry_yaw_diff;
    fs["base"]["buff_pitch_diff"] >> buff_pitch_diff;
    fs["base"]["buff_yaw_diff"] >> buff_yaw_diff;
    fs.release();
    trans <<1,0,0,  0.0405,                             //x
            0,1,0,-0.0072,   //-0.0455           //y
            0,0,1,0.0809,  //0.1415              //z
            0,0,0,1;

//    trans <<1,0,0,  0.0,                             //x
//            0,1,0,-0.0455,   //-0.0455           //y
//            0,0,1,0.14155,  //0.1415              //z
//            0,0,0,1;
}

AngleSolver::~AngleSolver() {
    ;
}
bool AngleSolver::getRotX(double pitch) {
    RotX << 1,             0,             0, 0,
            0, cos(pitch), -sin(pitch), 0,
            0, sin(pitch), cos(pitch),  0,
            0,             0,             0,  1;
    return true;
}
bool AngleSolver::getRotY(double yaw) {
    RotY << cos(yaw), 0, sin(yaw), 0,
            0,           1,           0, 0,
            -sin(yaw), 0, cos(yaw), 0,
            0,            0,          0, 1;
    return true;
}
bool AngleSolver::getRotZ(double roll) {
    RotZ << cos(roll), -sin(roll), 0, 0,
            sin(roll), cos(roll), 0, 0,
            0,                       0, 1, 0,
            0,                       0, 0, 1;
    return true;
}
/**
 *    left 40.5mm     front 80.9
 * @param camPoint
 * @param stm
 * @return
 */
cv::Point3f AngleSolver::cam2abs(cv::Point3f camPoint, Ptz_infor stm) {
    if(!getRotX(-stm.pitch)){return cv::Point3f(0,0,0);}
    if(!getRotY(-stm.yaw)){return cv::Point3f(0,0,0);}
    if(!getRotZ(0)){return cv::Point3f(0,0,0);}
    pointMat << camPoint.x, camPoint.y, camPoint.z, 1;
    auto result =   trans*RotY*RotX*RotZ*pointMat; // y x z
    return cv::Point3f (result(0,0),result(1,0),result(2,0));
}
/**
 *
 * @param absPoint
 * @param stm
 * @return
 */
cv::Point3f AngleSolver::abs2cam(cv::Point3f absPoint, Ptz_infor stm) {
    if(!getRotX(-stm.pitch)){return cv::Point3f(0,0,0);}
    if(!getRotY(-stm.yaw)){return cv::Point3f(0,0,0);}
    if(!getRotZ(0)){return cv::Point3f(0,0,0);}
    absPointMat << absPoint.x, absPoint.y, absPoint.z ,1;
    auto T = trans*RotY*RotX*RotZ;
    auto camPointMat = T.inverse()*absPointMat;
    return cv::Point3f (camPointMat(0,0),camPointMat(1,0),camPointMat(2,0));

}

cv::Point3f AngleSolver::cam2gun(cv::Point3f cam_, cv::Point3f diff) {
    return cv::Point3f(cam_.x + diff.x, cam_.y + diff.y, cam_.z + diff.z);
}

cv::Point3f AngleSolver::gun2cam(cv::Point3f gun_, cv::Point3f diff) {
    return cv::Point3f(gun_.x - diff.x, gun_.y - diff.y, gun_.z - diff.z);
}

cv::Point3f AngleSolver::gun2ptz(cv::Point3f gun_, cv::Point3f diff) {
    return cv::Point3f(gun_.x + diff.x, gun_.y + diff.y, +gun_.z + diff.z);
}



/**
 *
 * @param cam_
 * @param pitch
 * @param yaw
 * @param Dis
 */
void AngleSolver::getAngle(cv::Point3f cam_, double &pitch, double &yaw, double &Dis) {
    cv::Point3f gun_ = cam2gun(cam_, cam2gunDiff);
    pitch = atan(gun_.y / sqrt(gun_.x * gun_.x + gun_.z * gun_.z));//more big more down
    yaw = atan(gun_.x / gun_.z); //more big more right
    Dis = sqrt(gun_.x * gun_.x + gun_.z * gun_.z +gun_.y*gun_.y);
    mtx_port.lock();
    double ptz_pitch = stm.pitch;
    double d_speed = stm.bulletSpeed;
    char mode = stm.mode;
    mtx_port.unlock();
    //std::cout<<"ppppppppppppppppppppppppp "<<ptz_pitch<<std::endl;
    abs_pitch = -1 * pitch - ptz_pitch;
    //abs_pitch = pitch + ptz_pitch;
    int if_fix = 1;
    if (if_fix) {
        //abs_pitch = -abs_pitch;
        double angle_fix = 0.5 *
                           (asin((9.8 * Dis * pow(cos(abs_pitch), 2)) / pow(d_speed, 2) - sin(abs_pitch)) +
                            abs_pitch);
//        std::cout << "angle fix " << angle_fix << std::endl;
        abs_pitch += angle_fix;
    }
    std::cout<<"ssssssss"<<std::endl;
    pitch = -abs_pitch - ptz_pitch;
    if (mode == 'b' || mode == 'a' || d_speed > 25) {
        std::cout << "buff add666" << std::endl;
        pitch += buff_pitch_diff;//0.01
        yaw += buff_yaw_diff;//0.001
    }else{
        pitch += simple_pitch_diff;//-0.010
        yaw += simple_yaw_diff;//-0.023
    }

}
void AngleSolver::getAngle_nofix(cv::Point3f cam_, double &pitch, double &yaw, double &Dis) {
    cv::Point3f gun_ = cam2gun(cam_, cam2gunDiff);
    pitch = atan(gun_.y / sqrt(gun_.x * gun_.x + gun_.z * gun_.z));//more big more down
    yaw = atan(gun_.x / gun_.z); //more big more right
    Dis = sqrt(gun_.x * gun_.x + gun_.z * gun_.z +gun_.y*gun_.y);
}
