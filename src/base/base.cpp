//
// Created by quanyi on 2022/4/2.
//

#include "base.h"
#include "serial_port.h"
#include "common.h"

using namespace cv;
extern SerialPort port;
extern Ptz_infor stm;

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

    double ptz_pitch = stm.pitch;
    //std::cout<<"ppppppppppppppppppppppppp "<<ptz_pitch<<std::endl;
    abs_pitch = -1 * pitch - ptz_pitch;
    //abs_pitch = pitch + ptz_pitch;
    int if_fix = 1;
    if (if_fix) {
        //abs_pitch = -abs_pitch;
        double angle_fix = 0.5 *
                           (asin((9.8 * Dis * pow(cos(abs_pitch), 2)) / pow(stm.bulletSpeed, 2) - sin(abs_pitch)) +
                            abs_pitch);
//        std::cout << "angle fix " << angle_fix << std::endl;
        abs_pitch += angle_fix;
    }
    //std::cout<<"ssssssss"<<std::endl;
    pitch = -abs_pitch - ptz_pitch;
    if (port.receive[1] == 'h') {
//        std::cout << "aaaaaaaaaaaaaaaaa" << std::endl;
        pitch += sentry_pitch_diff;//-0.012
        yaw += sentry_yaw_diff;//-0.023
    } else {
//        std::cout << "3ss" << std::endl;
        pitch += simple_pitch_diff;//-0.010
        yaw += simple_yaw_diff;//-0.023
    }
    if (port.receive[1] == 'b' || port.receive[1] == 'a') {
        std::cout << "buff add666" << std::endl;
        pitch += buff_pitch_diff;//0.01
        yaw += buff_yaw_diff;//0.001
    }
}
void AngleSolver::getAngle_nofix(cv::Point3f cam_, double &pitch, double &yaw, double &Dis) {
    cv::Point3f gun_ = cam2gun(cam_, cam2gunDiff);
    pitch = atan(gun_.y / sqrt(gun_.x * gun_.x + gun_.z * gun_.z));//more big more down
    yaw = atan(gun_.x / gun_.z); //more big more right
    Dis = sqrt(gun_.x * gun_.x + gun_.z * gun_.z +gun_.y*gun_.y);
}

///==================================== AngleSolver End ==================================================///


/***********************************************
 * @descrip
 * @param
 * @return
 * @date
 *************************************************/
/////////////// AngleFilter ///////////////

/**
 *
 * @param a_process_noise
 * @param a_measure_noise
 * @param a_val_diff
 */
Kalman::Kalman(double a_process_noise, double a_measure_noise, double a_val_diff)
{
    setParam();
    m_KF = std::make_shared<cv::KalmanFilter>(m_state_num, m_measure_num, 0);
    m_measurement = cv::Mat::zeros(m_measure_num, 1, CV_32F);
    cv::setIdentity(m_KF->measurementMatrix);
    cv::setIdentity(m_KF->processNoiseCov, cv::Scalar::all(a_process_noise));
    cv::setIdentity(m_KF->measurementNoiseCov, cv::Scalar::all(a_measure_noise));
    cv::setIdentity(m_KF->errorCovPost, cv::Scalar::all(1));
    float dt = 1.f / m_control_freq;
    m_KF->transitionMatrix = (cv::Mat_<float>(m_state_num, m_state_num) <<
                              1, dt,
                              0, 1);
    m_val_diff = a_val_diff;
}

/**
 *
 */
void Kalman::setParam() {

    m_debug;
    m_init_count_threshold;
    m_predict_coe;
    m_control_freq;
}

/**
 *
 * @param a_val
 */
void Kalman::initFilter(double a_val) {
    m_KF->statePost = (cv::Mat_<float>(m_state_num, 1) << a_val, 0);
    m_KF->predict();
    m_measurement.at<float>(0) = a_val;

    for (int i = 0; i < m_init_count_threshold; i++) {
        m_KF->correct(m_measurement);
        m_KF->predict();
    }
    std::cout << "Init" << std::endl;
}

/**
 *
 * @param a_val
 * @return
 */
double Kalman::predict(double a_val) {
    cv::Mat result;

    if (!m_initialized) {
        initFilter(a_val);
        m_last_val = a_val;
        m_initialized = true;
    }

    m_measurement.at<float>(0) = a_val;
    m_KF->correct(m_measurement);
    result = m_KF->predict();
    if (std::fabs(a_val - m_last_val) > m_val_diff) {
        initFilter(a_val);
        if (m_debug)
            std::cout << "Target has changed" << std::endl;
    }
    m_last_val = a_val;

    return 1.0 * result.at<float>(0) + 0.0 * result.at<float>(1);
}