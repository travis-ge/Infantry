//
// Created by ares on 2022/4/2.
//

#ifndef MAIN_CPP_BASE_H
#define MAIN_CPP_BASE_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
using namespace Eigen;

struct Ptz_infor {
    double pitch;
    double yaw;
    double bulletSpeed;
    double pitch_w;
    double yaw_w;
};


///角度解算
class AngleSolver {
public:

    AngleSolver();
    ~AngleSolver();


    cv::Point3f cam2gun(cv::Point3f cam_, cv::Point3f diff);
    cv::Point3f gun2cam(cv::Point3f gun_, cv::Point3f diff);
    cv::Point3f gun2ptz(cv::Point3f gun_, cv::Point3f diff);
    cv::Point3f cam2abs(cv::Point3f camPoint, Ptz_infor stm);
    cv::Point3f abs2cam(cv::Point3f absPoint, Ptz_infor stm);
    void getAngle(cv::Point3f cam_, double &pitch, double &yaw, double &Dis);
    void getAngle_nofix(cv::Point3f cam_, double &pitch, double &yaw, double &Dis);
    double abs_pitch = 0;
    double abs_yaw = 0;

    cv::Point3f cam2gunDiff = cv::Point3f(0, -0.0395, 0.0805);
    cv::Point3f gun2ptzDiff = cv::Point3f(0, -0.006, 0.061);

private:

    const float pitch_diff[8] = {0, 4.0, 3.3, 0.3, 0, 1.0, 0.0, 0.0};
    const float yaw_diff[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    float simple_pitch_diff;
    float simple_yaw_diff;
    float sentry_pitch_diff;
    float sentry_yaw_diff;
    float buff_pitch_diff;
    float buff_yaw_diff;
    bool getRotX(double pitch);
    bool getRotY(double yaw);
    bool getRotZ(double roll);

    Matrix<double, 4, 4> trans;
    Matrix<double,4, 4> RotX;
    Matrix<double, 4,4> RotY;
    Matrix<double , 4, 4>RotZ;
    Matrix<double , 4, 4>Rot;
    Matrix<double, 4,1> pointMat;
    Matrix<double, 4,1> absPointMat;

};

///卡尔曼滤波
/**
     * @brief 陀螺自瞄专用一维滤波器
     */
class Kalman {
public:
    Kalman(double, double, double);

    ~Kalman() {}

    void setParam();

    double predict(double);

private:
    bool m_initialized{false};
    int m_debug;
    int m_measure_num = 1;
    int m_state_num = 2;
    int m_lost_count;
    int m_init_count_threshold;
    float m_control_freq;
    float m_predict_coe;
    // double  m_process_noise;
    // double  m_measure_noise;
    double m_last_val;
    double m_val_diff;
    cv::Mat m_measurement;
    std::shared_ptr<cv::KalmanFilter> m_KF;

    void initFilter(double);
};


#endif MAIN_CPP_BASE_H