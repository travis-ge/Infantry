//
// Created by ares on 2022/2/17.
//

#ifndef CAM_POSE_CALIB_KALMAN_H
#define CAM_POSE_CALIB_KALMAN_H

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

using namespace cv;

class MotionPredict{
public:
    MotionPredict();
    ~MotionPredict() {}
    void setParam();
    cv::Point3f predict(cv::Point3f);
    double setShootSpeed(double speed); // 设置射速等级 根据射速调整预测量

private:
    bool        m_initialized{false};
    int         m_debug;
    int         m_measure_num = 3;
    int         m_state_num   = 9;
    int         m_lost_count;
    int         m_init_count_threshold;
    float       m_control_freq;
    float       m_predict_coe;
    float       m_cur_max_shoot_speed;
    float       m_cur_armor_dis;
    float       m_last_armor_dis;
    float       m_target_change_threshold;
    cv::Mat     m_measurement;
    cv::Point3f m_last_position;
    std::shared_ptr<cv::KalmanFilter> m_KF;

    void initFilter(cv::Point3f);
};
#endif //CAM_POSE_CALIB_KALMAN_H
