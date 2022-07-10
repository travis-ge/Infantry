//
// Created by ares on 2022/2/17.
//

#include "kalman.h"


MotionPredict::MotionPredict()
{
    setParam();
    m_KF = std::make_shared<cv::KalmanFilter>(m_state_num, m_measure_num, 0);
    m_measurement = cv::Mat::zeros(m_measure_num, 1, CV_32F);
    cv::setIdentity(m_KF->measurementMatrix);
    cv::setIdentity(m_KF->processNoiseCov, cv::Scalar::all(1e-6)); //Q-5
    cv::setIdentity(m_KF->measurementNoiseCov, cv::Scalar::all(1e-5)); //-3 R
    cv::setIdentity(m_KF->errorCovPost, cv::Scalar::all(1));
    float dt = 1.f / m_control_freq;
    m_KF->transitionMatrix = (cv::Mat_<float>(m_state_num, m_state_num) <<
            1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0, 0,
            0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt, 0,
            0, 0, 1, 0, 0, dt, 0, 0, 0.5*dt*dt,
            0, 0, 0, 1, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1);
}

/*
 * @breif 预测器参数设置
 */
void MotionPredict::setParam()
{
    m_debug = 1;
    m_init_count_threshold = 500;
    m_predict_coe = 3.1;
    m_control_freq = 18 ;//18
    m_target_change_threshold = 0.5;
    m_cur_max_shoot_speed = 15.0;
}

/*
 * @breif 滤波器初始化
 *
 * @param a_position 第一帧的装甲板绝对坐标
 */
void MotionPredict::initFilter(cv::Point3f a_position)
{
    m_KF->statePost = (cv::Mat_<float>(m_state_num, 1) <<
                                                       a_position.x, a_position.y, a_position.z, 0, 0, 0, 0, 0, 0);
    m_KF->predict();
    m_measurement.at<float>(0) = a_position.x;
    m_measurement.at<float>(1) = a_position.y;
    m_measurement.at<float>(2) = a_position.z;

    m_cur_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));
    m_last_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));

    for(int i = 0; i < m_init_count_threshold; i++)
    {
        m_KF->correct(m_measurement);
        m_KF->predict();
    }
}

/*
 * @breif 根据射速等级设置弹速 调整预测量大小
 *
 * @param a_shoot_level 当前射速等级
 */
double MotionPredict::setShootSpeed(double speed)
{
    return speed;
}

/*
 * @breif 根据新的绝对坐标测量量预测
 *
 * @param a_position 当前装甲绝对坐标
 *
 * @return 返回预测后的绝对坐标
 */
cv::Point3f MotionPredict::predict(cv::Point3f a_position)
{
    cv::Mat prediction;
    m_cur_armor_dis = std::sqrt(std::pow(a_position.x, 2) + std::pow(a_position.y, 2) + std::pow(a_position.z, 2));

    if(!m_initialized)
    {
        std::cout << "Init" << std::endl;
        initFilter(a_position);
        m_last_position = a_position;
        m_initialized = true;
    }

    m_measurement.at<float>(0) = a_position.x;
    m_measurement.at<float>(1) = a_position.y;
    m_measurement.at<float>(2) = a_position.z;
    m_KF->correct(m_measurement);
    prediction = m_KF->predict();
    if(std::sqrt(std::pow(std::fabs(a_position.x - m_last_position.x), 2) + std::pow(std::fabs(a_position.y - m_last_position.y), 2) + std::pow(std::fabs(a_position.z - m_last_position.z), 2)) > m_target_change_threshold )
    {
        initFilter(a_position);
        if(m_debug)
            std::cout << "Target has changed" << std::endl;

    }
    m_last_position = a_position;
    m_last_armor_dis = m_cur_armor_dis;

    return cv::Point3f(prediction.at<float>(0) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(3),
                       prediction.at<float>(1) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(4),
                       prediction.at<float>(2) + m_predict_coe * m_cur_armor_dis / m_cur_max_shoot_speed * prediction.at<float>(5));
}

