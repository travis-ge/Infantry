#ifndef __PREDICTOR_H
#define __PREDICTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "particle_filter.h"
#include "common.h"
class Predictor{
public:

    Predictor();
    ~Predictor();

    double yaw_last;
    double w_last;
    double delta_t;

    int iters;
    double yaw_predict(double yaw_camera, double yaw_ptz, double dis, double speed);
    double pitch_predict( double pitch_camera, double pitch_ptz, double dis, float v);
    cv::Point3f sentry_w_predict(cv::Point3f current_p,long long t, float time);
  
  
    int dir_cnt;
    static const int pixel_hub_sum = 10;
    double pixel_hub[pixel_hub_sum] = {0};
    double pixel_blur(double pixel_now);
    double pixel = 0;
    static const int dir_sum = 3;
    int judge_direction[dir_sum] = {0};
    int getDirection(int this_dir);
    /*****Sentry*****/

    bool sentry_mode(double &forcast_pixel,double dd);
    bool initialPredictionData(float gyro_speed, float _bullet_velocity, float yaw_angle,double dd);
    float filter_direction_ = 0;
    double sentry_dis = 0;
    double sentry_dis_last = 0;
    bool predict(double abs_pitch, double abs_yaw, double time_stamp,double &pre_yaw);
private:
    bool monitor(double abs_pitch, double abs_yaw, double time_stamp,double &yaw);
//    cv::Point2f predict(double abs_pitch, double abs_yaw, double time_stamp);
    std::vector<std::pair<double ,double>> pitch_queue;
    std::vector<std::pair<double ,double>> yaw_queue;



    const string pf_config = PROJECT_DIR"/config/filter_param.yaml";
    bool is_filter_inited = false;
    double last_loss;
    long long t_last;
    cv::Point3f last_p;
    std::vector<long long> v_t_diff;
    std::vector<float> v_x_diff;
    bool inited = false;
    double last_v = 0;
    double current_v = 0;
    /**sentry**/
    int num_cnt_ = 0;
    int judge_direction_ = 1;
    float current_direction_ = 0;
 
    int proportion_direction_ = 15;
    float last_direction_ = 0;
    float deviation_angle_ = 0;
    float initial_gyroscope_ = 0;
    float last_deviation_angle_ = 0;

    // 哨兵模型初始化计数器
    int sentry_cnt_                = 15;
    // 哨兵装甲板到枪管高度
    const int sentry_height_       = 350;
    // 哨兵到环形高度水平面垂直深度
    const int sentry_dist_         = 3380;
    // 哨兵到初始化位置实际水平深度
    int actual_z_                  = 0;
    // 哨兵到枪管实际深度
    int actual_depth_              = 0;

    float forecast_angle_ = 0;
        // 预测像素点数量
    int forecast_pixels_           = 0;
    // 上一帧预测像素点数量
    int last_forecast_pixels_      = 0;
    // 上上帧预测像素点数量
    int last_last_forecast_pixels_ = 0;

    int abrupt_variable_           = 5;
};
#endif