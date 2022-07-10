#include "predictor.h"
#include <math.h>
#include "base.h"

extern Ptz_infor stm;
Predictor::Predictor(){
    yaw_last = 0;
    w_last = 0;
    delta_t = 1 / 120;
    iters = 20;
    last_loss = 0;
}

Predictor::~Predictor(){
    
}

/***********************************************
 * @descrip yaw预测
 * @param
 * @return
 * @date
 *************************************************/

double Predictor::yaw_predict(double yaw_camera, double yaw_ptz, double dis, double speed){

    double a_yaw = yaw_camera + yaw_ptz;
//    if(frequency > 0)
//        delta_t = 1 / frequency;
//    else
//        delta_t = 0.000001;
    delta_t = dis / speed;
    double yaw_delta = a_yaw - yaw_last;

    float w = yaw_delta / delta_t;

    float a = (w - w_last) / delta_t;

    double yaw_pre = w * delta_t + 0.5 * a * delta_t * delta_t ;

    yaw_last = a_yaw;
    w_last = w;
   // std::clog<<"w == "<<w << " a == "<< a<<std::endl;

    return yaw_pre;
    
}

/***********************************************
 * @descrip 弹道补偿
 * @param
 * @return
 * @date
 *************************************************/
 #define PI 3.14
 #define G 9.8
 #define kf 0.001
double Predictor::pitch_predict( double pitch_camera, double pitch_ptz, double dis, float v){
    /// pitch up, angle > 0. object on the center, camera angle > 0
    //if(dis < 3){return pitch_camera;}
   // std::cout<<"infer..."<<std::endl;
    double abs_pitch = pitch_ptz - pitch_camera;
    if(abs_pitch > 90 | abs_pitch < -90){
        std::cout<<"angle error"<<std::endl;
        return -1;
    }

    double aimed_y = dis * sin(abs_pitch/180*PI);
    double aimed_x = dis * cos(abs(abs_pitch/180*PI));
    double temp_y = 10;
    double angle;
    double t, y;
    double loss;
    for (int i = 0; i < iters; i++) {
        angle = atan2(temp_y, aimed_x);
        if (angle>PI/2 | angle<-1*PI/2){break;}
        if(angle > 0){
            double t0, x0, y0;
            t0 = v * sin(angle) / G;
            x0 = v * cos(angle) * t0;
            y0 = 0.5 * G * t0 * t0;
            if(aimed_x < x0){
                t = aimed_x / (v* cos(angle));
                y = v* sin(angle)*t - 0.5*G*t*t;
              //  std::clog<<"only up"<<std::endl;
            }else{
                double t1, x1;
                x1 = aimed_x - x0;
                t1 = (exp(kf * x1) - 1) / (kf*v* cos(angle));
                t = t0 + t1;
                y = y0 - 0.5 * G * t1 * t1;
               // std::clog<<"up and down"<<std::endl;
            }
        }else{
            t = (exp(kf*aimed_x) - 1) / (kf*v* cos(angle));
            y = v* sin(angle)* - 0.5*G*t*t;
           // std::clog<<"only down"<<std::endl;
        }

        loss = aimed_y - y;
        if (abs(loss) > abs(last_loss) && i>1){ break;}
        last_loss = loss;
        //std::clog<<"iters: "+ std::to_string(i)+"/"+ std::to_string(iters)<<", loss: "<<loss<<std::endl;
        temp_y += 0.5*loss;
        if (abs(loss)<0.001){break;}

    }
    return angle/PI*180;
}
cv::Point3f Predictor::sentry_w_predict(cv::Point3f current_p, long long t, float time) {
    if (!inited){
        last_p = current_p;
        t_last = t;
        inited = true;
        last_v = 0;
    }
    std::cout<<"t   "<< t<<std::endl;
    double x_diff = abs(current_p.x - last_p.x);
    //std::cout<<"current x "<<current_p.x;
    //std::cout<<" last x "<<last_p.x<<std::endl;
    long long t_diff = t - t_last;
   // std::cout<<"t diff "<<t_diff<<std::endl;
    //std::cout<<"t diff "<<t_diff<<std::endl;
//    v_x_diff.push_back(x_diff);
//    v_t_diff.push_back(t_diff);
//    if(v_x_diff.size()<5)
//        return cv::Point3f (current_p.x + last_v*time,current_p.y,current_p.z);

//    double v_sum = 0.0;
//    for (int i = 0; i < v_x_diff.size(); i++) {
//        v_sum += v_x_diff.at(i)*1000000/v_t_diff.at(i);
//    }
    //double v = v_sum / v_x_diff.size();
    double v = x_diff*1000000/t_diff;
    double k = 0;
    current_v =(1 - k*0.01)*last_v + k*0.01*v;
    last_v = current_v;
    //v = ext_kf.KalmanFilter(&ext_kf.sentry_kf, v);
    std::clog<<"calculate v is "<<current_v<<std::endl;
    if (abs(v)< 0.5){v = 0;}
    last_p = current_p;
    t_last = t;
    last_v = v;
    v_x_diff.clear();
    v_t_diff.clear();

    cv::Point3f pre_p(current_p.x +current_v*time, current_p.y,current_p.z);
    std::cout<<"pre x "<<pre_p.x<<", pre y "<<pre_p.y<<std::endl;
    return pre_p;
}

double Predictor::pixel_blur(double pixel_now) {
    for (int i = 1; i < pixel_hub_sum+1; i++) {
        pixel_hub[i-1] = pixel_hub[i];
    }
    pixel_hub[pixel_hub_sum] = pixel_now;

    double pixel_sum = 0;
    for (int i = 1; i < pixel_hub_sum+1; i++) {
        pixel_sum += pixel_hub[i];
       // std::cout<<"ppppppp "<<pixel_hub[i]<<std::endl;
    }
    return pixel_sum / pixel_hub_sum;
}

int Predictor::getDirection(int this_dir) {
    for (int i = 1; i < dir_sum+1; i++) {
        judge_direction[i-1] = judge_direction[i];
    }
    judge_direction[dir_sum] = this_dir;
    int ss = 0;
    for (int i = 1; i < dir_sum+1; i++) {
        ss += judge_direction[i];
        //std::cout<<"aaaaaaaaaaaaaaa"<<judge_direction[i]<<std::endl;
    }

    return ss /dir_sum;
}

bool Predictor::sentry_mode(double &forcast_pixel,double dd){
    if(sentry_cnt_ == 0){
        //std::cout<<"start yaw "<<initial_gyroscope_<<std::endl;
        initialPredictionData(stm.yaw_w, stm.bulletSpeed, stm.yaw,dd);
        if(forecast_pixels_ > 300){forecast_pixels_ = 300;}
        forcast_pixel = forecast_pixels_;
    }else{
        if (sentry_cnt_ < 5) {
            initial_gyroscope_ += stm.yaw;
            initial_gyroscope_ *= 0.5;
        }
        sentry_cnt_--;
        return false;
    }
}

bool Predictor::initialPredictionData(float gyro_speed, float _bullet_velocity, float yaw_angle, double dd){
     num_cnt_++;
  // 隔帧计算
  if (num_cnt_ % 2 == 0) {
    // 判断方向
    if (gyro_speed > judge_direction_ * 0.02) {
      current_direction_ = -1;
    } else if (gyro_speed < -judge_direction_ * 0.02) {
      current_direction_ = 1;
    } else {
      current_direction_ = 0;
    }
    // 延时滤波
    filter_direction_ = (1 - proportion_direction_ * 0.01) * last_direction_ + proportion_direction_ * 0.01 * current_direction_;
    last_direction_   = filter_direction_;
    // 计算偏差角度
    deviation_angle_ = yaw_angle - initial_gyroscope_;
    if (last_deviation_angle_ != 0) {
      deviation_angle_ = (deviation_angle_ + last_deviation_angle_) * 0.5;
    }
    last_deviation_angle_ = deviation_angle_;

    // 计算水平深度
    actual_z_ = sentry_dist_ / cos(deviation_angle_);

    // 计算实际深度
    actual_depth_ = std::sqrt(actual_z_ * actual_z_ + sentry_height_ * sentry_height_);
    //std::cout<<"sssssssssssssssssssssssssss "<<dd<<std::endl;
    // 计算预测角度 角速度 * 时间
    forecast_angle_ = static_cast<float>(dd/ _bullet_velocity) * gyro_speed;

    // 计算像素点个数
    forecast_pixels_ = abs(6 * tan(forecast_angle_) * 15.5 * 100 / 4.8);

    forecast_pixels_ = (last_last_forecast_pixels_ + last_forecast_pixels_ + forecast_pixels_) * 0.333;

    if (forecast_pixels_ - last_forecast_pixels_ > abrupt_variable_) {
      forecast_pixels_ = last_forecast_pixels_ + abrupt_variable_;
    } else if (forecast_pixels_ - last_forecast_pixels_ < -abrupt_variable_) {
      forecast_pixels_ = last_forecast_pixels_ - abrupt_variable_;
    }
    last_last_forecast_pixels_ = last_forecast_pixels_;
    last_forecast_pixels_      = forecast_pixels_;
    // forecast_pixels_           = kalman_.run(forecast_pixels_);
  }
  // 计数器归零
  if (num_cnt_ % 10 == 0) {
    num_cnt_ = 0;
  }
}