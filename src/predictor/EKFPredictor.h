//
// Created by aididiz on 22-7-5.
//

#ifndef ARMOURSHOOT_EKFPREDICTOR_H
#define ARMOURSHOOT_EKFPREDICTOR_H

#include <opencv2/opencv.hpp>
#include "AdaptiveEKF.hpp"
#include "common.h"
struct Armour_case{
    cv::Point3f world_point;
    int id;
};
struct Predict {
    /*
     * 此处定义匀速直线运动模型  x1 = x0+v*delta_t
     * x1 = [x,v,]
     */
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }
    double delta_t;
};

//  xyz
// pitch  y / (x2 + z2)
//yaw  x/z
template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[1], ceres::sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));  // pitch
    pyd[1] = ceres::atan2(xyz[0], xyz[2]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};

class EKFPredictor {
private:
    AdaptiveEKF<5, 3> ekf;  // 创建ekf
    AdaptiveEKF<5, 3> anti_ekf;
    std::deque<Armour_case> armour_seq;
    Armour_case last_armour;
    double last_time;
    double last_dis = 0;
    /// anti judge
    void anti_judge(Armour_case & armour, double t);
    bool is_anti = false;
    float yaw_diff_max = 0.2;
    int anti_judge_cnt = 0;
    int anti_lose_cnt = 0;
    double last_yaw_diff = 0;

    static inline double normalize_angle(double angle){
        const double result = fmod(angle + M_PI, 2.0 * M_PI);
        if(result <= 0.0)
            return result  + M_PI;
        return result - M_PI;
    }

    static inline double shortest_angle_distance(double from, double to){
        return normalize_angle(to - from);
    }

public:
    bool is_inited = false;
    inline void load_param(bool update_all = true) {
        cv::FileStorage fin(Param, cv::FileStorage::READ);
        fin["ekf"]["Q00"] >> ekf.Q(0, 0);
        fin["ekf"]["Q11"] >> ekf.Q(1, 1);
        fin["ekf"]["Q22"] >> ekf.Q(2, 2);
        fin["ekf"]["Q33"] >> ekf.Q(3, 3);
        fin["ekf"]["Q44"] >> ekf.Q(4, 4);
        // 观测过程协方差
        fin["ekf"]["R00"] >> ekf.R(0, 0);
        fin["ekf"]["R11"] >> ekf.R(1, 1);
        fin["ekf"]["R22"] >> ekf.R(2, 2);
        ///
        fin["anti_ekf"]["Q00"] >> anti_ekf.Q(0, 0);
        fin["anti_ekf"]["Q11"] >> anti_ekf.Q(1, 1);
        fin["anti_ekf"]["Q22"] >> anti_ekf.Q(2, 2);
        fin["anti_ekf"]["Q33"] >> anti_ekf.Q(3, 3);
        fin["anti_ekf"]["Q44"] >> anti_ekf.Q(4, 4);
        // 观测过程协方差
        fin["anti_ekf"]["R00"] >> anti_ekf.R(0, 0);
        fin["anti_ekf"]["R11"] >> anti_ekf.R(1, 1);
        fin["anti_ekf"]["R22"] >> anti_ekf.R(2, 2);


    }

    explicit EKFPredictor() {
        load_param();
        std::cout << "Finish create a new EKF." << std::endl;
    }
    ~EKFPredictor() = default;

    cv::Point3f predict(Armour_case  &armour, double t);

};

#endif //ARMOURSHOOT_EKFPREDICTOR_H
