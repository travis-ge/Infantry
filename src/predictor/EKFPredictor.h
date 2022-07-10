//
// Created by aididiz on 22-7-5.
//

#ifndef ARMOURSHOOT_EKFPREDICTOR_H
#define ARMOURSHOOT_EKFPREDICTOR_H

#include <opencv2/opencv.hpp>
#include "AdaptiveEKF.hpp"
#include "common.h"

struct Predict {
    /*
     * 此处定义匀速直线运动模型
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
    double last_time;
    bool inited = false;
    double last_dis = 0;
    bool antitop = false;

public:
    inline void load_param(bool update_all = true) {
        /*
         * 通过文件读入，实现实时调餐
         */
        cv::FileStorage fin(PROJECT_DIR"/config/autoaim-param.yml", cv::FileStorage::READ);


        fin["Q00"] >> ekf.Q(0, 0);
        fin["Q11"] >> ekf.Q(1, 1);
        fin["Q22"] >> ekf.Q(2, 2);
        fin["Q33"] >> ekf.Q(3, 3);
        fin["Q44"] >> ekf.Q(4, 4);
        // 观测过程协方差
        fin["R00"] >> ekf.R(0, 0);
        fin["R11"] >> ekf.R(1, 1);
        fin["R22"] >> ekf.R(2, 2);
//        std::cout<<ekf.Q<<std::endl;
//        std::cout<<ekf.R<<std::endl;

    }

    explicit EKFPredictor() {
        cv::FileStorage fin(PROJECT_DIR"/asset/camera-param.yml", cv::FileStorage::READ);
        load_param();
        std::cout << "Finish create a new EKF." << std::endl;
    }
    ~EKFPredictor() = default;

    cv::Point3f predict(cv::Point3f world_point, double t);

};

#endif //ARMOURSHOOT_EKFPREDICTOR_H
