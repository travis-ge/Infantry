//
// Created by quanyi on 22-7-5.
//
#include "EKFPredictor.h"
#include "base.h"
extern Ptz_infor stm;
cv::Point3f EKFPredictor::predict(cv::Point3f world_point, double t) {
    double shoot_delay = 0.1;
    double dis = sqrt(pow(world_point.x,2)+ pow(world_point.y,2)+ pow(world_point.z,2));


    if((fabs(dis - last_dis) > 0.15 || (t - last_time) > 1) && inited){
        inited = false;
        std::cout<<"ekf reinit "<<std::endl;
    }
    std::cout<<"dis error "<<fabs(dis-last_dis)<<std::endl;
    if(!inited){
        last_time = t;
        last_dis = dis;
        inited = true;
        Eigen::Matrix<double, 5, 1> Xr;
        Xr << world_point.x, 0, world_point.y, 0, world_point.z;
        ekf.init(Xr);
        return world_point;
    }

    last_dis = dis;
    double delta_t = t - last_time;
    last_time = t;
    Predict predictfunc;
    Measure measure;

    Eigen::Matrix<double, 5, 1> Xr;
    Xr << world_point.x, 0, world_point.y, 0, world_point.z;
    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());

    predictfunc.delta_t = delta_t;
    ekf.predict(predictfunc);
    Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);
//    std::cout<<"  ppppppppppppppppppppppppp"<<Xe<<std::endl;
    double predict_time = sqrt(pow(world_point.x,2)+ pow(world_point.y,2)+ pow(world_point.z,2))/stm.bulletSpeed + shoot_delay;
    predictfunc.delta_t = predict_time;
    Eigen::Matrix<double, 5, 1> Xp;
    predictfunc(Xe.data(), Xp.data());

    Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
    Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)}; // predict

    return cv::Point3f (p_pw(0,0),p_pw(1,0),p_pw(2,0));
}