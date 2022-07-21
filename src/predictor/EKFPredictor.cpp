//
// Created by quanyi on 22-7-5.
//
#include "EKFPredictor.h"
#include "base.h"

extern Ptz_infor stm;
cv::Point3f EKFPredictor::predict(Armour_case &armour, double t) {
    if(armour_seq.size() > 10)
        armour_seq.pop_front();
    armour_seq.push_back(armour);
    double shoot_delay = 0.1;
    double dis = sqrt(pow(armour.world_point.x,2)+ pow(armour.world_point.y,2)+ pow(armour.world_point.z,2));

    if(fabs(dis - last_dis) > 0.15 || (t - last_time) > 1 || armour.id != (armour_seq.end()-1)->id){
        is_inited = false;
        std::cout<<"target changed ,ekf re init !!!!! "<<"dis diff "<<fabs(dis-last_dis)<<
        " t diff "<<t - last_time<<std::endl;
    }
    std::cout<<"id "<<armour.id << " last id "<<(armour_seq.end()-1)->id<<std::endl;
    std::cout<<"dis error "<<fabs(dis-last_dis)<<std::endl;
    if(!is_inited){
        last_time = t;
        last_dis = dis;
        is_inited = true;
        Eigen::Matrix<double, 5, 1> Xr;
        Xr << armour.world_point.x, 0, armour.world_point.y, 0, armour.world_point.z;
        ekf.init(Xr);
        return armour.world_point;
    }
    last_dis = dis;
    double delta_t = t - last_time;
    last_time = t;
    Predict predictfunc;
    Measure measure;

    Eigen::Matrix<double, 5, 1> Xr;
    Xr << armour.world_point.x, 0, armour.world_point.y, 0, armour.world_point.z;
    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());

    predictfunc.delta_t = delta_t;
    ekf.predict(predictfunc);
    Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);
//    std::cout<<"  ppppppppppppppppppppppppp"<<Xe<<std::endl;
    double predict_time = sqrt(pow(armour.world_point.x,2)+ pow(armour.world_point.y,2)+ pow(armour.world_point.z,2))/stm.bulletSpeed + shoot_delay;
    predictfunc.delta_t = predict_time;
    Eigen::Matrix<double, 5, 1> Xp;
    predictfunc(Xe.data(), Xp.data());

    Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
    Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)}; // predict
    std::cout<<" x v_speed "<<Xe(1,0) << " y v_speed "<<Xe(3,0)<<std::endl;

    double pre_dis = sqrt(pow(p_pw(0,0),2) + pow(p_pw(1,0),2) + pow(p_pw(2,0),2));
    ///check if predict acceptable
    if(fabs(pre_dis - dis) > 0.5){
        std::cout<<" pre error :"<< fabs(pre_dis-dis)<<std::endl;
        is_inited = false;
        return armour.world_point;
    }
    return cv::Point3f (p_pw(0,0),p_pw(1,0),p_pw(2,0));
}