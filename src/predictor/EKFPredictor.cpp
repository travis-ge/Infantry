//
// Created by quanyi on 22-7-5.
//
#include "EKFPredictor.h"
#include "serial_port.h"
#include "base.h"

extern Ptz_infor stm;
extern SerialPort port;

cv::Point3f EKFPredictor::predict(Armour_case &armour, double t) {
    anti_judge(armour,t);
    std::cout<<"< ==================== ANTI STATUS: "<< is_anti<<std::endl;
    if(port.receive[1] == 'c')
        is_anti = true;
    else
        is_anti = false;
    double shoot_delay = 0.1;
    double dis = sqrt(pow(armour.world_point.x,2)+ pow(armour.world_point.y,2)+ pow(armour.world_point.z,2));

    if(fabs(dis - last_dis) > 0.15 || (t - last_time) > 1 || armour.id != last_armour.id){
        is_inited = false;
        std::cout<<"target changed ,ekf re init !!!!! "<<"dis diff "<<fabs(dis-last_dis)<<
        " t diff "<<t - last_time<<std::endl;
    }
    std::cout<<"id "<<armour.id << " last id "<<last_armour.id<<std::endl;
    std::cout<<"dis error "<<fabs(dis-last_dis)<<std::endl;
    if(!is_inited){
        last_time = t;
        last_dis = dis;
        last_armour = armour;
        is_inited = true;
        Eigen::Matrix<double, 5, 1> Xr;
        Xr << armour.world_point.x, 0, armour.world_point.y, 0, armour.world_point.z;
        ekf.init(Xr);
        return armour.world_point;
    }
    last_dis = dis;
    double delta_t = t - last_time;
    last_time = t;
    last_armour = armour;
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

//    double pre_dis = sqrt(pow(p_pw(0,0),2) + pow(p_pw(1,0),2) + pow(p_pw(2,0),2));
    ///check if predict acceptable
//    if(fabs(pre_dis - dis) > 0.5){
//        std::cout<<" pre error :"<< fabs(pre_dis-dis)<<std::endl;
//        is_inited = false;
//        return armour.world_point;
//    }
    return cv::Point3f (p_pw(0,0),p_pw(1,0),p_pw(2,0));
}

void EKFPredictor::anti_judge(Armour_case &armour, double t) {
    if(t - last_time > 0.5){
        is_anti = false;
        anti_judge_cnt = 0;
    }
    double current_yaw = atan2(armour.world_point.x, armour.world_point.z);
    double last_yaw = atan2(last_armour.world_point.x, last_armour.world_point.z);
    double yaw_diff = shortest_angle_distance(last_yaw, current_yaw);
    std::cout<<"yaw diff "<<yaw_diff<<std::endl;
    if(fabs(yaw_diff) > yaw_diff_max){
        anti_judge_cnt ++ ;
        if(anti_judge_cnt>1 && (std::signbit(yaw_diff) == std::signbit(last_yaw_diff))){
            is_anti = true;
            anti_lose_cnt = 0;
        }
        last_yaw_diff = yaw_diff;
    }else{
        if(std::signbit(yaw_diff) != std::signbit(last_yaw_diff)){
            anti_lose_cnt++;
        }
        if(anti_lose_cnt > 10){
            is_anti = false;
            anti_judge_cnt = 0;
        }
    }
}