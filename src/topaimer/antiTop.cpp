//
// Created by ares on 22-5-8.
//

#include "antiTop.h"
#include "memory"
Antitop::Antitop() {
    t_angle_solver = std::make_shared<AngleSolver>();
}

Antitop::~Antitop() {
    ;
}

cv::Point3f Antitop::getFinalArmour(Armour &armour, Ptz_infor cur_pose) {
    if (model_ready){
        min_dis = 10;
        v_armour_seq.clear();
        t_lost_cnt = 0;
        armour_change_cnt = 0;
        model_ready = false;
    }
    buildModel(armour, cur_pose);
    if(armour_change_cnt > 4){
        model_ready = true;
        return t_cur_target.m_position;
    }else{
        return Point3f (0,0,0);
    }
}
void Antitop::buildModel(Armour &armour, Ptz_infor cur_pose) {
    //std::cout<<"cam "<<armour.m_position<<std::endl;
    Point3f gun_ = t_angle_solver->cam2gun(armour.m_position, t_angle_solver->cam2gunDiff);
    double dis_now = sqrt(gun_.x * gun_.x + gun_.z * gun_.z);
    double deep = gun_.z;
    //std::cout<<"                                                    cur dis "<<dis_now<<std::endl;

    if(v_armour_seq.size()){
        cv::Point3f pose_diff = armour.m_position - v_armour_seq.back().m_position;
        double dis_diff = sqrt(pow(pose_diff.x,2)+pow(pose_diff.y,2));
        std::cout<<"armour dis is "<<dis_diff<<std::endl;
        if(dis_diff > max_dis_value){
            std::cout<<"armour changed!! "<<std::endl;
            armour_change_cnt++;
            std::cout<<"armour changed "<<armour_change_cnt<<" times"<<std::endl;
        }
        else if(dis_diff < min_dis_value){
            std::cout<<"armour sleeping......"<<std::endl;
            t_lost_cnt++;
            if(t_lost_cnt>10){
                v_armour_seq.clear();
                t_lost_cnt = 0;
                armour_change_cnt = 0;
                min_dis = 10;
            }
        }

    }
    v_armour_seq.emplace_back(armour);

    if(dis_now < min_dis){
        min_dis = dis_now;
        t_cur_target = armour;
        //std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa "<<t_cur_target.m_position<<std::endl;
    }
}