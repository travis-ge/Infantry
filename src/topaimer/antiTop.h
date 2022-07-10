//
// Created by ares on 22-5-8.
//

#ifndef ARMOURSHOOT_ANTITOP_H
#define ARMOURSHOOT_ANTITOP_H
#include "armour.h"
#include "base.h"
#include <opencv2/opencv.hpp>

class Antitop{
public:
    Antitop();
    ~Antitop();

    cv::Point3f getFinalArmour(Armour &armour, Ptz_infor cur_pose);

    Armour t_cur_target;

private:
    void buildModel(Armour &armour, Ptz_infor cur_pose);
    void reset();
    vector<Armour> v_armour_seq;
    vector<double> v_dis;
    bool model_ready = false;

    double max_dis_value = 0.2;
    double min_dis_value = 0.002;

    double min_dis = 10.0;

    int t_lost_cnt = 0;
    int armour_change_cnt = 0;
    std::shared_ptr<AngleSolver> t_angle_solver;

};
#endif //ARMOURSHOOT_ANTITOP_H
