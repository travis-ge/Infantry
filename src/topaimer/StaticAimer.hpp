//#pragma once
//
//#include "Aimer.hpp"
//#include "armour.h"
//#include "base.h"
//
//    class StaticAimer
//    {
//    public:
//        StaticAimer();
//        ~StaticAimer() {};
//
//    private:
//        void setParam2();
//        void buildModel(Armour&);
//        Ptz_infor getTargetPose(Armour&);
//
//        // std::pair<double, cv::Point3f> m_pos_with_time[2];
//        Armour m_abs_armor;
//        Armour m_armor_target[2];
//        Armour m_armor_cur_target;           // 最近装甲板
//
//        Ptz_infor m_pose_target[2];
//        Ptz_infor m_pose_cur_target;       // 最近装甲板所对应位姿
//
//        int m_armor_no = 0;
//
//        double m_horizontal_diff_max;
//        double m_horizontal_diff_min;
//        double m_min_dist;
//        double m_time_diff;                 // 旋转90°用时
//        double m_angle_predict;             // 偏离正对位置的角度，角度制
//    };
