//#pragma once
//
//#include <vector>
//#include <memory>
//#include <cmath>
//#include <utility>
//#include <opencv2/opencv.hpp>
//
//#include "base.h"
//#include "common.h"
//#include "armour.h"
//
//#define TOP_CFG PROJECT_DIR"/src/topaimer/TopAimer.yaml"
//
//class Ransac {
//public:
//    Ransac();
//
//    ~Ransac() {};
//
//    std::pair<double, cv::Point2d> fitCircle(std::vector<Armour> &);
//
//private:
//    void setParam();
//
//    void initData();
//
//    // std::pair<double, cv::Point2d> getCircle(Armor &, Armor &);
//    std::pair<double, cv::Point2d> getCircle(Armour &, Armour &, Armour &);
//
//    float verifyCircle(double, cv::Point2d, std::vector<Armour> &);
//
//    bool m_debug;
//
//    // 迭代参数
//    int m_iteration_cnt;                                // 当前迭代次数
//    int m_cur_iteration;                                // 当前需迭代次数
//    int m_iteration_max;                                // 最大迭代次数
//    double m_model_confidence;                          // 置信度
//
//    // 模型参数
//    cv::RNG m_rng;                                      // 随机数发生器
//    int m_point_need;                                   // 取一个圆所需目标点数，2 或 3
//    int m_vec_size;                                     // 目标点数量
//    double m_radius_min;
//    double m_radius_max;
//    double m_inlier_dist_ratio;                         // 内点与圆的距离和半径的比值
//    double m_inlier_dist_min;                           // 内点距离阈值下限
//    double m_inlier_dist_max;                           // 内殿距离阈值上限
//
//    // 最佳结果记录
//    float m_bset_inlier_proportion;               // 最佳内点占比
//    double m_best_radius;                          // 最佳拟合半径
//    cv::Point2d m_best_center;                          // 最佳拟合圆心
//};
//
//class Aimer {
//public:
//    Aimer();
//
//    ~Aimer() {};
//
//    Armour getFinalArmor(Ptz_infor &, Armour &);
//
//    Armour getFinalArmor(Ptz_infor &, Armour &, float);
//
//    Ptz_infor getFinalPose(Ptz_infor &, Armour &);
//
//    Ptz_infor getFinalPose(Ptz_infor &, Armour &, float);
//
//    enum TrackMode {
//        FOLLOW = 0,
//        FOCUS = 1
//    };
//
//    void setTrackMode(TrackMode);
//
//    void setBulletSpeed(float);
//
//    bool shootable();
//
//    bool m_shoot_flag;
//
//private:
//    void setParam();
//
//    void setParam2();
//    void buildModel(Armour&);
//    Ptz_infor getTargetPose(Armour&);
//
//    // std::pair<double, cv::Point3f> m_pos_with_time[2];
//    Armour m_abs_armor;
//    Armour m_armor_target[2];
//    Armour m_armor_cur_target;           // 最近装甲板
//
//    Ptz_infor m_pose_target[2];
//    Ptz_infor m_pose_cur_target;       // 最近装甲板所对应位姿
//
//    int m_armor_no = 0;
//
//    double m_horizontal_diff_max;
//    double m_horizontal_diff_min;
//    double m_min_dist;
//    double m_time_diff;                 // 旋转90°用时
//    double m_angle_predict;             // 偏离正对位置的角度，角度制
//
//
////    virtual void buildModel(Armour &) = 0;
////
////    virtual Armour getTargetArmor(Armour &) = 0;
////
////    virtual Ptz_infor getTargetPose(Armour &) = 0;
//
//    // DEBUG
//    bool m_debug;
//    bool m_write_file;
//    int m_file;
//
//    cv::Point3f m_center;
//    Ptz_infor m_cur_pose;
//
//    // 控制参数
//    bool m_model_ready;
//    int m_lost_cnt;
//    int m_min_vec_size;                                 // 最小建模尺寸
//    double m_time_delay;                                // 控制延迟
//    TrackMode m_track_mode;                             // 跟随模式
//
//    // 测量量
//    float m_bullet_speed;
//    double m_palstance;                                 // 角速度，弧度制
//    double m_radius;                                    // 装甲板旋转半径
//
//    // 记录量
//    std::vector<Armour> m_armor_seq;
//    std::vector<Armour> m_abs_armor_seq;
//    std::vector<Ptz_infor> m_pose_seq;         // armor角度为相对值，须对应保留GimbalPose
//
//    std::shared_ptr<AngleSolver> m_angle_solver;
//    std::shared_ptr<Ransac> m_ransac_solver;
//
//public:
//    double getDistance(cv::Point3f p) {
//        return sqrt(p.x * p.x + p.z * p.z);
//    }
//
//    double getDistance(cv::Point2d p) {
//        return sqrt(p.x * p.x + p.y * p.y);
//    }
//
//};
