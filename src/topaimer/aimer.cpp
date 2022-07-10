//#include "Aimer.hpp"
//#include "tool_fun.h"
//
//
//inline double GetDistance(cv::Point3f point) {
//    return sqrt(point.x * point.x + point.y * point.y);
//}
//
//Aimer::Aimer()
//        : m_model_ready{false},
//          m_shoot_flag{false},
//          m_track_mode{FOLLOW} {
//    m_angle_solver = std::make_shared<AngleSolver>();
//    m_ransac_solver = std::make_shared<Ransac>();
//    setParam();
//    setParam2();
//}
//
//void Aimer::setParam() {
//    cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
//    std::cout << "loading file " << std::endl;
//    fs["aimer"]["debug"] >> m_debug;
//    fs["aimer"]["write_file"] >> m_write_file;
//
//    fs["aimer"]["min_vec_size"] >> m_min_vec_size;
//    fs["aimer"]["delay"] >> m_time_delay;
//    fs["aimer"]["radius"] >> m_radius;
//    fs["aimer"]["bullet_speed"] >> m_bullet_speed;
//    //std::cout<<"aimer speed "<<m_bullet_speed<<std::endl;
//    fs.release();
//
//    std::cout << "[Top] Top param set" << std::endl;
//}
//
///**
//* @brief 主循环调用，返回目标装甲板
//*
//* @param cur_pose 当前位姿
//* @param armor 当前装甲板
//*/
//Armour Aimer::getFinalArmor(Ptz_infor &cur_pose, Armour &armor) {
//    m_cur_pose = cur_pose;
//    Armour target_armor;
//
//    //buildModel(armor);
//    if (m_model_ready) {
//      //  target_armor = getTargetArmor(armor);
//        std::cout << "<<<<<< Top model built >>>>>>";
//        if (m_shoot_flag) {
//            std::cout << "\t<<<<<< Enable shoot >>>>>>" << std::endl;
//        } else {
//            std::cout << std::endl;
//        }
//    } else {
//        target_armor = armor;
//        m_shoot_flag = false;
//    }
//
//    return target_armor;
//}
//
///**
//* @brief 主循环调用，返回目标装甲板
//*
//* @param cur_pose 当前位姿
//* @param armor 当前装甲板
//* @param bullet_speed 当前射速
//*/
//Armour Aimer::getFinalArmor(Ptz_infor &cur_pose, Armour &armor, float bullet_speed) {
//    setBulletSpeed(bullet_speed);
//    return getFinalArmor(cur_pose, armor);
//}
//
///**
//* @brief 主循环调用，返回目标位姿。受跟随模式影响
//*
//* @param cur_pose 当前位姿
//* @param armor 当前装甲板
//*/
//Ptz_infor Aimer::getFinalPose(Ptz_infor &cur_pose, Armour &armor) {
//    m_cur_pose = cur_pose;
//    Ptz_infor target_pose;
//
//    if (m_debug) {
//        if (m_track_mode == FOLLOW)
//            std::cout << "[Top] Track mode: FOLLOW" << std::endl;
//        else if (m_track_mode == FOCUS)
//            std::cout << "[Top] Track mode: FOCUS" << std::endl;
//    }
//
//    buildModel(armor);
//    if (m_model_ready) {
//        target_pose = getTargetPose(armor);
//        std::cout << "<<<<<< Top model built >>>>>>";
//        if (m_shoot_flag) {
//            std::cout << "\t<<<<<< Enable shoot >>>>>>" << std::endl;
//        } else {
//            std::cout << std::endl;
//        }
//    } else {
//        switch (m_track_mode) {
//            case FOLLOW:
//                target_pose = m_angle_solver->getAngle(armor.m_position, cur_pose);
//                break;
//            case FOCUS:
//                target_pose = m_angle_solver->getAngle(armor.m_position, cur_pose);
//                break;
//            default:
//                target_pose = cur_pose;
//                break;
//        }
//        m_shoot_flag = false;
//    }
//
//    return target_pose;
//}
//
///**
//* @brief 主循环调用，返回目标位姿。受跟随模式影响
//*
//* @param cur_pose 当前位姿
//* @param armor 当前装甲板
//* @param bullet_speed 当前射速
//*/
//Ptz_infor Aimer::getFinalPose(Ptz_infor &cur_pose, Armour &armor, float bullet_speed) {
//    setBulletSpeed(bullet_speed);
//    return getFinalPose(cur_pose, armor);
//}
//
///**
// * @brief 设置射速
// *
// * @param bullet_speed 射速
// */
//void Aimer::setBulletSpeed(float bullet_speed) {
//    m_bullet_speed = bullet_speed;
//}
//
///**
// * @brief 设置跟随模式
// *
// * @param track_mode FOLLOW: 跟随装甲板；
// *                   FOCUS: 保持云台指向敌方中心，选择最佳角度射击。
// */
//void Aimer::setTrackMode(TrackMode track_mode) {
//    m_track_mode = track_mode;
//}
//
//bool Aimer::shootable() {
//    return m_shoot_flag;
//}
//
//
//void Aimer::setParam2() {
//    std::cout << "setting param in child class" << std::endl;
//    cv::FileStorage fs(TOP_CFG, cv::FileStorage::READ);
//    fs["static"]["horizontal_diff_max"] >> m_horizontal_diff_max;
//    fs["static"]["horizontal_diff_min"] >> m_horizontal_diff_min;
//    fs["static"]["angle_predict"] >> m_angle_predict;
//    fs.release();
//}
//
//void Aimer::buildModel(Armour &armor) {
//    // 转绝对坐标
//    m_abs_armor.m_position = m_angle_solver->cam2abs(armor.m_position, m_cur_pose); //转成绝对坐标
//    std::cout<<"anti abs "<<m_abs_armor.m_position<<std::endl;
//    if (m_armor_seq.size()) {     //存了一定装甲板
//        cv::Point3f pos_diff = m_abs_armor.m_position - m_armor_seq.back().m_position;
//        double horizontal_diff = getDistance(pos_diff);
//        std::cout << "[Armor] horizontal_diff " << horizontal_diff << std::endl;
//        // 装甲板切换，即一个周期结束
//        if (horizontal_diff > m_horizontal_diff_max) {
//            std::cout << "[Armor] changed to another armor !" << std::endl;
//            m_min_dist = INFINITY;
//            std::cout << "[Armor] vecSize " << m_armor_seq.size() << std::endl;
//            if (m_armor_seq.size() >= m_min_vec_size) {
//                m_armor_cur_target.m_time_seq = tool::getTimeNow();
//                m_armor_target[m_armor_no] = m_armor_cur_target;
//                m_pose_target[m_armor_no] = m_pose_cur_target;
//                // 两个周期后认为建模完成
//                if (!m_model_ready && m_armor_no == 1) {
//                    m_model_ready = true;
//                    std::cout << "[Aimer] model ready!" << std::endl;
//                }
//                if (m_model_ready) {
//                    // 计算角速度大小
//                    m_time_diff =
//                            m_armor_target[m_armor_no].m_time_seq - m_armor_target[(m_armor_no + 1) % 2].m_time_seq;
//                    m_palstance = (CV_PI / 2) *1000000/ m_time_diff;
//                    // 判断角速度方向
//                    double x_start = m_angle_solver->abs2cam(m_armor_seq.front().m_position, m_pose_cur_target).x;
//                    double x_end = m_angle_solver->abs2cam(m_armor_seq.back().m_position, m_pose_cur_target).x;
//                    m_palstance *= (x_start - x_end) / abs(x_start - x_end);//求取方向
//                    std::cout << "[Aimer] palstance = " << m_palstance << std::endl;
//                }
//                m_armor_no = (m_armor_no + 1) % 2;
//            } else {
//                m_model_ready = false;
//                m_armor_no = 0;
//                m_shoot_flag = 0;
//            }
//            m_armor_seq.clear();
//        }
//            // 认为装甲板静止
//        else if (horizontal_diff < m_horizontal_diff_min) {
//            std::cout << "[Armor] not spining " << horizontal_diff << std::endl;
//            m_lost_cnt++;
//            if (m_lost_cnt > 10) {
//                m_armor_seq.clear();
//                m_model_ready = false;
//                m_lost_cnt = 0;
//            }
//        }
//    }
//    m_armor_seq.emplace_back(m_abs_armor);
//
//    // 找最近装甲板，即正面位置
//    double dist = getDistance(m_abs_armor.m_position);
//    if (dist < m_min_dist) {
//        m_min_dist = dist;
//        m_armor_cur_target = m_abs_armor;
//        m_pose_cur_target = m_angle_solver->getAngle(armor.m_position, m_cur_pose, m_bullet_speed);
//    }
//}
//
//Ptz_infor Aimer::getTargetPose(Armour &) {
//    int no = m_armor_no;
//    Ptz_infor final_pose;
//    long long  time_now = tool::getTimeNow(); //us
//    // 预计击中时刻
//    long long time_next = time_now + 1000000*(m_time_delay + m_armor_target[m_armor_no].m_position.z / m_bullet_speed);//us
//    // 提前量时长
//    long long time_len = (m_angle_predict * CV_PI / 180) / m_palstance * 1000000; //us
//
//    Armour armor_target = m_armor_target[m_armor_no];
//    Ptz_infor pose_target = m_pose_target[m_armor_no];
//
//    // 下一装甲板正对时刻
//    long long time_target = armor_target.m_time_seq + m_time_diff;  //us
//    // cv::Point3f pos_next = armor_target.m_position;
//
//    while (time_next > time_target + time_len) {
//        no = (no + 1) % 2;
//        armor_target = m_armor_target[no];
//        //pose_target = m_pose_target[(m_armor_no + 1) % 2];
//        time_target += m_time_diff;
//    }
//
//    // if(time_next <= time_target - time_len)
//    // {
//    //     std::cout << "[Armor] waiting" << std::endl;
//    //     //sleep(time_target - time_len - time_next);
//    //     armor_target.m_position.y -= sin(time_len * m_palstance) * m_radius;
//    //     m_shoot_flag = 0;
//    // }
//
//    // 不精准计算
//    // armor_target.m_position.x += ?
//    std::cout<<"predict before "<<armor_target.m_position<<std::endl;
//    armor_target.m_position.x += sin((time_next - time_target)/1000000 * m_palstance) * m_radius;
//    //std::cout<<"add xxxxx "<<sin((time_next - time_target)/1000000 * m_palstance) * m_radius<<std::endl;
//    // 判断发射窗口
//    std::cout<<"predict after  "<<armor_target.m_position<<std::endl;
//    if (abs(time_next - time_target) < time_len) {
//        std::cout << "[Armor] shooting" << std::endl;
//        m_shoot_flag = 1;
//    }
//
//    std::cout << "[Armor] predict cord after" << armor_target.m_position << std::endl;
//    armor_target.m_position = m_angle_solver->abs2cam(armor_target.m_position, m_cur_pose);
//    final_pose = m_angle_solver->getAngle(armor_target.m_position, m_cur_pose, m_bullet_speed);
//
//    return final_pose;
//}
//
