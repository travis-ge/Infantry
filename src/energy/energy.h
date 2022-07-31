//
// Created by quanyi on 2022/3/6.
//

#ifndef MAIN_CPP_ENERGY_H
#define MAIN_CPP_ENERGY_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArmourFinder.h"
#include "base.h"
#include <memory>
#include "common.h"
class Energy{
public:
    Energy();
    ~Energy();

    cv::RotatedRect energy_finder(cv::Mat src, cv::Mat img);
    void draw_energy_target(cv::Mat &src, cv::RotatedRect, cv::Point2f &center);
    bool sortPoints(cv::Point2f *points);
    bool loadCameraParams(std::string file_path);
    double getDistance(cv::Point2f point1, cv::Point2f point2);
    void draw_energy_target(cv::Mat &src, cv::Point2f *points, cv::Point2f &center);
    bool getCamP(cv::Point2f *points);
    void getPredicPoints(cv::Point2f center, cv::Point2f src_P, cv::Point2f  &dst_P,double angle);
    bool run(cv::Mat &src, int time_stamp, int find_color_energy, char mode,double & p, double &y, double &d);
    void energy_init();
    std::vector<cv::Point3f> realCorners;
    std::vector<cv::Point2f> cameraCorners;

    cv::Point3f camEnergyP;
    Send energy_last_send;
    uint8_t energy_last_flag = 0;
    std::shared_ptr<ArmourFinder> finder;
private:
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
    double Dis_last=6.4;
    std::shared_ptr<AngleSolver> angleSolver;
    double predict_time = 0.24;
    double last_p = 0;
    double last_y = 0;
    double last_d = 0;
};

#endif //MAIN_CPP_ENERGY_H
