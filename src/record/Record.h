//
// Created by quanyi on 22-5-18.
//

#ifndef ARMOURSHOOT_RECORD_H
#define ARMOURSHOOT_RECORD_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>

class Record{
public:
    Record();
    ~Record();

    bool writeVideo(cv::Mat src);
    bool setParam();
    bool writeImage(std::string path, cv::Mat src);

private:
    std::string path;
    int code;
    double fps;
    cv::Size size;
    bool is_inited;
    cv::VideoWriter * videoWriter;
    int record_cnt;

};
#endif //ARMOURSHOOT_RECORD_H
