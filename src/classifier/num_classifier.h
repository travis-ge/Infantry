//
// Created by aididiz on 22-7-8.
//

#ifndef ARMOURSHOOT_NUM_CLASSIFIER_H
#define ARMOURSHOOT_NUM_CLASSIFIER_H
#include <iostream>
#include <opencv2/opencv.hpp>

class NumClassifier{
public:
    NumClassifier(const std::string &model_path, const std::string &label_path, const double threshold);
    int predict(cv::Mat &src);
    double threshold;
private:
    cv::dnn::Net net_;
    std::vector<char> class_name_;
    void  gammaTransform(cv::Mat& srcImage, cv::Mat& resultImage, float kFactor);
};
#endif //ARMOURSHOOT_NUM_CLASSIFIER_H
