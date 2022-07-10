//
// Created by quanyi on 2021/10/24.
//

#ifndef ARMORSHOT_CLASSIFIER_H
#define ARMORSHOT_CLASSIFIER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include "svm.h"

#define FEATURE_NUM 2304
class Classifier{
public:

    Classifier();
    ~Classifier();

    void lodeSample();
    void trainModel();
    int numPredict(cv::Mat &src);

    const int kClassNum = 8;
    const int kImgNum = 150; //
    const int kImageCols = 130; //width
    const int kImageRows = 55;  //height

    const char * sample_path_;
    const char * dataset_path_;
    const char * model_path_;

    svm_problem prob;
    svm_parameter param;
    svm_model *model ;
    svm_node *x_space;
    //
    cv::HOGDescriptor *hog = new cv::HOGDescriptor(cv::Size(128, 128), cv::Size(8, 8), cv::Size(8, 8), cv::Size(8, 8), 9);
    std::vector<float> descriptors;//存放结果    为HOG描述子向量

    int num = 0;

private:
    cv::Mat sample_gray_;
    cv::Mat sample_threshold_;
    std::string save_path_;

};

#endif //ARMOURSHOT_ARMOURCLASSIER_H
