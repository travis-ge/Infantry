//
// Created by aididiz on 22-7-8.
//
#include <opencv2/core.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include "num_classifier.h"


NumClassifier::NumClassifier(const std::string &model_path, const std::string &label_path, const double threshold) {
    net_ = cv::dnn::readNetFromONNX(model_path);
    std::ifstream label_file(label_path);
    std::string line;
    while (std::getline(label_file,line)){
        class_name_.push_back(line[0]);
    }
}
/**
 * 7 qianshaozhan  ? sentry
 * 0 jidi xiaozhuangjia
 *
 * @param src
 * @return
 */
int NumClassifier::predict(cv::Mat &src) {
    cv::cvtColor(src,src,CV_BGR2GRAY);
//    cv::imshow("sssss",src);
    cv::threshold(src,src,0,255,CV_THRESH_OTSU);
//    cv::imshow("num",src);
//    cv::waitKey(1);
    src = src / 255.0;
    cv::Mat blob;
    cv::dnn::blobFromImage(src,blob,1.,cv::Size(28,20));
    net_.setInput(blob);
    cv::Mat outputs = net_.forward();
    /*softmax*/
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;
    char num = class_name_[label_id];
    std::cout<< "label id "<<label_id<<" confidence "<<confidence<<std::endl;
    return label_id;
}