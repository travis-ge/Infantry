//
// Created by quanyi on 2021/10/24.
//

#include "Classifier.h"
#include <stdio.h>
#include "../common.h"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <iostream>

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
static char *line = NULL;
static int max_line_len;

std::ostringstream oss;
//using namespace cv;
svm_model *my_model = svm_load_model(PROJECT_DIR"/config/model6.txt");
Classifier::Classifier() {
    sample_path_ = PROJECT_DIR"/data/sample/";
    dataset_path_ = PROJECT_DIR"/config/data4.txt";
    model_path_ = PROJECT_DIR"/config/model6.txt";
//    lodeSample();
//    trainModel();
}

Classifier::~Classifier() {

}
/**
 *
 * @param input
 * @return
 */
static char* readline(FILE *input)
{
    int len;
    if(fgets(line,max_line_len,input) == NULL)
        return NULL;
    while(strrchr(line,'\n') == NULL)
    {
        max_line_len *= 2;
        line = (char *) realloc(line,max_line_len);
        len = (int) strlen(line);
        if(fgets(line+len,max_line_len-len,input) == NULL)
            break;
    }
    return line;
}
/**
 *
 */
void Classifier::lodeSample() {
    std::ofstream data_set(dataset_path_,std::ios::trunc);
    for(int p = 0; p < kClassNum; p++){
        oss << sample_path_;
        num += 1;
        int label = num;
        oss << num << "/*.jpg";
        std::string pattern = oss.str();

        oss.str("");
        std::vector<cv::Mat> input_images_;
        std::vector<cv::String> input_images_name_;
        cv::Mat tmp_img_;
        cv::glob(pattern,input_images_name_, false);

        int img_sum = input_images_name_.size();

        for(int i = 0; i < img_sum; i++){
            data_set << p+1 << " \t";
            cv::Mat roi_img_ = cv::imread(input_images_name_[i]);
            cv::cvtColor(roi_img_, sample_gray_, CV_BGR2GRAY);
            cv::threshold(sample_gray_,sample_gray_,0,255,CV_THRESH_OTSU);
            cv::Mat trainingImg_ = cv::Mat(cv::Size(128,128), CV_8UC1);
            cv::resize(sample_gray_, trainingImg_, trainingImg_.size());
            //处理HOG特征
//            cv::imshow("sample", trainingImg_);
//            cv::waitKey(0);
            cv::HOGDescriptor *hog = new cv::HOGDescriptor(cv::Size(28, 28), cv::Size(14, 14), cv::Size(7, 7), cv::Size(7, 7), 9);

            std::vector<float> descriptors;//存放结果    为HOG描述子向量
            hog->compute(trainingImg_, descriptors, cv::Size(7, 7)); //Hog特征计算，检测窗口移动步长(1,1)
            std::cout << "HOG描述子向量维数    : " << descriptors.size() << std::endl;
            for (int j = 0; j < descriptors.size(); j++) {
                data_set << j+1;
                data_set << ':';
                data_set << descriptors[j];
                data_set << " \t";
            }
            data_set << " \t\n";
                // std::cout << descriptors[i] << std::endl;
            }
    }
    std::cout<< "dataset over!"<<std::endl;
}
/**
 *
 */
void Classifier::trainModel() {
    const char * filename = dataset_path_;
    ///init params
    param.svm_type = C_SVC;
    param.kernel_type = RBF;
    param.degree = 3;
    param.gamma = 0;	// 1/num_features
    param.coef0 = 0;
    param.nu = 0.5;
    param.cache_size = 100;
    param.C = 1;
    param.eps = 1e-3;
    param.p = 0.1;
    param.shrinking = 1;
    param.probability = 1;
    param.nr_weight = 0;
    param.weight_label = NULL;
    param.weight = NULL;

    int max_index, inst_max_index, i;
    size_t elements, j;
    FILE *fp = fopen(filename,"r");
    char *endptr;
    char *idx, *val, *label;

    if(fp == NULL)
    {
        fprintf(stderr,"can't open input file %s\n",filename);
        exit(1);
    }

    prob.l = 0;
    elements = 0;

    max_line_len = 1024;
    line = Malloc(char,max_line_len);
    while(readline(fp)!=NULL)
    {
        char *p = strtok(line," \t"); // label ,返回被分解的第一个子字符串
        // features
        while(1)
        {
            p = strtok(NULL," \t");  //此时NULL指针指向前一个被分割处
            if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
                break;
            ++elements;  //特征数
        }
        ++elements; //最后一个‘\n’
        ++prob.l; //样本数
    }
    rewind(fp); //开头

    prob.y = Malloc(double,prob.l);
    prob.x = Malloc(struct svm_node *,prob.l);
    x_space = Malloc(struct svm_node,elements);

    max_index = 0;
    j=0;
    for(i=0;i<prob.l;i++)
    {
        inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
        readline(fp);
        prob.x[i] = &x_space[j];
        label = strtok(line," \t\n");

        if(label == NULL) // empty line
            exit(1);
            //exit_input_error(i+1);

        prob.y[i] = strtod(label,&endptr);
        if(endptr == label || *endptr != '\0')
            exit(1);
            //exit_input_error(i+1);

        while(1)
        {
            idx = strtok(NULL,":");
            val = strtok(NULL," \t");
            //std::cout<<val<<std::endl;
            if(val == NULL)
                break;

            errno = 0;
            x_space[j].index = (int) strtol(idx,&endptr,10);
            if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
                //exit_input_error(i+1);
                exit(1);
            else
                inst_max_index = x_space[j].index;

            errno = 0;
            x_space[j].value = strtod(val,&endptr);
            if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
                //exit_input_error(i+1);
                exit(1);

            ++j;
        }

        if(inst_max_index > max_index)
            max_index = inst_max_index;
        x_space[j++].index = -1;
    }

    if(param.gamma == 0 && max_index > 0)
        param.gamma = 1.0/max_index;

    if(param.kernel_type == PRECOMPUTED){
        for(i=0;i<prob.l;i++)
        {
            if (prob.x[i][0].index != 0)
            {
                fprintf(stderr,"Wrong input format: first column must be 0:sample_serial_number\n");
                exit(1);
            }
            if ((int)prob.x[i][0].value <= 0 || (int)prob.x[i][0].value > max_index)
            {
                fprintf(stderr,"Wrong input format: sample_serial_number out of range\n");
                exit(1);
            }
        }

    }
    fclose(fp);
    model = svm_train(&prob,&param);
    svm_save_model(model_path_,model);

    std::cout << "train over" << std::endl;
}

void Classifier::gammaTransform(cv::Mat &srcImage, cv::Mat &resultImage, float kFactor) {
    unsigned char LUT[256];
    for (int i = 0; i < 256; i++)
    {
        float f = (i + 0.5f) / 255;
        f = (float)(pow(f, kFactor));
        LUT[i] = cv::saturate_cast<uchar>(f * 255.0f - 0.5f);
    }
    if (srcImage.channels() == 1)
    {
        cv::MatIterator_<uchar> iterator = resultImage.begin<uchar>();
        cv::MatIterator_<uchar> iteratorEnd = resultImage.end<uchar>();
        for (; iterator != iteratorEnd; iterator++)
        {
            *iterator = LUT[(*iterator)];
        }
    }
    else
    {
        cv::MatIterator_<cv::Vec3b> iterator = resultImage.begin<cv::Vec3b>();
        cv::MatIterator_<cv::Vec3b> iteratorEnd = resultImage.end<cv::Vec3b>();
        for (; iterator != iteratorEnd; iterator++)
        {
            (*iterator)[0] = LUT[((*iterator)[0])];//b
            (*iterator)[1] = LUT[((*iterator)[1])];//g
            (*iterator)[2] = LUT[((*iterator)[2])];//r
        }
    }
}
/**
 *
 * @param src
 * @return
 */
int Classifier::numPredict(cv::Mat &src) {

    cv::cvtColor(src,src,CV_BGR2GRAY);

    cv::Mat element3  = cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::erode(src,src,element3);
    cv::dilate(src,src,element3);
    cv::erode(src,src,element3);
    cv::dilate(src,src,element3);
    gammaTransform(src,src,0.3);
    threshold(src, src, 0 ,255,cv::THRESH_OTSU);
    cv::resize(src, src, cv::Size(28,28));
//    cv::imshow("sample", src);
    //处理HOG特征

    hog->compute(src, descriptors, cv::Size(7, 7)); //Hog特征计算，检测窗口移动步长(1,1)

    svm_node *features = new svm_node[FEATURE_NUM + 1];
    for (int i = 0; i < descriptors.size(); i++) {
        features[i].index = i + 1;
        features[i].value = descriptors[i];
    }
    features[FEATURE_NUM].index = -1;

    int predictValue = 0;                      //最终识别数值
    double a[kClassNum];
    double Probability = svm_predict_probability(my_model, features,a);
    //for (int i = 0; i < kClassNum; ++i) {
    //    std::cout<<a[i]<<std::endl;
   // }

    int i = (int)Probability - 1;
    if(a[i]>0.5){
        predictValue = Probability;
        std::cout<<"识别数字："<<Probability<<"      概率:"<<a[i]<<std::endl;
    }else{
        predictValue = -1;
    }

    return predictValue;
}

