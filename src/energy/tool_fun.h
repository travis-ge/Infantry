//
// Created by gyb on 2021/12/6.
//

//这是小工具函数
#ifndef ARMOURSHOT_TOOL_FUN_H
#define ARMOURSHOT_TOOL_FUN_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include <sys/time.h>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
#define T_PI 3.1415926
namespace tool {
        ///常用函数
        // 旋转矩形的长宽比
        inline double aspectRatio(const cv::RotatedRect &rect) {
            return rect.size.height > rect.size.width ?
                   rect.size.height / rect.size.width :
                   rect.size.width / rect.size.height;
        }

        inline void getHiger(float& Hig, float& Wid){
            if(Hig< Wid) {std::swap(Hig,Wid);}
        }
        inline void getWider(float& Hig, float& Wid){
            if(Hig> Wid) {std::swap(Hig,Wid);}
        }

        inline double rad2deg(const float Radians) { return (Radians * 180 / T_PI); }

        inline double deg2rad(const float degrees) { return (degrees * T_PI / 180); }

        inline double getDistance (cv::Point2f point1, cv::Point2f point2){
            return sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
        }

        inline double getCircleArea(float radius){return T_PI*radius*radius;}

        //范围0~2PI
        inline float Arctan(cv::Point2f p){
            float angle = atan2(p.y,p.x);
            return fmod(CV_2PI-angle,CV_2PI);
        }

        inline float Arctan(float ptsY, float ptsX){
            float angle = atan2(ptsY,ptsX);
            return fmod(CV_2PI-angle,CV_2PI);
        }

        inline long long getTimeNow() {
            timeval tv;
            gettimeofday(&tv, NULL);
            return  1000000*tv.tv_sec + tv.tv_usec; //us
        }


    class CV_plot{
        private:

//        Mat plot = Mat::zeros(rows,cols, CV_8UC3);
//        public:
//        const int rows = 400;
//        const int cols = 200;
//
//        void clear_plot(){
//            plot = Mat::zeros(rows,cols, CV_8UC3);
//        }
//
//        void draw_point(vector<Point2f> points){
//
//        }

    };




}


#endif //ARMOURSHOT_TOOL_FUN_H
