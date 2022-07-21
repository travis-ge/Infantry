#ifndef ARMOUR_H
#define ARMOUR_H

#include<opencv2/opencv.hpp>
#include <memory>
#include "Classifier.h"
#include <queue>
#include <chrono>
#include "base.h"
#include "num_classifier.h"
#include <EKFPredictor.h>
#include "predictor.h"
using namespace std;
using namespace cv;

struct Tmp_armour{
    cv::Point2f v_Pts_L;
    cv::Point2f v_Pts_R;
    cv::RotatedRect armour_rect;
    int id;
};
struct Armour_data{
    RotatedRect armour_rect;
    cv::Point2f armour_PL;
    cv::Point2f armour_PR;
    int id;
    cv::Point3f world_point;
};
//#define SHOW_IMG
class Armour {
public:

    Armour();
    ~Armour();

    bool readCameraParameters(string filename);
    void img_pretreatment(const cv::Mat &src, cv::Mat &dst, int team);
    bool Armor_Detector(const cv::Mat &src, double time_stamp);
    void armourSort();
    Point3f getCamCoordinate(cv::Point2f &tg_pt_L, cv::Point2f &tg_pt_R, float wh_rate);

    void draw_target(RotatedRect rect, Mat &src);

    bool ifShoot(double angle_p,double angle_y);
    cv::Point2f cam2pixel(cv::Point3f camPoint);
    void armour_imgProcess();
    void armour_tracking();
    ///
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Point3f> corners;
    std::vector<cv::Point2f> observationPts;
    cv::Point2f tg_center;
    double cx,cy,fx,fy;
    cv::Point3f m_position;               //三维坐标信息
    uint8_t debug;
    queue<pair<double,Armour_data>> armour_queue;
    Rect tracking_roi ;
    cv::Point2f roi_pl;
private:
    bool p_is_inited = false;
    Mat src,src_gray, src_separation, src_green;
    std::vector<cv::Mat> src_split_;
    vector<Tmp_armour> tmp_armour_vec;
    queue<pair<double,cv::Mat>> imgProcessed_queue;
    queue<pair<double,vector<Tmp_armour>>>  tmp_armour_queue;
    bool id_locked = false;
    int select_id = 0;
    int id_cnt = 0;
    double last_abs = 0;
    int tracking_cnt = 0;
    int losing_cnt = 0;
    int tmp_losing = 0;
    int r_spilt_threshold;
    int r_gray_threshold;
    int r_green_threshold;

    int b_spilt_threshold;
    int b_gray_threshold;
    int b_green_threshold;
    int light_contour_min;
    int last_id = -1;
    shared_ptr<Classifier> numClass;
    shared_ptr<AngleSolver> angleSolver;
    shared_ptr<EKFPredictor> ekf;
};

const Mat element3 = getStructuringElement(MORPH_RECT,Size(3,3));
const Mat element3_5 = getStructuringElement(MORPH_RECT,Size(3,5));
const Mat element5_7 = getStructuringElement(MORPH_RECT,Size(5,7));
const Mat element7_9 = getStructuringElement(MORPH_RECT,Size(7,9));

#endif