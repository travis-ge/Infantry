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
#include "energy.h"
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
    void imgPretreatment(const cv::Mat &src, cv::Mat &dst, int team);
    bool armourDetector(const cv::Mat &src);
    bool armourSort(void);
    Point3f getCamCoordinate(cv::Point2f &tg_pt_L, cv::Point2f &tg_pt_R, float wh_rate);
    void draw_target(RotatedRect rect, Mat &src);

    bool ifShoot(double angle_p,double angle_y);
    cv::Point2f cam2pixel(cv::Point3f camPoint);
    void armour_tracking();
    [[noreturn]]void run();
    ///

    std::vector<cv::Point3f> corners;
    std::vector<cv::Point2f> observationPts;
    cv::Point2f tg_center;
    double cx,cy,fx,fy;
    uint8_t debug;
private:
    Send last_send;
    uint8_t last_find_f = 0;
    cv::Mat camMatrix;
    cv::Mat distCoeffs;
    double time_stamp;
    Mat src,src_gray, src_separation, src_green;
    Mat roi;
    std::vector<cv::Mat> src_split_;
    vector<Tmp_armour> tmp_armour_vec;
    bool id_locked = false;
    int select_id = 0;
    int id_cnt = 0;
    double last_abs = 0;
    int track_cnt = 0;
    int losing_cnt = 0;
    int id_losing_cnt = 0;
    int tmp_losing = 0;
    int r_spilt_threshold;
    int r_gray_threshold;
    int r_green_threshold;
    int tracking_flag = 0;
    int find_cnt = 0;
    double fps_cnt = 0;
    uint8_t roi_ready = 0;
    int tracking_cnt = 0;
    cv::Rect armourROI = {0,0,0,0};

    int b_spilt_threshold;
    int b_gray_threshold;
    int b_green_threshold;
    int light_contour_min;
    int last_id = -1;
//    shared_ptr<Classifier> numClass;
    shared_ptr<NumClassifier> numClass;
    shared_ptr<AngleSolver> angleSolver;
    shared_ptr<EKFPredictor> ekf;
    shared_ptr<Energy> energy;
    const Mat element3 = getStructuringElement(MORPH_RECT,Size(3,3));
    const Mat element3_5 = getStructuringElement(MORPH_RECT,Size(3,5));
    const Mat element5_7 = getStructuringElement(MORPH_RECT,Size(5,7));
    const Mat element7_9 = getStructuringElement(MORPH_RECT,Size(7,9));
};



#endif