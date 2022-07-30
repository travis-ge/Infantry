//
// Created by gyb on 2021/12/6.
//

#ifndef ARMOURSHOT_ENERGY_H
#define ARMOURSHOT_ENERGY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tool_fun.h"
#include "data_tpye.h"
#include "particle_filter.h"
//智能车写法，全局变量一堆
using namespace cv;
using namespace std;

// typedef Point_<long long> Point2l;


 struct fan_data{
  Point2f center;
 };

struct Fan_data{
    cv::Point2f pre_point;
    cv::Point2f tag_point;
    cv::Point2f armour_points[4];
    cv::RotatedRect armour_rect;
    cv::Point2f circle_center;
    double pre_angle;
};

class ArmourFinder
{
public:
    ArmourFinder();
    ~ArmourFinder();
    int searchArmour(Mat& src, long long timestamp, double p_time, uint8_t find_color_energy, uint8_t mode);

    Point2f   aim_box[4];
    Point2f   pre_point;              //当前帧中预测点
    Point2f   tag_point;              //当前帧中待击打装甲板中心点
    RotatedRect min_tagbox;           //目标装甲板
    bool is_inited;
    cv::Point circle_center_point;    //风车圆心坐标
    Rect center_ROI;                //风车中心候选区
    Fan_data fan_;

private:

    uint8_t debug;
    uint8_t debug_no_img;
    int r_gray_threshold;
    int r_spilt_threshold;
    int r_green_threshold;
    int b_gray_threshold;
    int b_spilt_threshold;
    int b_green_threshold;

    int fan_size_threshold;

    //****变量区******//
    //SolvePara solveFan;
    uint cnt=0;

    Mat dst;
    Mat tagROI;
    std::vector<cv::Mat> src2bgr;
    Mat src_gray, src_separation, src_green, src_bin;

    ///一个任务周期清零一次///
    bool  is_first_find;
    bool  is_find_dir;                  //本任务周期是否确定旋转方向
    int   rotation_direction=0;         //当前方向 顺时针 1 逆时针 -1
    int   last_rotation_direction=0;    //上一次方向
    double   predict_time=0;               //预测时间

    ParticleFilter pf;
    ParticleFilter pf_param_loader;

    /////////

    ///每帧图像变量都清零一次///
    vector<vector<Point>> fan_contours;    //外轮廓矩阵
    vector<Vec4i> fan_hierarchy;            //存储形式结构
    vector<Rect>  tag_outline_con;          //待击打扇叶外接矩阵

    vector<vector<Point> > armor_contours;
    vector<Vec4i> armor_hierarchy;
    vector<vector<Point> > armor_contours_external;//用总轮廓减去外轮廓，只保留内轮廓，除去流动条的影响。
    /////////
    

    fan_data tag_fan;
    cv::RotatedRect centerR;        //风车中心字母R的可能候选区
    long long  time_stamp;          //类内全局变量，防止函数调用多次传参

    vector<time_angle> v_angle;
    vector<time_angle> fit_speed;        //观测到的转速数组
    vector<time_angle> fit_acc;          //加速度计算

    int last_idx_acc =0;
    int last_idx_spd =0;


    // Point2f tag_point;              //当前帧中待击打装甲板中心点


    Point2f last_tag_point;
    float   target_polar_angle=0;   //待击打装甲板的极坐标角度
    float   last_polar_angle=0;     //装甲板的极坐标角度
    float   changed_angle=0;        //变换角度
    int     idxOfchanged = 0;       //跳转角度索引
//    bool    is_target_changed;      //目标切换的标志
    int     target_changed_delay;      //目标切换的标志
    bool    is_sine_solved;         //正弦参数拟合完成标志位
    double  pre_angle=0;            //预测角度

    int stride     = 1;       //记录步长
    //****常量*****//
    const Mat element3   = getStructuringElement(MORPH_RECT,Size(3,3));
    const Mat element3_5 = getStructuringElement(MORPH_RECT,Size(3,5));
    const Mat element5   = getStructuringElement(MORPH_RECT,Size(5,5));
    const Mat element5_7 = getStructuringElement(MORPH_RECT,Size(5,7));
    const Mat element7 = getStructuringElement(MORPH_RECT,Size(7,7));
    const Mat element7_9 = getStructuringElement(MORPH_RECT,Size(7,9));

    ///调参部分
    const int sample_size = 600;                                     //拟合采样数据数

    //****函数区******//
    void taskInit();                                                //任务周期初始化
    void Init(long long timestamp, double p_time);                         //每帧初始化
    void binImg(const Mat& src, Mat& dst, int dealColor);
    void binROI(const Mat& src, Mat& dst, const int team);//预处理
    bool findFans(const cv::Mat &src);                              //寻找锤形扇叶
    bool findtagpoint(Mat & dst,const int find_color_energy);       //寻找装甲板
    bool getROIofR();                                               //寻找R标志所在区域
    bool findRinROI(Mat& src);                                      //所在区域内找R标志

    void getTargetPolarAngle();                                     //获取目标极坐标值
    void getInfo();                                                 //提取多帧间信息
    void getTimeDura(vector<time_angle>&data,  int dura, int & start_idx, int & end_idx, int & last_idx);  //获取时间段对应点，单位ms

    void getDirection();
    float getChangeAngle(float deltaAngle);                       //获取跳转角度
    float predictAngle(uint8_t mode);
    void angle2points_PIX();
    void angle2points_PIX(Point2f& point, double angle);

    float getAcc();

};





#endif //ARMOURSHOT_ENERGY_H