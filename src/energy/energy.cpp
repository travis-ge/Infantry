//
// Created by quanyi on 2022/3/6.
//

#include "energy.h"
#include "common.h"
#include "base.h"

//#define PLOT_DRAW

using namespace cv;
using namespace std;
extern Ptz_infor stm;
float armour_energy_pt[5][2] = {
        0.0,0.0,       //世界坐标系原点
        -0.115,-0.0625,    //zuoxia
        -0.115,0.0625,    //zuoshang
        0.115,0.0625,   //youshang
        0.115,-0.0625,   //youxia
};

Energy::Energy() {
    finder = make_shared<ArmourFinder>();
    angleSolver = make_shared<AngleSolver>();
    Point3f tmp;
    for (int i = 0; i < 5; i++) {
        tmp.x = armour_energy_pt[i][0];
        tmp.y = armour_energy_pt[i][1];
        tmp.z = 0;
        realCorners.push_back(tmp);
    }
    if(!loadCameraParams(cameraParam)){
        clog<<"check param file"<<endl;
    }
    Dis_last = 0;
}

Energy::~Energy() {

}
/**
 *
 * @param filename
 * @return
 */
bool Energy::loadCameraParams(std::string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    return true;
}
/**
 *
 * @param src
 * @param img
 * @return
 */
RotatedRect Energy::energy_finder(cv::Mat src, cv::Mat img) {

    std::vector<std::vector<cv::Point>> energy_contours;
    std::vector<cv::Vec4i> hierarchy;

    RotatedRect box = RotatedRect(cv::Point2f(-1, -1), cv::Size2f(0, 0), 0);

    int contour[20] = {0};

    findContours(src, energy_contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));

    if (energy_contours.size() > 0) {
        for (int i = 0; i < energy_contours.size(); i++) {
            //cout<<energy_contours[i].size()<<endl;
            if(energy_contours[i].size() < 80){continue;}
            if(hierarchy[i][3] >20){break;}
            if (hierarchy[i][3] != -1 ) {
                contour[hierarchy[i][3]]++;
            }
        }
        for (int j = 0; j < energy_contours.size(); j++) {
            /*if father contour count is once, then this father contour only has one son contour which is our target*/
            if (j>20){break;}
            if (contour[j] == 1 ) {
                int num = hierarchy[j][2];
                box = minAreaRect(energy_contours[num]);
            }
        }
    }

    return box;
}

/**
 *
 * @param src
 * @param box
 * @param center
 */
void Energy::draw_energy_target(cv::Mat &src, cv::RotatedRect box, cv::Point2f &center) {

    Point2f vertex[4];
    box.points(vertex);
    for (int i = 0; i < 4; i++) {
        line(src, vertex[i], vertex[(i + 1) % 4], Scalar(0, 0, 255), 2, LINE_AA);
    }
    center = (vertex[0] + vertex[2]) / 2;
    putText(src, "target", vertex[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
}

void Energy::draw_energy_target(cv::Mat &src, cv::Point2f *vertex, cv::Point2f &center) {

    for (int i = 0; i < 4; i++) {
        line(src, vertex[i], vertex[(i + 1) % 4], Scalar(0, 0, 255), 2, LINE_AA);
    }
    center = (vertex[0] + vertex[2]) / 2;
    putText(src, "target", vertex[0], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
}

double Energy::getDistance(cv::Point2f point1, cv::Point2f point2) {
    return sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
}
/**
 *
 * @param points
 * @return
 */
bool Energy::sortPoints(cv::Point2f *points) {
    Point2f center = (points[0] + points[2])/2;
    Point2f point_01 = (points[0] + points[1])/2;
    Point2f point_03 = (points[0] + points[3])/2;

    double center_01 = getDistance(center, point_01);
    double center_03 = getDistance(center, point_03);

    if(center_01 > center_03){
        std::clog<<"no need sort"<<std::endl;
        return true;
    }else{
        std::clog<<"need sort"<<std::endl;
        Point2f tmp = points[3];
        points[3] = points[2];
        points[2] = points[1];
        points[1] = points[0];
        points[0] = tmp;
        //std::clog<<""<<std::endl;
        return true;
    }

}
/**
 *
 * @param points
 * @return
 */
bool Energy::getCamP(cv::Point2f *points) {

    const double cx = camMatrix.at<double>(0, 2);
    const double cy = camMatrix.at<double>(1, 2);
    const double fx = camMatrix.at<double>(0, 0);
    const double fy = camMatrix.at<double>(1, 1);
    cameraCorners.clear();
    for (int i = 0; i < 5; i++) {
        cameraCorners.push_back(points[i]);
    }
    //std::cout<<cameraCorners<<endl;
    Mat rvec, tvec;

    cv::solvePnP(cv::Mat(realCorners), cv::Mat(cameraCorners), camMatrix, distCoeffs, rvec, tvec, false);

    double Z = tvec.at<double>(2, 0);
    double X = tvec.at<double>(0,0);
    double Y = tvec.at<double>(1,0);

    camEnergyP = Point3f (X,Y,Z);
    return true;

}
/**
 *
 * @param center
 * @param src_P
 * @param dst_P
 * @param angle
 * @return
 */
void Energy::getPredicPoints(cv::Point2f center, cv::Point2f src_P, cv::Point2f &dst_P, double angle) {
    float d_x = src_P.x - center.x;
    float d_y = src_P.y - center.y;
    dst_P.x = d_x * cos(angle) - d_y * sin(angle) + center.x;
    dst_P.y = d_x * sin(angle) + d_y * cos(angle) + center.y;
//    std::cout<<d_x << " "<< d_y<<" "<<dst_P.x<<" "<<dst_P.y<<std::endl;
}
/**
 *
 * @param src
 * @param time_stamp
 * @param find_color_energy
 * @param mode
 * @return
 */
bool Energy::run(cv::Mat &src, int time_stamp, int find_color_energy, char mode,double & p, double &y, double &d) {
    int mode_chose;
    if(mode == 'a'){mode_chose = 1;}   //small
    if (mode == 'b'){mode_chose = 2;}  //big

    mode_chose = 2;
//    mode_chose = 1;
//    std::cout<<"buff predict time "<<predict_time<<std::endl;
//    int finder_status  = finder->searchArmour(src,time_stamp,predict_time+0.05, find_color_energy,mode_chose);
    int finder_status  = finder->searchArmour(src,time_stamp,predict_time+0.05, find_color_energy,mode_chose);
    if(!finder_status||finder_status==-1){
        return false;
    }
    cv::Point2f center;

    sortPoints(finder->fan_.armour_points);
    Point2f cameraPoints[5];
    cameraPoints[0] = finder->fan_.tag_point;
    for (int i = 1; i < 5; i++) {
        cameraPoints[i] = finder->fan_.armour_points[i-1];
    }

    if(finder_status == 2){
        ;
    }else if(finder_status == 1){
        cv::Point2f precamP[5];
        for (int i = 0; i < 5; i++) {
            getPredicPoints(finder->circle_center_point,cameraPoints[i], precamP[i],finder->fan_.pre_angle);

        }
        circle(src,precamP[0],10,Scalar(0,255,0),-1);
#ifdef PLOT_DRAW
        draw_energy_target(src,finder->fan_.armour_points,center);
        circle(src,precamP[0],6,Scalar(0,255,0),-1);

        circle(src,finder->circle_center_point,10,Scalar(0,0,255),-1);
#endif
        if(finder->fan_.pre_angle == 0 && last_p !=0){
            p = last_p;
            y = last_y;
            d = last_d;
            last_y = last_p =last_d = 0;
        }else{
            getCamP(precamP);
            angleSolver->getAngle(camEnergyP,p,y,d);
            d = 7.1;
            last_p = p;
            last_y = y;
            last_d = d;
            double dis = (d + Dis_last)/2;
            predict_time = dis / stm.bulletSpeed;
            Dis_last = dis;
        }
        return true;
    }

}
void Energy::energy_init() {
    finder->is_inited = false;
}