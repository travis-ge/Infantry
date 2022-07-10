#include "armour.h"
#include "kalman.h"
#include "base.h"
#include "serial_port.h"
#include "common.h"
#include "tool_fun.h"
#include <mutex>
#include "common.h"

extern std::queue<pair<std::chrono::time_point<std::chrono::steady_clock>, cv::Mat>> img_buf;
extern SerialPort port;
extern std::mutex mImg_buf;
std::mutex mTmp_armour_queue;
std::mutex mArmour_queue;
std::mutex mDst_queue;
std::mutex mSrc;
extern Ptz_infor stm;
float armour_small_pt[5][2] = {
        0.0, 0.0,
        0.0675, 0.0,
        0.0, 0.0670,
        -0.0675, 0.0,
        0.0, -0.0670,
};
float armour_big_pt[5][2] = {
        0.0,0.0,       //世界坐标系原点
        0.1125,0.0,    //右
        0.0,0.1120,    //上
        -0.1125,0.0,   //左
        0.0,-0.1120,   //下
};
Armour::Armour() {

    FileStorage fs1(PROJECT_DIR"/config/params4.yml", FileStorage::READ);
    fs1["armour"]["red_spilt_threshold"] >> r_spilt_threshold;
    fs1["armour"]["red_gray_threshold"] >> r_gray_threshold;
    fs1["armour"]["red_green_threshold"] >> r_green_threshold;
    fs1["armour"]["blue_spilt_threshold"] >> b_spilt_threshold;
    fs1["armour"]["blue_gray_threshold"] >> b_gray_threshold;
    fs1["armour"]["blue_green_threshold"] >> b_green_threshold;
    fs1["armour"]["debug"] >> debug;
    fs1["armour"]["light_contour_min"] >> light_contour_min;
    fs1.release();
//    numClass = make_shared<Classifier>();
    angleSolver = make_shared<AngleSolver>();
    mlp = make_shared<NumClassifier>(PROJECT_DIR"/config/fc.onnx", PROJECT_DIR"/config/label.txt", 0.8);
    ekf = make_shared<EKFPredictor>();
}

Armour::~Armour() {

}

/**
 *
 * @param filename
 * @return
 */
bool Armour::readCameraParameters(string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}
/**
// *
// * @param src
// * @param dst
// * @param team
// */
void Armour::img_pretreatment(const cv::Mat &src, cv::Mat &dst, int team) {
    cv::split(src, src_split_);
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    if (debug) {
        namedWindow("armour", CV_WINDOW_AUTOSIZE);
    }
    if (team == 2) {//red
        if (debug) {
            cvCreateTrackbar("r_spilt_threshold", "armour", &r_spilt_threshold, 255);
            cvCreateTrackbar("r_gray_threshold", "armour", &r_gray_threshold, 255);
            cvCreateTrackbar("r_green_threshold", "armour", &r_green_threshold, 255);
        }
        subtract(src_split_[2], src_split_[1], src_separation);
//        subtract(src_split_[2], src_split_[1], src_green);
        threshold(src_separation, src_separation, r_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, r_gray_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_green, src_green, r_green_threshold, 255, cv::THRESH_BINARY);
//        dilate(src_green, src_green, element5_7);
//        dilate(src_green, src_green, element7_9);
//        dilate(src_separation, src_separation, element3_5);
//        dilate(src_separation, src_separation, element5_7);
//        erode(src_separation, src_separation, element3_5);
//        dst = src_separation & src_gray & src_green;
        dilate(src_separation, src_separation, element7_9);
        erode(src_separation, src_separation, element3);
        dst = src_separation & src_gray;
//        imshow("sparation",src_separation);
//        dilate(dst, dst, element3_5);
    } else {//blue
        if (debug) {
            cvCreateTrackbar("b_spilt_threshold", "armour", &b_spilt_threshold, 255);
            cvCreateTrackbar("b_gray_threshold", "armour", &b_gray_threshold, 255);
            cvCreateTrackbar("b_green_threshold", "armour", &b_green_threshold, 255);
        }
        subtract(src_split_[0], src_split_[1], src_separation);
//        subtract(src_split_[0], src_split_[1], src_green);
        threshold(src_separation, src_separation, b_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, b_gray_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_green, src_green, b_green_threshold, 255, cv::THRESH_BINARY);
//        dilate(src_green, src_green, element5_7);
//        dilate(src_green, src_green, element7_9);
//
//        dilate(src_separation, src_separation, element3_5);
//        dilate(src_separation, src_separation, element5_7);
//        erode(src_separation, src_separation, element3_5);
//        dst = src_separation & src_gray & src_green;
//        dilate(dst, dst, element3_5);
//        erode(dst, dst, element3_5);
        dilate(src_separation, src_separation, element7_9);
        erode(src_separation, src_separation, element3);
        dst = src_separation & src_gray;
    }
    if (debug) {
        imshow("dst", dst);
        imshow("gray", src_gray);
//        imshow("green",src_green);
    }
}
///**
// *
// * @param src
// * @param dst
// * @param team
// */
//void Armour::img_pretreatment(const cv::Mat &src, cv::Mat &dst, int team) {
//    cv::split(src,src_split_);
//    cv::cvtColor(src, src_gray,CV_BGR2GRAY);
//    if(debug){
//        namedWindow("armour", CV_WINDOW_AUTOSIZE);
//    }
//    if(team == 2){//red
//        if(debug){
//            cvCreateTrackbar("r_spilt_threshold", "armour", &r_spilt_threshold, 255);
//            cvCreateTrackbar("r_gray_threshold", "armour", &r_gray_threshold, 255);
//            cvCreateTrackbar("r_green_threshold", "armour", &r_green_threshold, 255);
//        }
//        subtract(src_split_[2], src_split_[0],src_separation);
//        subtract(src_split_[2], src_split_[1], src_green);
//        threshold(src_separation, src_separation, r_spilt_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_gray, src_gray, r_gray_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_green, src_green, r_green_threshold, 255, cv::THRESH_BINARY);
//        dilate(src_green, src_green, element5_7);
//        dilate(src_green, src_green, element7_9);
//        dilate(src_separation, src_separation, element3_5);
//        dilate(src_separation, src_separation, element5_7);
//        erode(src_separation, src_separation, element3_5);
//        dst = src_separation & src_gray & src_green;
//        dilate(dst, dst, element3_5);
//    }else{//blue
//        if(debug){
//            cvCreateTrackbar("b_spilt_threshold", "armour", &b_spilt_threshold, 255);
//            cvCreateTrackbar("b_gray_threshold", "armour", &b_gray_threshold, 255);
//            cvCreateTrackbar("b_green_threshold", "armour", &b_green_threshold, 255);
//        }
//        subtract(src_split_[0], src_split_[2],src_separation);
//        subtract(src_split_[0], src_split_[1], src_green);
//        threshold(src_separation, src_separation, b_spilt_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_gray, src_gray, b_gray_threshold, 255, cv::THRESH_BINARY);
//        threshold(src_green, src_green, b_green_threshold, 255, cv::THRESH_BINARY);
//        dilate(src_green, src_green, element5_7);
//        dilate(src_green, src_green, element7_9);
//
//        dilate(src_separation, src_separation, element3_5);
//        dilate(src_separation, src_separation, element5_7);
//        erode(src_separation, src_separation, element3_5);
//        dst = src_separation & src_gray & src_green;
//        dilate(dst, dst, element3_5);
//        erode(dst, dst, element3_5);
//    }
//    if(debug){
//        imshow("dst", dst);
//        imshow("gray", src_gray);
//        imshow("green",src_green);
//    }
//}

/**
 *
 * @param src
 * @return
 */
bool Armour::Armor_Detector(const Mat &src,double time_stamp) {
    tmp_armour_vec.clear();
    cv::RotatedRect lightBar, lightBar_fitEllipse, lightBar_minAreaRect;
    std::vector<cv::RotatedRect> v_lightBar;
    Mat src_contours;
    src_contours = src;
    vector<vector<Point>> contours;
    vector<Vec4i> white_hierarchy;
    findContours(src_contours, contours, white_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    for (unsigned int i = 0; i < contours.size(); i++) {
        if (contours[i].size() < light_contour_min)
            continue;
        lightBar_fitEllipse = fitEllipse(contours[i]);
        lightBar_minAreaRect = minAreaRect(contours[i]);

        lightBar.angle = lightBar_fitEllipse.angle;
        lightBar.center = lightBar_fitEllipse.center;
        if (lightBar_minAreaRect.size.width > lightBar_minAreaRect.size.height) {
            lightBar.size.height = lightBar_minAreaRect.size.width;
            lightBar.size.width = lightBar_minAreaRect.size.height;
        } else {
            lightBar.size.height = lightBar_minAreaRect.size.height;
            lightBar.size.width = lightBar_minAreaRect.size.width;
        }

        if ((lightBar.size.width / lightBar.size.height) > 1)
            continue;
        int x = lightBar.center.x - lightBar.size.width;
        if (x < 0)
            continue;
        int y = lightBar.center.y - lightBar.size.height;
        if (y < 0)
            continue;
        int w = lightBar.size.width + lightBar.size.width;
        if (w > src.cols - x)
            continue;
        int h = lightBar.size.height + lightBar.size.width;
        if (h > src.rows - y)
            continue;

        if ((lightBar.angle < 45 || lightBar.angle > 135) &&
            (lightBar.size.height > 5) && (lightBar.size.height < 150)) {
            v_lightBar.push_back(lightBar);
        }
    }

    if (v_lightBar.size() < 2){
        return false;
    }
    Tmp_armour tmp_armour;
    for (unsigned int i = 0; i < v_lightBar.size() - 1; i++) {
        for (unsigned int j = i + 1; j < v_lightBar.size(); j++) {
            double height_diff = abs(v_lightBar[i].size.height - v_lightBar[j].size.height);//高度差
            double height_sum = v_lightBar[i].size.height + v_lightBar[j].size.height;//高度和
            double width_diff = abs(v_lightBar[i].size.width - v_lightBar[j].size.width);//宽度差
            double width_sum = v_lightBar[i].size.width + v_lightBar[j].size.width;//宽度和
            double angle_diff = fabs(v_lightBar[i].angle - v_lightBar[j].angle);//角度差
            double Y_diff = abs(v_lightBar[i].center.y - v_lightBar[j].center.y);//纵坐标差值
            double MH_diff = (min(v_lightBar[i].size.height, v_lightBar[j].size.height)) * 2 / 3;//高度差限幅
            double height_max = (max(v_lightBar[i].size.height, v_lightBar[j].size.height));//最大高度
            double X_diff = abs(v_lightBar[i].center.x - v_lightBar[j].center.x);//横坐标差值
        
            if (Y_diff < 0.8*height_max && X_diff < height_max * 5 &&
                (angle_diff < 5 || 180 - angle_diff < 5) &&
                /*lightBar_dis / v_lightBar[i].size.height >0.5 &&*/
                height_diff / height_max < 0.5 &&
                width_diff / width_sum < 0.4  &&
                X_diff / MH_diff > 2)//还可以加入高度差限制
            {
                tmp_armour.armour_rect.center.x = ((v_lightBar[i].center.x + v_lightBar[j].center.x) / 2);
                tmp_armour.armour_rect.center.y = ((v_lightBar[i].center.y + v_lightBar[j].center.y) / 2);
                tmp_armour.armour_rect.angle = (v_lightBar[i].angle + v_lightBar[j].angle) / 2;

                if (v_lightBar[i].center.x < v_lightBar[j].center.x) {
                    tmp_armour.v_Pts_L = cv::Point2f(v_lightBar[i].center);
                    tmp_armour.v_Pts_R = cv::Point2f(v_lightBar[j].center);
                } else {
                    tmp_armour.v_Pts_L = cv::Point2f(v_lightBar[j].center);
                    tmp_armour.v_Pts_R = cv::Point2f(v_lightBar[i].center);
                }
                if (180 - angle_diff < 5)
                    tmp_armour.armour_rect.angle += 90;
                int nL = (v_lightBar[i].size.height + v_lightBar[j].size.height) / 2; //装甲的高度
                int nW = sqrt((v_lightBar[i].center.x - v_lightBar[j].center.x) *
                              (v_lightBar[i].center.x - v_lightBar[j].center.x) +
                              (v_lightBar[i].center.y - v_lightBar[j].center.y) *
                              (v_lightBar[i].center.y - v_lightBar[j].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                if (nL < nW) {
                    tmp_armour.armour_rect.size.height = nL;
                    tmp_armour.armour_rect.size.width = nW;
                } else {
                    tmp_armour.armour_rect.size.height = nW;
                    tmp_armour.armour_rect.size.width = nL;
                }
                if (Y_diff < nW / 2) {
                    tmp_armour_vec.push_back(tmp_armour);
                }
            }
        }
    }
    if(tmp_armour_vec.size() > 0){
        mTmp_armour_queue.lock();
        if(tmp_armour_queue.size()>5){
            tmp_armour_queue.pop();
        }
        tmp_armour_queue.push(make_pair(time_stamp,tmp_armour_vec));
        std::cout<<"tmp armour queue size "<<tmp_armour_queue.size()<<std::endl;
        mTmp_armour_queue.unlock();
        return true;
    }
    else
        return false;
}

/**
 *
 * @param tg_pt_L
 * @param tg_pt_R
 * @param wh_rate
 * @return
 */
Point3f Armour::getCamCoordinate(Point2f &tg_pt_L, Point2f &tg_pt_R, float wh_rate){
    cx = camMatrix.at<double>(0, 2);
    cy = camMatrix.at<double>(1, 2);
    fx = camMatrix.at<double>(0, 0);
    fy = camMatrix.at<double>(1, 1);
    corners.clear();
    observationPts.clear();
    Point3f tmp;
    if (wh_rate > 3.3 ) {
        std::cout << "big armour" << std::endl;
        for (int i = 0; i < 5; i++) {
            tmp.x = armour_big_pt[i][0];
            tmp.y = armour_big_pt[i][1];
            tmp.z = 0;
            corners.push_back(tmp);
        }
    } else {
        std::cout << "small armour" << std::endl;
        for (int i = 0; i < 5; i++) {
            tmp.x = armour_small_pt[i][0];
            tmp.y = armour_small_pt[i][1];
            tmp.z = 0;
            corners.push_back(tmp);
        }
    }
    //得到pattern-cam坐标
    tg_center.x = (tg_pt_L.x + tg_pt_R.x) * 0.5;
    tg_center.y = (tg_pt_L.y + tg_pt_R.y) * 0.5;
    observationPts.push_back(tg_center);
    observationPts.push_back(tg_pt_R);

    cv::Point2f temp_pt_1(tg_pt_R.x - tg_center.x, tg_pt_R.y - tg_center.y);
    cv::Point2f temp_pt_2(-temp_pt_1.y, temp_pt_1.x);
    cv::Point2f tg_pt_down(temp_pt_2.x + tg_center.x, temp_pt_2.y + tg_center.y);
    observationPts.push_back(tg_pt_down);
    observationPts.push_back(tg_pt_L);
    cv::Point2f temp_pt_3(tg_pt_L.x - tg_center.x, tg_pt_L.y - tg_center.y);
    cv::Point2f temp_pt_4(-temp_pt_3.y, temp_pt_3.x);
    cv::Point2f tg_pt_up(temp_pt_4.x + tg_center.x, temp_pt_4.y + tg_center.y);
    observationPts.push_back(tg_pt_up);

    Mat rvec, tvec;
    cv::solvePnP(cv::Mat(corners), cv::Mat(observationPts), camMatrix, distCoeffs, rvec, tvec, false);
    double Z = tvec.at<double>(2, 0);
    double X = tvec.at<double>(0, 0) ;
    double Y = tvec.at<double>(1, 0) ;
//    double X = (tg_center.x - cx) * Z / fx ;
//    double Y = (tg_center.y - cy) * Z / fy ;
    m_position = Point3f (X,Y,Z);
    return Point3f (X,Y,Z);
}
/**
 *
 * @param camPoint
 * @return
 */
cv::Point2f Armour::cam2pixel(cv::Point3f camPoint) {
    float pixel_x = camPoint.x * fx / camPoint.z + cx;
    float pixel_y = camPoint.y * fy / camPoint.z + cy;
    return cv::Point2f (pixel_x, pixel_y);
}
/**
 *
 * @param rect
 * @param src
 */
void Armour::draw_target(RotatedRect rect, Mat &src) {
    Point2f point[4];
    rect.points(point);
    for (int i = 0; i < 4; i++) {
        line(src, point[i], point[(i + 2) % 4], Scalar(0, 255, 0), 2);
    }
}
/**
 *
 * @param angle_p
 * @param angle_y
 * @return
 */
bool Armour::ifShoot(double angle_p, double angle_y) {
    if(abs(angle_p) < 1.0 && abs(angle_y) < 1.5)
        return true;
    else
        return false;
}
#define K_SPEED 10000
void Armour::armour_detect() {
    if (!readCameraParameters("../config/cameraParams_157.xml")) {
        std::cout << "read camera param fail ..." << std::endl;
    }
    auto start_time = chrono::steady_clock::now();
    while (true){
        std::chrono::time_point<std::chrono::steady_clock> t1;
        cv::Mat img;
        double time_stamp;
        mImg_buf.lock();
        if(img_buf.size()>0){
            std::cout<<"img buff "<<img_buf.size()<<std::endl;
            t1 = img_buf.front().first;
            time_stamp = chrono::duration<double>(t1 - start_time).count();//s
//            std::cout<<"time stamp "<<time_stamp<<std::endl;
            mSrc.lock();
            src =  img_buf.front().second.clone();
            mSrc.unlock();
            img_buf.pop();
        }
        mImg_buf.unlock();
        mSrc.lock();
        if(!src.data){
            mSrc.unlock();
            continue;
        }
        mSrc.unlock();

#ifdef SHOW_IMG
        src = img.clone();
#endif
        auto start = chrono::steady_clock::now();
        uint8_t find_color_armour = 2;
        if (port.receive[4] > K_SPEED) {   /// 我方是蓝色，击打红色
            find_color_armour = 2;
            std::cout << "our team is blue, shoot red!!" << std::endl;
        }
        if (port.receive[4] < K_SPEED) {
            find_color_armour = 1;
            std::cout << "our team is red, shoot blue!!" << std::endl;
        }
        cv::Mat dst;
        STATE mode = getMode(port.receive[1]);
        switch (mode) {
            case STATE_BUFF:
                ;
                break;
            default:{
//                std::cout<<" << default mode  >>  " <<std::endl;
                mSrc.lock();
                img_pretreatment(src,dst,find_color_armour);
                mSrc.unlock();
                Armor_Detector(dst,time_stamp);
//                imshow("dst",dst);
//                cv::waitKey(1);
            }
        }
        auto end = chrono::steady_clock::now();
        double cost = chrono::duration<double,milli>(end-start).count();
        cout<<" armour detect cost "<< cost<<" ms"<<endl;
    }

}
/**
 *
 * @param src
 * @param P_l
 * @param P_r
 * @return
 */
void Armour::armourSort() {
    while (true){
        auto start = chrono::steady_clock::now();
        vector<Tmp_armour> armour_seq;
        double time_stamp;
        mTmp_armour_queue.lock();
//        std::cout<<"sort tmp armour size "<<tmp_armour_queue.size()<<std::endl;
        if(tmp_armour_queue.size()>0){
            std::cout<<"tmp_armour_queue size "<<tmp_armour_queue.size()<<std::endl;
            time_stamp = tmp_armour_queue.front().first;
            armour_seq = tmp_armour_queue.front().second;
            tmp_armour_queue.pop();
        }else{
            mTmp_armour_queue.unlock();
            continue;
        }
        mTmp_armour_queue.unlock();
        if (armour_seq.size() == 0){ continue;}
        int idx = 0;
        for(auto i = armour_seq.begin(); i != armour_seq.end(); ){
            /*cut number area*/
            Point2f srcRect[4], dstRect[4],P[4];
            i->armour_rect.points(P);
            //矫正长宽
            float width =tool::getDistance(P[0],P[1]);
            float height=tool::getDistance(P[1],P[2]);
            if(width>height){
                srcRect[0]=P[0]; srcRect[1]=P[1];
                srcRect[2]=P[2]; srcRect[3]=P[3];
            }else{
                swap(width,height);
                srcRect[0]=P[1]; srcRect[1]=P[2];
                srcRect[2]=P[3]; srcRect[3]=P[0];
            }
            if( (srcRect[0].y>srcRect[3].y) && (srcRect[0].x>srcRect[1].x) ){
                swap(srcRect[0], srcRect[2]);
                swap(srcRect[1], srcRect[3]);
            }
            float scale_x = 0.25;
            float scale_y = 0.4;
            //高度上,默认放大
            srcRect[0].y -= height * scale_y ; srcRect[1].y -= height * scale_y ;
            srcRect[3].y += height * scale_y ; srcRect[2].y += height * scale_y ;
            //宽度上，默认缩小
            srcRect[0].x += width * scale_x ; srcRect[1].x -= width * scale_x ;
            srcRect[3].x += width * scale_x ; srcRect[2].x -= width * scale_x ;
            //按照顺序存入目标矩形点坐标
            dstRect[0]=Point2f(0,0);
            dstRect[1]=Point2f(width,0);
            dstRect[2]=Point2f(width,height);
            dstRect[3]=Point2f(0,height);
            Mat transform = getPerspectiveTransform(srcRect,dstRect);
            Mat src1;
            mSrc.lock();
            warpPerspective(src,src1,transform,Size(width, height));
            mSrc.unlock();
//        imshow("warp after", src1);
            int num = 5;
            ///recognize number
//            num = numClass->numPredict(src1);
            num = mlp->predict(src1);
            if(num < 1){
                i = armour_seq.erase(i);
#ifdef SHOW_IMG
                putText(src, "0",tmp_rect.tl(),FONT_HERSHEY_COMPLEX_SMALL,2,Scalar(255,0,0),2);
#endif
                continue;
            }else{
                ++i;
            }
            idx++;
#ifdef SHOW_IMG
            putText(src, to_string(num),tmp_rect.tl(),FONT_HERSHEY_COMPLEX_SMALL,2,Scalar(255,0,0),2);
#endif
        }
        if(armour_seq.size() == 0){
            continue;
        }else if(armour_seq.size() == 1){
            Armour_data armour_data;
            armour_data.armour_rect = armour_seq[0].armour_rect;
            armour_data.armour_PL = cv::Point2f (armour_seq[0].v_Pts_L);
            armour_data.armour_PR = cv::Point2f (armour_seq[0].v_Pts_R);
            mArmour_queue.lock();
            if(armour_queue.size()>5)
                armour_queue.pop();
            armour_queue.push(make_pair(time_stamp,armour_data));
            mArmour_queue.unlock();
        }else{
            double min_dis = 10000;
            int min_idx = -1;
            for(int i = 0; i < armour_seq.size(); i++){
                double dis = fabs(armour_seq[i].armour_rect.center.x - src.cols / 2) +
                             fabs(armour_seq[i].armour_rect.center.y - src.rows / 2);
                if(dis < min_dis){
                    min_dis = dis;
                    min_idx = i;
                }
            }
            if(min_idx > 0){
                Armour_data armour_data;
                armour_data.armour_rect = cv::RotatedRect(armour_seq[min_idx].armour_rect);
                armour_data.armour_PL = cv::Point2f (armour_seq[min_idx].v_Pts_L);
                armour_data.armour_PR = cv::Point2f (armour_seq[min_idx].v_Pts_R);
                mArmour_queue.lock();
                if(armour_queue.size()>5)
                    armour_queue.pop();
                armour_queue.push(make_pair(time_stamp,armour_data));
                mArmour_queue.unlock();
            }else{
                continue;
            }
        }
        auto end = chrono::steady_clock::now();
        double cost = chrono::duration<double,milli>(end - start).count();
        cout<<" armour sort cost "<<cost<<" ms"<<endl;
    }

}
/**
 *
 */
 int tg_num = 0;
 int frame_cnt = 0;
 chrono::time_point<chrono::steady_clock> last_t;

#define USE_EKF
void Armour::armour_tracking() {
    last_t = chrono::steady_clock::now();
    while (true){
        auto t0 = chrono::steady_clock::now();
        Armour_data target_armour;
        double time_stamp;
        target_armour.armour_PL = Point2f (-1,-1);
        mArmour_queue.lock();
        if (armour_queue.size()>0){
            std::cout<<"armour queue size "<<armour_queue.size()<<std::endl;
            target_armour = armour_queue.front().second;
            time_stamp = armour_queue.front().first;
            armour_queue.pop();
        }else{
//            mArmour_queue.unlock();
//            continue;
        }
        mArmour_queue.unlock();
        double angle_P, angle_Y, Dis;
        char cmd = 0x30;
        if(target_armour.armour_PL.x> 0){
            cmd = 0x31;
            frame_cnt++;
            std::cout<<"find armour !!!"<<std::endl;
#ifdef SHOW_IMG
            //add thread mutex
            draw_target(target_armour.armour_rect,src);
            imshow("src",src);
//            cv::waitKey(1);
#endif
            Point2f corPoints[4];
            target_armour.armour_rect.points(corPoints);
            float armour_width = sqrt(
                    pow(corPoints[0].x - corPoints[1].x, 2) + pow(corPoints[0].y - corPoints[1].y, 2));
            float armour_height = sqrt(
                    pow(corPoints[1].x - corPoints[2].x, 2) + pow(corPoints[1].y - corPoints[2].y, 2));
            if (armour_height > armour_width) {
                float tmp = armour_width;
                armour_width = armour_height;
                armour_height = tmp;
            }
            float wh_rate = armour_width / armour_height;
//            std::cout<<"wh_rate "<<wh_rate<<std::endl;
            auto camPoint = getCamCoordinate(target_armour.armour_PL,target_armour.armour_PR,wh_rate);
#ifdef USE_EKF
            auto world_point = angleSolver->cam2abs(camPoint,stm);
//            std::cout<<"prepreper" <<time_stamp<<std::endl;
            auto pre_abs = ekf->predict(world_point,time_stamp);
            auto pre_cam = angleSolver->abs2cam(pre_abs,stm);
            angleSolver->getAngle(pre_cam,angle_P,angle_Y,Dis);
#else
            angleSolver->getAngle(camPoint,angle_P,angle_Y,Dis);
#endif
            std::cout<<" == ang_P "<<angle_P
                     <<" == ang_Y "<<angle_Y
                     <<" == Dis "<<Dis
                     <<std::endl;
        }else{
            cmd = 0x30;
            angle_Y = angle_P = Dis = 0;
        }


        auto t2 = chrono::steady_clock::now();
        if((int)chrono::duration<double>(t2 - last_t).count() == 1){
            cout<<" real frame "<<frame_cnt<<std::endl;
            frame_cnt = 0;
            last_t = t2;
        }
        cv::waitKey(3);
        *(signed char *) &port.buff_w_[0] = int16_t(10000 * (angle_P));
        *(signed char *) &port.buff_w_[1] = int16_t((10000 * (angle_P))) >> 8;
        *(signed char *) &port.buff_w_[2] = int16_t(10000 * (angle_Y));
        *(signed char *) &port.buff_w_[3] = int16_t((10000 * (angle_Y))) >> 8;
        *(signed char *) &port.buff_w_[4] = int16_t(100 * Dis);
        *(signed char *) &port.buff_w_[5] = int16_t(100 * Dis) >> 8;
        port.SendBuff(cmd, port.buff_w_, 6);

        auto t1 = chrono::steady_clock::now();
        double cost = chrono::duration<double,milli> (t1-t0).count();
        std::cout<<"armour tracking thread cost "<<cost<<" ms"<<std::endl;
    }
}