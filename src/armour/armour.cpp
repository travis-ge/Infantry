#include "armour.h"
#include "base.h"
#include "serial_port.h"
#include "common.h"
#include "tool_fun.h"
#include <mutex>
#include "camera_api.h"

extern pair<std::chrono::time_point<std::chrono::steady_clock>, cv::Mat> img_buf;
extern SerialPort port;
extern std::mutex mImg_buf;
extern Ptz_infor stm;
extern Send send_data;
extern bool expose_time_reset;
extern bool expose_time_set;
float armour_small_pt[5][2] = {
        0.0, 0.0,
        0.0675, 0.0,
        0.0, 0.0670,
        -0.0675, 0.0,
        0.0, -0.0670,
};
float armour_big_pt[5][2] = {
        0.0, 0.0,       //世界坐标系原点
        0.1125, 0.0,    //右
        0.0, 0.1120,    //上
        -0.1125, 0.0,   //左
        0.0, -0.1120,   //下
};

Armour::Armour() {

    FileStorage fs1(Param, FileStorage::READ);
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
    numClass = make_shared<NumClassifier>(PROJECT_DIR"/config/fc.onnx",PROJECT_DIR"/config/label.txt",0.6);
    angleSolver = make_shared<AngleSolver>();
    ekf = make_shared<EKFPredictor>();
    energy = make_shared<Energy>();
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
//void Armour::imgPretreatment(const cv::Mat &src, cv::Mat &dst, int team) {
//    cv::split(src, src_split_);
//    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
//    if (debug) {
//        namedWindow("armour", CV_WINDOW_AUTOSIZE);
//    }
//    if (team == 2) {//red
//        if (debug) {
//            cvCreateTrackbar("r_spilt_threshold", "armour", &r_spilt_threshold, 255);
//            cvCreateTrackbar("r_gray_threshold", "armour", &r_gray_threshold, 255);
//            cvCreateTrackbar("r_green_threshold", "armour", &r_green_threshold, 255);
//        }
//        subtract(src_split_[2], src_split_[0], src_separation);
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
//    } else {//blue
//        if (debug) {
//            cvCreateTrackbar("b_spilt_threshold", "armour", &b_spilt_threshold, 255);
//            cvCreateTrackbar("b_gray_threshold", "armour", &b_gray_threshold, 255);
//            cvCreateTrackbar("b_green_threshold", "armour", &b_green_threshold, 255);
//        }
//        subtract(src_split_[0], src_split_[2], src_separation);
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
//    if (debug) {
//        imshow("dst", dst);
//        imshow("gray", src_gray);
//        imshow("green", src_green);
//        cv::waitKey(1);
//    }
//}
void Armour::imgPretreatment(const cv::Mat &src, cv::Mat &dst, int team) {
    cv::split(src, src_split_);
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    if (debug) {
        namedWindow("armour", CV_WINDOW_AUTOSIZE);
    }
    if (team == 2) {//red
        if (debug) {
            cvCreateTrackbar("r_spilt_threshold", "armour", &r_spilt_threshold, 255);
            cvCreateTrackbar("r_gray_threshold", "armour", &r_gray_threshold, 255);
        }
        subtract(src_split_[2], src_split_[1], src_separation);
        threshold(src_separation, src_separation, r_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, r_gray_threshold, 255, cv::THRESH_BINARY);
        dilate(src_separation, src_separation, element7_9);
        dst = src_separation & src_gray;
    } else {//blue
        if (debug) {
            cvCreateTrackbar("b_spilt_threshold", "armour", &b_spilt_threshold, 255);
            cvCreateTrackbar("b_gray_threshold", "armour", &b_gray_threshold, 255);
        }
        subtract(src_split_[0], src_split_[1], src_separation);
        threshold(src_separation, src_separation, b_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, b_gray_threshold, 255, cv::THRESH_BINARY);
        dilate(src_separation,src_separation,element7_9);
        dst = src_separation & src_gray;
    }
    if (debug) {
        imshow("dst", dst);
        imshow("gray", src_gray);
        imshow("sep", src_separation);
        cv::waitKey(1);
    }
}

/**
 *
 * @param src
 * @return
 */
bool Armour::armourDetector(const Mat &src) {
    tmp_armour_vec.clear(); // all armour vector
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

        if ((lightBar.size.width / lightBar.size.height) > 0.8)
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

    if (v_lightBar.size() < 2) {
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

            if (Y_diff < 0.8 * height_max && X_diff < height_max * 5 &&
                (angle_diff < 5 || 180 - angle_diff < 5) &&
                /*lightBar_dis / v_lightBar[i].size.height >0.5 &&*/
                height_diff / height_max < 0.3 &&
                width_diff / width_sum < 0.4 &&
                X_diff / MH_diff > 2) {
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
    ///check if vector is empty!
    if (tmp_armour_vec.size() > 0) {
        return true;
    } else
        return false;
}

/**
 *
 * @param tg_pt_L
 * @param tg_pt_R
 * @param wh_rate
 * @return
 */
Point3f Armour::getCamCoordinate(Point2f &tg_pt_L, Point2f &tg_pt_R, float wh_rate) {
    cx = camMatrix.at<double>(0, 2);
    cy = camMatrix.at<double>(1, 2);
    fx = camMatrix.at<double>(0, 0);
    fy = camMatrix.at<double>(1, 1);
    corners.clear();
    observationPts.clear();
    Point3f tmp;
    if (wh_rate > 3.3) {
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
//    double X = tvec.at<double>(0, 0);
//    double Y = tvec.at<double>(1, 0);
    double X = (tg_center.x - cx) * Z / fx ;
    double Y = (tg_center.y - cy) * Z / fy ;
    return Point3f(X, Y, Z);
}

/**
 *
 * @param camPoint
 * @return
 */
cv::Point2f Armour::cam2pixel(cv::Point3f camPoint) {
    float pixel_x = camPoint.x * fx / camPoint.z + cx;
    float pixel_y = camPoint.y * fy / camPoint.z + cy;
    return cv::Point2f(pixel_x, pixel_y);
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
        line(src, point[i], point[(i + 1) % 4], Scalar(0, 255, 0), 2);
    }
}

/**
 *
 * @param angle_p
 * @param angle_y
 * @return
 */
bool Armour::ifShoot(double angle_p, double angle_y) {
    if (abs(angle_p) < 1.0 && abs(angle_y) < 1.5)
        return true;
    else
        return false;
}


/**
 *2th
 * @param src
 * @param P_l
 * @param P_r
 * @return
 */
bool Armour::armourSort(void) {
    if (tmp_armour_vec.size() == 0) { return false; }
    for (auto i = tmp_armour_vec.begin(); i != tmp_armour_vec.end();) {
        /*cut number area*/
        Point2f srcRect[4], dstRect[4], P[4];
        i->armour_rect.points(P);
        //矫正长宽
        float width = tool::getDistance(P[0], P[1]);
        float height = tool::getDistance(P[1], P[2]);
        if (width > height) {
            srcRect[0] = P[0];
            srcRect[1] = P[1];
            srcRect[2] = P[2];
            srcRect[3] = P[3];
        } else {
            swap(width, height);
            srcRect[0] = P[1];
            srcRect[1] = P[2];
            srcRect[2] = P[3];
            srcRect[3] = P[0];
        }
        if ((srcRect[0].y > srcRect[3].y) && (srcRect[0].x > srcRect[1].x)) {
            swap(srcRect[0], srcRect[2]);
            swap(srcRect[1], srcRect[3]);
        }
        float scale_x = 0.2;
        float scale_y = 0.4;
        //高度上,默认放大
        srcRect[0].y -= height * scale_y;
        srcRect[1].y -= height * scale_y;
        srcRect[3].y += height * scale_y;
        srcRect[2].y += height * scale_y;
        //宽度上，默认缩小
        srcRect[0].x += width * scale_x;
        srcRect[1].x -= width * scale_x;
        srcRect[3].x += width * scale_x;
        srcRect[2].x -= width * scale_x;
        //按照顺序存入目标矩形点坐标
        dstRect[0] = Point2f(0, 0);
        dstRect[1] = Point2f(width, 0);
        dstRect[2] = Point2f(width, height);
        dstRect[3] = Point2f(0, height);
        Mat transform = getPerspectiveTransform(srcRect, dstRect);
        Mat src1;
        if (roi_ready) {
            warpPerspective(roi, src1, transform, Size(width, height));
            std::cout << "roi ready " << std::endl;
        } else
            warpPerspective(src, src1, transform, Size(width, height));
//        imshow("warp after", src1);
        int num = -1;
        ///recognize number
//        num = numClass->numPredict(src1);
        num = numClass->predict(src1);
        if (num > 7) {
            i = tmp_armour_vec.erase(i);
            continue;
        } else {
            i->id = num;
            ++i; // with right id (pl rl id )
        }
    }
    if (tmp_armour_vec.size() == 0) {
        return false;
    } else {
        return true;
    }
}

/**
 *3th
 */
int tg_num = 0;
int frame_cnt = 0;
chrono::time_point<chrono::steady_clock> last_t;
#define USE_PRE
//#define SHOW_IMG

void Armour::armour_tracking() {
    last_t = chrono::steady_clock::now();
    /**creat searching ROI **/
    int tg_num = 0;
    char cmd = 0x30;
    Armour_data target_armour;
    target_armour.armour_PL = Point2f(-1, -1);
    if (tmp_armour_vec.size()) { tg_num = tmp_armour_vec.size(); }
    std::cout << "tg_num " << tg_num << std::endl;
    if (tg_num) {
        if (tmp_armour_vec.size() == 1) {
            target_armour.armour_rect = tmp_armour_vec[0].armour_rect;
            target_armour.armour_PL = tmp_armour_vec[0].v_Pts_L;
            target_armour.armour_PR = tmp_armour_vec[0].v_Pts_R;
            target_armour.id = tmp_armour_vec[0].id;
        } else if (tmp_armour_vec.size() > 1) {
            if (!id_locked) {
                double min_dis = 10000;
                int min_idx = -1;
                for (int i = 0; i < tmp_armour_vec.size(); i++) {
                    double dis = fabs(tmp_armour_vec[i].armour_rect.center.x - src.cols / 2) +
                                 fabs(tmp_armour_vec[i].armour_rect.center.y - src.rows / 2);
                    if (dis < min_dis) {
                        min_dis = dis;
                        min_idx = i;
                    }
                }
                if (min_dis > 0) {
                    target_armour.armour_rect = cv::RotatedRect(tmp_armour_vec[min_idx].armour_rect);
                    target_armour.armour_PL = cv::Point2f(tmp_armour_vec[min_idx].v_Pts_L);
                    target_armour.armour_PR = cv::Point2f(tmp_armour_vec[min_idx].v_Pts_R);
                    target_armour.id = int(tmp_armour_vec[min_idx].id);
                    select_id = tmp_armour_vec[min_idx].id;
                    if (select_id == last_id) {
                        id_cnt++;
                        tmp_losing = 0;
                    } else {
                        tmp_losing++;
                        if (tmp_losing > 5) {
                            id_cnt = 0;
                        }
                    }
                    last_id = select_id;
                    if (id_cnt) {
                        id_locked = true;
                        tmp_losing = 0;
                        id_cnt = 0;
                        std::cout << "id locked !!" << std::endl;
                    }
                }
            } else {//tracking
                double lose_count = 0;
                for (auto &armour_data: tmp_armour_vec) {
                    if (armour_data.id == select_id) {
                        target_armour.armour_rect = armour_data.armour_rect;
                        target_armour.armour_PL = armour_data.v_Pts_L;
                        target_armour.armour_PR = armour_data.v_Pts_R;
                        target_armour.id = armour_data.id;
                        track_cnt++;
                        std::cout << "tracking id " << select_id << std::endl;
                        id_losing_cnt = 0;
                        break;
                    }
                    lose_count++;
                }
                if (lose_count == tmp_armour_vec.size()) {
                    id_losing_cnt++;
//                        target_armour = armour_queue.back().second;
                }
                if (id_losing_cnt > 20) {
                    id_locked = false;
                    select_id = -1;
                    last_id = 0;
                    id_cnt = 0;
                    track_cnt = 0;
                }
            }
        }
    } else {
        if(last_find_f){
            cmd = 0x31;
            send_data = last_send;
            last_find_f = 0;
        } else{
            cmd = 0x30;
            send_data = {0, 0, 0};
            losing_cnt++;
        }
    }
    /// select armour is real
    if (target_armour.armour_PL.x > 0) {
        find_cnt++;
        if (roi_ready) {
            //imshow("armour roi",img_process);
            target_armour.armour_PL.x += armourROI.x;
            target_armour.armour_PL.y += armourROI.y;
            target_armour.armour_PR.x += armourROI.x;
            target_armour.armour_PR.y += armourROI.y;
            target_armour.armour_rect.center.x += armourROI.x;
            target_armour.armour_rect.center.y += armourROI.y;
        }
        if (tracking_flag) {
            cv::Rect tmpROI = target_armour.armour_rect.boundingRect();
            int roi_x = tmpROI.x - 0.7 * tmpROI.width;
            int roi_y = tmpROI.y - 0.5 * tmpROI.width;
            int roi_w = 2.5 * tmpROI.width;
            int roi_h = 2 * tmpROI.width;
            if (roi_x < 0) { roi_x = 0; }
            if (roi_y < 0) { roi_y = 0; }
            if (roi_x + roi_w > src.cols) { roi_w = src.cols - roi_x; }
            if (roi_y + roi_h > src.rows) { roi_h = src.rows - roi_y; }
            armourROI = Rect(roi_x, roi_y, roi_w, roi_h);
            rectangle(src, armourROI, Scalar(255, 0, 0), 2);
            if (armourROI.width > 0) {
                roi_ready = 1;
            }
        }
        cmd = 0x31;
        frame_cnt++;
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
        auto camPoint = getCamCoordinate(target_armour.armour_PL, target_armour.armour_PR, wh_rate);
        auto pixelPoint = cam2pixel(camPoint);
//            std::cout<<"============================= abs "<<world_point<<std::endl;
#ifdef USE_PRE
        Ptz_infor use_stm;
        if (!ekf->is_inited) {
            current_yaw = stm.yaw;
        }
        use_stm.yaw = stm.yaw - current_yaw;
        use_stm.pitch = stm.pitch;
        use_stm.bulletSpeed = stm.bulletSpeed;
        auto world_point = angleSolver->cam2abs(camPoint, use_stm);
        target_armour.world_point = world_point;
//            std::cout<<"prepreper" <<target_armour.id<<std::endl;
        Armour_case a;
        a.world_point = target_armour.world_point;
        a.id = target_armour.id;
        auto pre_abs = ekf->predict(a, time_stamp);
        auto pre_cam = angleSolver->abs2cam(pre_abs, use_stm);
        auto pre_pixel = cam2pixel(pre_cam);
        if(fabs(pre_pixel.x - pixelPoint.x) > 0.5*src.cols || fabs(pre_pixel.y-pixelPoint.y) > 0.5*src.rows){
            ekf->is_inited = false;
            angleSolver->getAngle(camPoint, send_data.pitch, send_data.yaw, send_data.dis);
        }else
            angleSolver->getAngle(pre_cam, send_data.pitch, send_data.yaw, send_data.dis);
        last_send = {send_data.pitch,send_data.yaw,send_data.dis};
        last_find_f = 1;
#else
        angleSolver->getAngle(camPoint,angle_P,angle_Y,Dis);
        auto world_point = angleSolver->cam2abs(camPoint,stm);
        //            angleSolver->getAngle(camPoint,angle_P,angle_Y,Dis);
        double abs_yaw = stm.yaw - angle_Y;
        double abs_pitch = stm.pitch + angle_P;
#endif

#ifdef SHOW_IMG
        //add thread mutex
        draw_target(target_armour.armour_rect,src);
        putText(src, to_string(target_armour.id),target_armour.armour_rect.boundingRect().tl(),FONT_HERSHEY_COMPLEX_SMALL,1,(255,0,0),1);
        putText(src, "cam_yaw "+to_string(send_data.yaw),Point2f(0,30),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
        putText(src, "cam_pitch "+to_string(send_data.pitch),Point2f(400,30),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
        putText(src, "ptz_yaw "+to_string(stm.yaw),Point2f(0,50),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
        putText(src, "ptz_pitch "+to_string(stm.pitch),Point2f(400,50),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
//            putText(src, "abs_yaw "+to_string(abs_yaw),Point2f(0,70),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
//            putText(src, "abs_pitch "+to_string(abs_pitch),Point2f(400,70),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0),1);
        putText(src,"world point "+ to_string(world_point.x)+" "+ to_string(world_point.y)+" "+ to_string(world_point.z),Point2f(0,90),FONT_HERSHEY_COMPLEX_SMALL,1,Scalar(255,0,0));
#ifdef USE_PRE
        circle(src,pixelPoint,20,Scalar(0,0,255),-1);
        circle(src,pre_pixel,20,Scalar(0,255,0),-1);
#endif

#endif
    } else {
        if(last_find_f){
            cmd = 0x31;
            send_data = last_send;
            last_find_f = 0;
        } else{
            send_data = {0, 0, 0};
            cmd = 0x30;
            losing_cnt++;
        }
    }
    auto t2 = chrono::steady_clock::now();
    if ((int) chrono::duration<double>(t2 - last_t).count() == 1) {
//        cout << " real frame " << frame_cnt << std::endl;
        frame_cnt = 0;
        last_t = t2;
    }
    *(signed char *) &port.buff_w_[0] = int16_t(10000 * (send_data.pitch));
    *(signed char *) &port.buff_w_[1] = int16_t((10000 * (send_data.pitch))) >> 8;
    *(signed char *) &port.buff_w_[2] = int16_t(10000 * (send_data.yaw));
    *(signed char *) &port.buff_w_[3] = int16_t((10000 * (send_data.yaw))) >> 8;
    *(signed char *) &port.buff_w_[4] = int16_t(100 * send_data.dis);
    *(signed char *) &port.buff_w_[5] = int16_t(100 * send_data.dis) >> 8;
    port.SendBuff(cmd, port.buff_w_, 6);
}

/**
 *
 */
#define K_SPEED 10000

[[noreturn]]void Armour::run() {
    if (!readCameraParameters(cameraParam)) {
        for (int i = 0; i < 100; ++i)
            std::cout << "read camera param fail ..." << std::endl;
    }
    int img_cnt = 0;
    auto start_time = chrono::steady_clock::now();
    STATE last_mode;
    std::chrono::time_point<std::chrono::steady_clock> last_time;
    while (true) {
        auto ts = chrono::steady_clock::now();
        std::chrono::time_point<std::chrono::steady_clock> t1;
        if(img_buf.second.empty()){ continue;}
        mImg_buf.lock();
//        if (img_buf.size() > 0) {
//            std::cout << "img buff " << img_buf.size() << std::endl;
//            t1 = img_buf.front().first;
//            time_stamp = chrono::duration<double>(t1 - start_time).count();//s
//            src = img_buf.front().second.clone();
//            img_buf.pop();
//        }
        t1 = img_buf.first;
        time_stamp = chrono::duration<double>(t1 - start_time).count();//s
        src = img_buf.second.clone();
        img_buf.second.release();
        mImg_buf.unlock();
        if (!src.data) {
            continue;
        }
#ifdef WRITE_IMG
        char key = cv::waitKey(30);
        if (key == 's') {
            img_cnt++;
            cv::imwrite(PROJECT_DIR"/data/" + to_string(img_cnt) + ".jpg", src);
            std::cout << "img idx =========================" << to_string(img_cnt) << std::endl;

        }
        imshow("picture", src);
        cv::waitKey(1);
        continue;
#endif
        uint8_t find_color_armour = 2;
        uint8_t find_color_energy = 1;
        if (port.receive[4] > K_SPEED) {
            find_color_armour = 2;
            find_color_energy = 1;
            std::cout << "our team is blue, shoot red!!" << std::endl;
        }
        if (port.receive[4] < K_SPEED) {
            find_color_armour = 1;
            find_color_energy = 2;
            std::cout << "our team is red, shoot blue!!" << std::endl;
        }
        STATE mode = getMode(port.receive[1]);
        switch (mode) {
            case STATE_BUFF:{
                std::cout<<"MODE BUFF "<<std::endl;
                if(mode != last_mode){
                    energy->finder->is_inited = false;
                }
                char cmd = 0x30;
                int pp = chrono::duration<double,milli> (t1-start_time).count();
                if(energy->run(src,pp,find_color_energy,port.receive[1],send_data.pitch,send_data.yaw,send_data.dis)){
                    cmd = 0x31;
                    energy->energy_last_flag = 1;
                    energy->energy_last_send = send_data;
                }else{
                    if(energy->energy_last_flag){
                        cmd = 0x31;
                        send_data = energy->energy_last_send;
                        energy->energy_last_flag = 0;
                    }else{
                        cmd = 0x30;
                        send_data = {0,0,0};
                    }
                }
                cv::waitKey(1);
                *(signed char *) &port.buff_w_[0] = int16_t(10000 * (send_data.pitch));
                *(signed char *) &port.buff_w_[1] = int16_t((10000 * (send_data.pitch))) >> 8;
                *(signed char *) &port.buff_w_[2] = int16_t(10000 * (send_data.yaw));
                *(signed char *) &port.buff_w_[3] = int16_t((10000 * (send_data.yaw))) >> 8;
                *(signed char *) &port.buff_w_[4] = int16_t(100 * send_data.dis);
                *(signed char *) &port.buff_w_[5] = int16_t(100 * send_data.dis) >> 8;
                port.SendBuff(cmd, port.buff_w_, 6);
                break;
            }
            default:
                /// tracking roi test
                if(expose_time_reset){
                    expose_time_reset = false;
                    expose_time_set = false;
                }
                fps_cnt++;
                if (fps_cnt > 10) {
                    fps_cnt = losing_cnt = find_cnt = 0;
                }
                if (find_cnt > 5) {
                    tracking_flag = 1;
                    losing_cnt = 0;
                }
                if (tracking_flag) { tracking_cnt++; }
                if (losing_cnt > 5) {
                    tracking_flag = tracking_cnt =roi_ready = find_cnt = 0;
                    armourROI = Rect(0, 0, 0, 0);
                }
                cv::Mat dst;
                if (roi_ready) {
                    roi = src(armourROI);
                    imgPretreatment(roi, dst, find_color_armour);
                } else
                    imgPretreatment(src, dst, find_color_armour);
                if (!armourDetector(dst))
                    std::cout << "<< ===== NO LIGHT ARMOUR FIND !!!! ===== >>" << std::endl;
                if (!armourSort())
                    std::cout << "<< ===== NO ID ARMOUR FIND !!!! ===== >>" << std::endl;
                armour_tracking();
                std::cout << "<< ===== AUTO AIM RUNNING !!!! ===== >>" << std::endl;
                std::cout << "<< ===== ARMOUR ROI STATUS " << (int) roi_ready << std::endl;
        }
        auto te = chrono::steady_clock::now();
        double cost = chrono::duration<double, milli>(te - ts).count();
        std::cout << " == ang_P " << send_data.pitch
                  << " == ang_Y " << send_data.yaw
                  << " == Dis " << send_data.dis
                  << " == Bs " <<stm.bulletSpeed
                  << std::endl;
        std::cout << "cost " << cost << " ms" << std::endl;
        last_mode = mode;
        double period = chrono::duration<double> (t1-last_time).count();
        double fps = 1.0 / period;
        last_time = t1;
        std::cout<<"================ FPS: "<<fps<<std::endl;
//        putText(src, to_string(fps),Point2f(0,30),FONT_HERSHEY_COMPLEX_SMALL,1, Scalar(255,0,0));
//        imshow("src",src);
//        cv::waitKey(1);
    }
}