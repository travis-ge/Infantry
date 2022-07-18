#include "ArmourFinder.h"
#include "common.h"
#include "serial_port.h"
#include "SolveSine.cpp"
#include "particle_filter.h"

using namespace std;
using namespace cv;

extern SerialPort port;
SolvePara solveFan;
double SolvePara::params_fitting[4]={1,1.9,0,1};
double SolvePara::PHI[1]={0};

ParticleFilter pf;
ParticleFilter pf_param_loader;
ArmourFinder::ArmourFinder() {
    FileStorage fs(Param, FileStorage::READ);
    fs["energy"]["red_spilt_threshold"] >> r_spilt_threshold;
    fs["energy"]["red_gray_threshold"] >> r_gray_threshold;
    fs["energy"]["blue_spilt_threshold"] >> b_spilt_threshold;
    fs["energy"]["blue_gray_threshold"] >> b_gray_threshold;
    fs["energy"]["red_green_threshold"] >> r_green_threshold;
    fs["energy"]["blue_green_threshold"] >> b_green_threshold;
    fs["energy"]["fan_size_threshold"] >> fan_size_threshold;
    fs["energy"]["debug_no_img"] >> debug_no_img;
    fs["energy"]["debug"] >> debug;
    fs.release();

    YAML::Node config = YAML::LoadFile(pf_path);
    pf_param_loader.initParam(config, "buff");
}
ArmourFinder::~ArmourFinder() {

}
/*
    mode 0:静止 1:小符 2:拟合正弦击打大符 3:过去几帧平均速度预测
*/
int ArmourFinder::searchArmour(Mat &src, long long timestamp, double p_time, const uint8_t find_color_energy,
                                const uint8_t mode) {

    ///任务周期初始化///
    /// change  notion!!!!! init
    if (!is_inited) { taskInit(); }
    ///识别部分///
    Init(timestamp, p_time);
    //预处理
    binImg(src, dst, find_color_energy);
    //初步提取轮廓
    if (!findFans(dst)) { return -1; }
    //寻找装甲板
    if (!findtagpoint(src, find_color_energy)) { return -1; }
    //寻找R所在区域
    if (!getROIofR()) { return -1; }
    // 寻找R标签 TODO：识别圆形换成R标签
//    if (!findRinROI(src)) { return; }
    if (!findRinROI(dst)) { return -1; }

    ///信息提取///
    //获取极坐标
    getTargetPolarAngle();
    //获取信息
    getInfo();
    //处理信息
    if ((!is_find_dir) && (mode)) {
        getDirection();
    }    //计算方向

    cout<<"-----------------"<<endl;
    cout<<rotation_direction<<endl;
    if(debug){
        std::cout<<"debug infor fan dir is "<<rotation_direction<<std::endl;
    }
    //角度预测
    if (predict_time == 0) {
        pre_point = tag_point;
        return 2;
    }

    pre_angle = predictAngle(mode);
    clog<<"pre_angle\n"<<pre_angle<<endl;
    //像素层面计算坐标
    fan_.pre_point = pre_point;
    fan_.circle_center = circle_center_point;
    fan_.tag_point = tag_point;
    fan_.armour_rect = min_tagbox;
    fan_.pre_angle = pre_angle;
    memcpy(fan_.armour_points, aim_box, 4 * sizeof (cv::Point2f));
    angle2points_PIX();
    return 1;
}

void ArmourFinder::taskInit() {
    //初始化一些变量
    last_tag_point.x = 0;
    last_tag_point.y = 0;

    target_polar_angle = 0;
    last_polar_angle = 0;

    is_find_dir = false;
    rotation_direction = -2;
    last_rotation_direction = -2;
    is_sine_solved = false;

    v_angle.clear();

    predict_time = 0;

    pf.initParam(pf_param_loader);
    solveFan.is_sine_found = false;
    solveFan.is_phi_start = false;
    solveFan.is_phi_found = false;

    is_inited = true;
}

void ArmourFinder::Init(long long timestamp, double p_time) {
    fan_contours.clear();
    fan_hierarchy.clear();
    tag_outline_con.clear();

    //TODO:加入预测时间变量初始化
    predict_time = p_time;
    time_stamp = timestamp;
//    std::clog<<"internal "<<time_stamp<<endl;

    tag_point.x = 0;
    tag_point.y = 0;
    pre_point.x = pre_point.y = 0;
    circle_center_point.x = circle_center_point.y = 0;
    for (int i = 0; i < 4; ++i) {
        aim_box[i].x = aim_box[i].y = 0;
    }
}

void ArmourFinder::binImg(const Mat &src, Mat &dst, const int dealColor) {
    if(debug){
        namedWindow("energy", CV_WINDOW_AUTOSIZE);
    }
    Mat gray_bin;
    Mat green;
    cv::split(src, src2bgr);
    cvtColor(src, gray_bin, CV_BGR2GRAY);
    if (dealColor == 1) { //  BLUE
        if(debug){
            cvCreateTrackbar("b_spilt_threshold", "energy", &b_spilt_threshold, 255);
            cvCreateTrackbar("b_gray_threshold", "energy", &b_gray_threshold, 255);
            cvCreateTrackbar("b_green_threshold", "energy", &b_green_threshold, 255);
        }
        cv::subtract(src2bgr[0], src2bgr[2], dst);
        cv::subtract(src2bgr[0], src2bgr[1], green);
        threshold(dst, dst, b_spilt_threshold, 255, THRESH_BINARY);
        threshold(gray_bin, gray_bin, b_gray_threshold, 255, THRESH_BINARY);
        threshold(green, green, b_green_threshold, 255, THRESH_BINARY);
    } else if (dealColor == 2) {    //RED
        if(debug){
            cvCreateTrackbar("r_spilt_threshold", "energy", &r_spilt_threshold, 255);
            cvCreateTrackbar("r_gray_threshold", "energy", &r_gray_threshold, 255);
            cvCreateTrackbar("r_green_threshold", "energy", &r_green_threshold, 255);
        }
        cv::subtract(src2bgr[2], src2bgr[0], dst);
        cv::subtract(src2bgr[2], src2bgr[1], green);
        threshold(dst, dst, r_spilt_threshold, 255, THRESH_BINARY);
        threshold(gray_bin, gray_bin, r_gray_threshold, 255, THRESH_BINARY);
        threshold(green, green, r_green_threshold, 255, THRESH_BINARY);
    }
    //dilate(gray_bin,gray_bin,element3);
    dilate(dst,dst, element5);
    dilate(green,green, element5);
    dilate(green,green, element7);
    //dilate(green,green, element7);
    if(debug){
        imshow("gray", gray_bin);
        imshow("spilt", dst);
        imshow("green", green);
    }
//    dst = dst & gray_bin & green;
    dst = dst & gray_bin ;
//    dst = gray_bin;
    dilate(dst,dst, element5);
   // erode(dst,dst,element5);
    if(debug){
//        imshow("gray", gray_bin);
        imshow("dst", dst);
    }
}

void ArmourFinder::binROI(const Mat &src, Mat &dst, const int team) {
    std::vector<cv::Mat> src_split_;
    cv::split(src, src_split_);
    if (src.empty()) { return; }
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);

    if (team == 2) {//red
        subtract(src_split_[2], src_split_[0], src_separation);
        subtract(src_split_[2], src_split_[1], src_green);

        threshold(src_separation, src_separation, r_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, r_gray_threshold, 255, cv::THRESH_BINARY);
        threshold(src_green, src_green, r_green_threshold, 255, cv::THRESH_BINARY);
        dilate(src_gray,src_gray,element3);
        dilate(src_separation, src_separation, element5);
        dilate(src_green,src_green, element5);
        dilate(src_green,src_green, element7);
//        dst = src_separation & src_gray & src_green;
        dst = src_separation & src_gray;
        //dilate(dst,dst,element3);
    } else {//blue
        subtract(src_split_[0], src_split_[2], src_separation);
        subtract(src_split_[0], src_split_[1], src_green);
        threshold(src_separation, src_separation, b_spilt_threshold, 255, cv::THRESH_BINARY);
        threshold(src_gray, src_gray, b_gray_threshold, 255, cv::THRESH_BINARY);
        threshold(src_green,src_green,r_green_threshold,255,cv::THRESH_BINARY);

        dilate(src_separation, src_separation, element5);
        dilate(src_green,src_green, element5);
        dilate(src_green,src_green, element7);
//        dst = src_separation & src_gray & src_green;
        dst = src_separation & src_gray;
    }


//    imshow("separation", src_separation);
////    imshow("src_green", src_green);
//    imshow("gray", src_gray);
//     imshow("dst", dst);
}

bool ArmourFinder::findFans(const cv::Mat &src) {
    fan_hierarchy.clear();
    fan_contours.clear();
    findContours(src, fan_contours, fan_hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));
    //findContours(src, fan_contours, fan_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    int sz = fan_contours.size();
    int contour[20] = {0};
    for(int i = 0; i < sz; i++){
        if(fan_hierarchy[i][3]>20){break;}
        if(fan_hierarchy[i][3] != -1){
            contour[fan_hierarchy[i][3]]++;
        }
    }
    //对于外轮廓进行初步筛选
    for (int i = 0; i < sz; i++) {

        if (fan_contours[i].size() < 6) { continue; }
        if(contour[i] != 1){ continue;}
        RotatedRect box = minAreaRect(fan_contours[i]);
        float r_area = box.size.height * box.size.width;
        if (debug || debug_no_img) {
            std::cout<<"//-----------------find fan information-----------------//"<<std::endl;
            std::cout << "area is " << r_area << std::endl;
        }
        if (r_area < fan_size_threshold) { continue; }

        ///通过长宽比筛选掉不合适的区域
        float height = box.size.height;
        float width = box.size.width;
        if (box.size.height < box.size.width) { swap(height, width); }
        float peri_rate = height / width;//筛掉长宽比不合适的轮廓
        if (debug || debug_no_img) {
            cout << "HW: " << peri_rate << '\t' << endl;
        }
        if (peri_rate > 2.7 || peri_rate < 0.9) { continue; }

        ///根据轮廓对于扇叶的面积比判断 待击打扇叶 0.5附近 已击打0.8以上
        double c_area = contourArea(fan_contours[i]);

        double area_rate = c_area / r_area;
        if (debug || debug_no_img) {
            cout << "area_rate: " << area_rate << '\t' << endl;
        }
        if ((area_rate <= 0.6) && (area_rate > 0.40)) {
//            if(c_area>6000){continue;}
            Rect tmp = box.boundingRect();
            cv::Point tl = tmp.tl();
            // 限制 ROI 出界条件
            if (tl.x < 0) { tl.x = 0; }
            if (tl.y < 0) { tl.y = 0; }
            if(tl.x > src.cols){tl.x = src.cols;}
            if(tl.y > src.rows){tl.y = src.rows;}
//            if (tl.x + tmp.width > src.cols) { tl.x -= tl.x + tmp.width - src.cols; }
//            if (tl.y + tmp.height > src.rows) { tl.y -= tl.y + tmp.height - src.rows; }

            if (tl.x + tmp.width > src.cols) { tl.x = src.cols - tmp.width; }
            if (tl.y + tmp.height > src.rows) { tl.y = src.rows - tmp.height; }

            tag_outline_con.emplace_back(tl.x, tl.y, tmp.width, tmp.height);

            tag_fan.center = box.center;

        } else if ((area_rate > 0.55) && (area_rate < 1)) {
            ///TODO 根据已经击打的扇叶计数，判断是否是同一轮
            continue;
        } else {
            continue;
        }

        if (0) {
            Mat drawing = Mat::zeros(src.size(), CV_8UC3);
            Scalar color = Scalar(0, 255, 255);
            Point2f rect[4];         //存储最小矩阵顶点数值，画点用

            drawContours(drawing, fan_contours, i, color, 2, 8, fan_hierarchy, 0, Point());

            box.points(rect);//该类接口不允许直接访问顶点，把最小外接矩形四个端点复制给rect数
            for (int j = 0; j < 4; j++) {
                line(drawing, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 2, 8);
            }

            cv::circle(drawing, Point(tag_point.x, tag_point.y), 3, Scalar(0, 0, 255), -1, 8);  //绘制最小外接矩形的中心点
//            cv::circle(drawing, Point (480, 384), 3, Scalar(255, 0, 255), -1, 8);  //绘制最小外接矩形的中心点
            tag_point.x = 0;
            tag_point.y = 0;
            imshow("轮廓图", drawing);
            waitKey(1);
        }

        if (tag_outline_con.empty()) { return false; }
        else { return true; }
    }
    return false;
}

bool ArmourFinder::findtagpoint(Mat &dst, const int team) {
    tagROI = dst(tag_outline_con[0]);

    //imshow("rroi",tagROI);
    armor_contours.clear();
    armor_hierarchy.clear();
    armor_contours_external.clear();

    ////预处理部分
    if (tagROI.type() == CV_8UC3) {
        binROI(tagROI, tagROI, team);
    }

    //第一步：分别筛选内外轮廓和外轮廓
//    findContours(tagROI,armor_contours,armor_hierarchy,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);
//    findContours(tagROI,armor_contours_external,armor_hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

    findContours(tagROI, armor_contours, armor_hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    findContours(tagROI, armor_contours_external, armor_hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int ext_cnt = armor_contours_external.size();
    int con_cnt = armor_contours_external.size();

    //第二步：
    //上交19年代码的思路
    //装甲板识别出内外两层
    for (int i = 0; i < armor_contours_external.size(); i++)//遍历所有外轮廓
    {
        unsigned long external_contour_size = armor_contours_external[i].size();//记录下外轮廓的尺寸
        for (int j = 0; j < armor_contours.size(); j++) {   //查找所有轮廓
            unsigned long all_size = armor_contours[j].size();  //遍历所有内轮廓
            if (external_contour_size == all_size) {            //如果内外尺寸相同，说明是同一个轮廓
                swap(armor_contours[j], armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//删除最后一个元素
                break;
            }
        }
    }


    ///提取待击打扇叶相关信息，四个顶点坐标，中心点坐标，倾斜程度等信息
    //其实只剩下一个装甲板
    for (int i = 0; i < armor_contours.size(); ++i) {

        //最终得到理想装甲板矩阵所在轮廓
        min_tagbox = minAreaRect(armor_contours[i]);
        double area = min_tagbox.size.height * min_tagbox.size.width;

        double tmp_w = min_tagbox.size.width;
        double tmp_h = min_tagbox.size.height;
        if(tmp_h > tmp_w){
            double tmp = tmp_w;
            tmp_w = tmp_h;
            tmp_h = tmp;
        }
        float wh_rate = tmp_w / tmp_h;
        if(debug || debug_no_img){
            std::cout<<"energy armour size "<<area<<std::endl;
            std::cout<<"energy armour wh rate "<<wh_rate<<std::endl;
            std::cout<<"^-----------------find fan information  End-----------------^"<<std::endl;
        }
        if(area<300){ continue;}
        if(wh_rate<1||wh_rate>3){ continue;}
//        std::cout<<"______________________"<<endl;
//        std::cout<<tag_point.z<<endl;
//        std::cout<<"______________________"<<endl;

        //使用小装甲板中心坐标加上在矩形框中偏移量得到在原图中像素坐标
        Point2f off_set(tag_outline_con[0].x, tag_outline_con[0].y);
//        tag_point.x = tag_outline_con[0].x+min_tagbox.center.x;
//        tag_point.y = tag_outline_con[0].y+min_tagbox.center.y;

        min_tagbox.points(aim_box);
        tag_point = off_set + min_tagbox.center;
        aim_box[0] = off_set + aim_box[0];
        aim_box[1] = off_set + aim_box[1];
        aim_box[2] = off_set + aim_box[2];
        aim_box[3] = off_set + aim_box[3];
//        std::cout<<"______________________"<<endl;
//        std::cout<<"mid_x:"<<middle_point.x<<endl;
//        std::cout<<"mid_y:"<<middle_point.y<<endl;
        return true;
    }
    return false;
}

bool ArmourFinder::getROIofR() {
    float k1 = 1.7;

    float length = min_tagbox.size.height > min_tagbox.size.width ?
                   min_tagbox.size.height : min_tagbox.size.width;
    float height = length * k1;
//    Point2f p2p (tag_fan.center.x-tag_point.x-length/3,
//                 tag_fan.center.y-tag_point.y-length/3);

//    Point2f p2p (tag_fan.center.x-tag_point.x-length/3,
//                 tag_fan.center.y-tag_point.y-length/3);

    Point2f p2p(tag_fan.center.x - tag_point.x,
                tag_fan.center.y - tag_point.y);

    p2p = p2p / tool::getDistance(tag_fan.center, tag_point);

    cv::Point2f R = cv::Point2f(tag_fan.center + p2p * length * 2.5);
    cv::Point2f R_tl(R.x - height / 2, R.y - height / 2);

    if (R_tl.x < 0) { R_tl.x = 0; }
    if (R_tl.y < 0) { R_tl.y = 0; }
    if (R.x + height > 958) { R.x = 958 - height; }
    if (R.y + height > 766) { R.x = 766 - height; }
    center_ROI = Rect(R_tl, Size2f(height, height));
    return true;
}

bool ArmourFinder::findRinROI(Mat &src) {
    if (src.empty())return false;

    Mat ROI = src(center_ROI);
    if (src.type() == CV_8UC3) {
        cvtColor(ROI, ROI, COLOR_BGR2GRAY);
        threshold(ROI, ROI, 150, 255, THRESH_OTSU);
    }
//    dilate(ROI, ROI, element3);
    erode(ROI, ROI, element3);
//    erode(ROI, ROI, element3);
    erode(ROI, ROI, element3);

    std::vector<vector<Point> > center_R_contours;
    findContours(ROI, center_R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

//     imshow("ROI", ROI);
//     waitKey(1);

    for (int i = 0; i != center_R_contours.size(); i++) {
        RotatedRect box = minAreaRect(center_R_contours[i]);
        double c_area = contourArea(center_R_contours[i]);

//        if(r_area<240){continue;}
//        cout<<r_area<<endl;

        if (c_area < 100 || c_area > 450) { continue; }
//        cout<<"c_area"<<c_area<<endl;

        ///通过长宽比筛选掉不合适的区域
        int height = box.size.height;
        int width = box.size.width;
        if (box.size.height < box.size.width) { swap(height, width); }
        float peri_rate = height / width;//筛掉长宽比不合适的轮廓
//        cout<<"peri_rate"<<peri_rate<<endl;
        if (peri_rate > 1.2) { continue; }

        Point2f center_tmp;
        float radius_tmp;
        minEnclosingCircle(center_R_contours[i], center_tmp, radius_tmp);
//        if(radius_tmp<5){continue;}
        double cir_area = tool::getCircleArea(radius_tmp);
//        cout<<"c_area/cir_area"<<c_area/cir_area<<endl;
        if (c_area / cir_area < 0.5) { continue; }

        centerR = cv::minAreaRect(center_R_contours[i]);
//        RotatedRect rotated_tmp = RotatedRect(Point(0, 0), center_ROI.size(), -90);

        if (tool::getDistance(center_tmp, centerR.center) > 5) { continue; }

        Point2f tmp = center_ROI.tl();
        circle_center_point = tmp + centerR.center;
//        circle_center_point = tmp + center_tmp;

//        float target_length =
//                min_tagbox.size.height > min_tagbox.size.width ? min_tagbox.size.height : min_tagbox.size.width;
//        circle_center_point.y += target_length / 7.5;//实际最小二乘得到的中心在R的下方
        return true;
    }
    return false;
}

void ArmourFinder::getTargetPolarAngle() {
    target_polar_angle = tool::Arctan(tag_point.y - circle_center_point.y ,tag_point.x - circle_center_point.x);

    float  angle = tool::rad2deg(target_polar_angle);
//    angle_rad_2pi_write<<"angle"<<target_polar_angle;
//    angle_deg_write<<"angle"<< angle;

}

const int rows = 500;
const int cols = 1000;
Mat plot_all =Mat::zeros(rows,cols, CV_8UC3);
int plot_cnt = 0;

void ArmourFinder::getInfo() {

    ////每帧信息
    //帧间角度差，判断扇叶是否发生切换
    float polar_angle_diff = last_polar_angle - target_polar_angle;
    float abs_angle = fabs(polar_angle_diff);

    if (abs_angle > 0.9 && abs_angle < 6.0) {
//        is_target_changed = true;
        target_changed_delay = stride;
        v_angle.emplace_back(time_stamp, target_polar_angle);
    }  else{
        v_angle.emplace_back(time_stamp, target_polar_angle);
    }

    if (v_angle.size() > sample_size) {v_angle.erase(v_angle.begin());}

    ///历史信息处理
    if (v_angle.size() > 1 + stride) {
        int idx = v_angle.size() - 1;
        int idx_before = idx - stride;
        float angleDifference = 0;
        if (target_changed_delay) {
            angleDifference =
                    fmod(abs(v_angle[idx].y - getChangeAngle(polar_angle_diff) - v_angle[idx_before].y)+ CV_2PI, CV_2PI);
//                    fmod(abs(v_angle[idx].y - getChangeAngle(polar_angle_diff) - v_angle[idx_before].y)+ CV_2PI, CV_2PI);
//            (abs(v_angle[idx].y - getChangeAngle(polar_angle_diff) - v_angle[idx_before].y));
        } else{
            angleDifference = fmod(abs(v_angle[idx].y - v_angle[idx_before].y)+ CV_2PI, CV_2PI);
//            angleDifference = abs(v_angle[idx].y - v_angle[idx_before].y);
//          float angleDifference_2 =(v_angle[idx].y - v_angle[idx_before].y);
        }

        fit_diff.emplace_back(time_stamp, angleDifference);

        if (fit_diff.size() > sample_size) {
            fit_diff.erase(fit_diff.begin());
        }

        if(fit_diff.size()>0){

            double span = double(time_stamp - fit_diff[fit_diff.size()-2].x)/1000;  //s
            //时间间隔限制
            if(span >0){
//                cout<<"---------------------------------------------\n";
//                cout<<"----------------------span-------------------\n"<<span<<endl;
//                cout<<"---------------------------------------------\n";
                float cur_spd =
                        fmod( abs(angleDifference) + CV_2PI,  CV_2PI)/span;

                float  cur_spd_2 = cur_spd;
                //不要小瞧沈航人.jpg （沈航RMer确实很强，这句话是智能车竞赛一个沈航老师有趣的梗）
                auto is_ready = pf.is_ready;
                Eigen::VectorXd measure(1);
                measure<<cur_spd;
                pf.update(measure);
                if (is_ready){
                    auto predict = pf.predict();
                    cur_spd = predict[0];
                }

                fit_speed.emplace_back(time_stamp, cur_spd);

                if (fit_speed.size() > sample_size) {
                    fit_speed.erase(fit_speed.begin());
                }

//                ////记录下每帧
//                cv::circle(plot_all,Point2f(fmod(plot_cnt,cols), rows-(cur_spd/5*rows)),2, Scalar(0,255,0));
//                plot_cnt++;
//                if(fmod(plot_cnt,cols) == 0){plot_all =Mat::zeros(rows,cols, CV_8UC3);}
//                imshow("plot_1",plot_all);
//                waitKey(1);


            }

        }
    }

    //更新
    last_polar_angle = target_polar_angle;
    if (target_changed_delay)target_changed_delay--;
}

float ArmourFinder::getChangeAngle(float deltaAngle) {
    int minIndex = 1;
    float minError = 100.0f;
    float delta_angle = fabs(deltaAngle);
    for (int i = 1; i <= 5; i++) {
        float error = fabs(delta_angle - CV_2PI * i / 5);
        if (error < minError) {
            minError = error;
            minIndex = i;
        }
    }

    if (deltaAngle < 0)
        return CV_2PI * minIndex / 5;
    else
        return -CV_2PI * minIndex / 5;

}

void ArmourFinder::getDirection() {

//    cout<<"-----------------"<<endl;
//    cout<<rotation_direction<<endl;

//    cout<<last_rotation_direction<<endl;
//    cout<<"-----------------"<<endl;

    if (v_angle.size() >= 30) {
        int stop = 0, clockwise = 0, counter_clockwise = 0;
        for (size_t i = 0; i < 25; i++) {
//            float angle_diff = tool::rad2deg(v_angle[i + 15].y) - tool::rad2deg(v_angle[i].y);

            float angle_1 = tool::rad2deg(v_angle[i + 3].y);
            float angle_2 = tool::rad2deg(v_angle[i].y);
            float angle_diff = angle_1 - angle_2;
            if (fabs(angle_diff) < 1.5)
                stop++;
            else if (angle_diff > 0)
                counter_clockwise++;
            else
                clockwise++;
        }
//        v_angle.clear();
        if (stop > counter_clockwise && stop > clockwise) {
            rotation_direction = 0;
        } else if (clockwise > counter_clockwise) {
            rotation_direction = 1;
            is_find_dir = true;
        } else {
            rotation_direction = -1;
            is_find_dir = true;
        }

//        //判断两次，两次值相同说明稳定，不再进行判断
//        if (last_rotation_direction == rotation_direction) { is_find_dir = true; }
//        last_rotation_direction = rotation_direction;

    }
}

float ArmourFinder::predictAngle(uint8_t mode) {
    if (mode == 0) { return 0; }
    else if (mode == 1) {       //小符
            return  rotation_direction * tool::deg2rad(60 * predict_time);

    } else if (mode == 2) {     //大符

        //拟合
        solveFan.solvePara(fit_speed);
        if(!solveFan.is_sine_found||!solveFan.is_phi_found){return 0;}
        //预测
        double time_passed=((float)(time_stamp-solveFan.start_time))/1000;
//        return  rotation_direction * solveFan.pos_fun(predict_time+time_passed) - solveFan.pos_fun(time_passed);
        return  rotation_direction * (solveFan.pos_fun(predict_time+time_passed) - solveFan.pos_fun(time_passed));
    }
}

void ArmourFinder::angle2points_PIX() {

    float d_x = tag_point.x - circle_center_point.x;
    float d_y = tag_point.y - circle_center_point.y;
    pre_point.x = d_x * cos(pre_angle) - d_y * sin(pre_angle) + circle_center_point.x;
    pre_point.y = d_x * sin(pre_angle) + d_y * cos(pre_angle) + circle_center_point.y;
}
