//
// Created by quanyi on 22-5-18.
//

#include "Record.h"
#include "time.h"

Record::Record() {
    setParam();
}
Record::~Record() {
    delete videoWriter;
}

bool Record::setParam() {
    time_t tt;
    time(&tt);
    tt = tt + 8*3600;
    tm* t = gmtime(&tt);
    std::string filename  = std::to_string(t->tm_mon+1) +"-"+
                            std::to_string(t->tm_mday)+"-"+
                            std::to_string(t->tm_hour)+
                            std::to_string(t->tm_min)+
                            std::to_string(t->tm_sec);
    //std::cout<<"t "<<filename<<std::endl;
    path = "/home/aididiz/video/"+filename + ".avi";
    //std::cout<<"path is "<<path<<std::endl;
    fps = 20;
    size = cv::Size(960,768);
    is_inited = false;
    videoWriter = new cv::VideoWriter();
    record_cnt = 0;
}
bool Record::writeVideo(cv::Mat src) {
    record_cnt++;
    if(!is_inited){
        videoWriter->open(path, CV_FOURCC('M','J','P','G'), fps, size, true);
        is_inited = true;
    }

    videoWriter->write(src);
    cv::waitKey(1);
}