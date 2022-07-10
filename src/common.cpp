//
// Created by Quanyi on 2022/3/27.
//
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <queue>
#include <thread>
#include <mutex>
#include <unistd.h>
#include "camera_api.h"
#include "tic_toc.h"
#include "armour.h"
#include "common.h"
#include "tool_fun.h"
#define image_width 960
#define image_height 768
int cnt = 0;
char *pRGB24Buf_0 = new char[image_width * image_height * 3];
std::queue<pair<std::chrono::time_point<std::chrono::steady_clock>,cv::Mat>> img_buf;
std::mutex mImg_buf;

/**
 *
 * @param pFrame
 */
void Frame_0_ProcessRGB(GX_FRAME_CALLBACK_PARAM *pFrame) {
//    cnt++;
//    if(cnt > 10)
//        cnt = 0;
//    if(cnt !=1 )
//        return;
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
        auto t0=std::chrono::steady_clock::now();
        /* if(!time_flag_0)
         {
             end_0 = start_0;
             time_flag_0 = true;
         }
         else
         {
             std::chrono::duration<double> elapsed_seconds = start_0 - end_0;
             std::cout << "the CAM_0 frame frequency is: " << 1/elapsed_seconds.count() << std::endl;
             end_0 = start_0;
         }*/
        //TicToc tic;
        //        printf("Image Time Stamp %ld\n",pFrame->nTimestamp);
        //        printf("Image Width %d\n",pFrame->nWidth);
        //        printf("Image Height %d\n",pFrame->nHeight);
        //        printf("Image_0 Frame ID %ld \n",pFrame->nFrameID);
        /*char *pRGB24Buf = new char[pFrame->nWidth * pFrame->nHeight * 3];           //输出图像RGB数据
        if (pRGB24Buf == NULL)
            return  ;
        else
            memset(pRGB24Buf,0,pFrame->nWidth * pFrame->nHeight * 3 * sizeof(char));    //缓冲区初始化*/

        memset(pRGB24Buf_0, 0, pFrame->nWidth * pFrame->nHeight * 3 * sizeof(char));    //缓冲区初始化*/
        DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR;           //选择插值算法
        DX_PIXEL_COLOR_FILTER nBayerType = BAYERBG;                   //选择图像Bayer格式
        bool bFlip = false;
        VxInt32 DxStatus = DxRaw8toRGB24(const_cast<void *>(pFrame->pImgBuf), pRGB24Buf_0, pFrame->nWidth,
                                         pFrame->nHeight, cvtype, nBayerType, bFlip);
        if (DxStatus != DX_OK) {
            /*if (pRGB24Buf != NULL)
            {
                delete []pRGB24Buf;
                pRGB24Buf = NULL;
            }*/
            return;
        }
        cv::Mat image_rgb24(pFrame->nHeight, pFrame->nWidth, CV_8UC3);
        memcpy(image_rgb24.data, pRGB24Buf_0, pFrame->nHeight * pFrame->nWidth * 3);
        //cv::Mat image_bgr;
        //cv::cvtColor(image_rgb24,image_bgr,CV_RGB2BGR);
        mImg_buf.lock();
        if(img_buf.size()>5){
            img_buf.pop();
        }
        img_buf.push(make_pair(t0, image_rgb24));
        mImg_buf.unlock();
        //cv::imshow("img_show: ",image_rgb24);
        //cv::waitKey(1);
        /*if (pRGB24Buf != NULL)
        {
            delete []pRGB24Buf;
            pRGB24Buf = NULL;
        }*/
        //std::cout << "the time of read image_0 is: " << tic.toc() << std::endl;
    }
    return;
}
/**
 *
 * @param infor
 * @return
 */
STATE getMode(char infor){
    STATE mode;
    if(infor == 'b' || infor == 'a')
        mode = STATE_BUFF;
    else if (infor == 'h')
        mode = STATE_SENTRY;
    else if(infor == 'r')
        mode = STATE_AUTOAIM;
    else if (infor == 't')
        mode = STATE_TOP;
    else if (infor == 'v')
        mode = STATE_RECORD;
    else
        mode = STATE_AUTOAIM;

    return mode;
}
