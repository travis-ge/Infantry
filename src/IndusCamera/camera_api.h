#ifndef CAMERA_API_H
#define CAMERA_API_H

#include <stdio.h>
#include "DxImageProc.h"
#include "GxIAPI.h"
//include cv library
#include "opencv2/opencv.hpp"

//控制相机相关参数结构体
struct camera_config {
    //device sn
    std::string sn_str;
    char *SN;
    //设置ROI的宽度
    int nWidth = 960;//960
    //设置ROI的高度
    int nHeight = 768;//768
    //设置ROI的X方向偏移
    int nOffsetX = 160;
    //设置ROI的Y方向偏移
    int nOffsetY = 128;
    //是否开启触发模式
    int TriggerMode = 0;
    //设置触发源
    int TriggerSource = 1;
    //设置上升沿还是下降沿触发
    int TriggerActivation = 1;
    //设置是否自动曝光
    int ExposureAuto = 0;
    //设置固定曝光时间(单位:um)
    double Exposure = 4000.0;//2000
    //设置自动曝光时间下限(单位:um)
    double MinExposure = 100.0;
    //设置自动曝光时间上限(单位:um)
    double MaxExposure = 50000.0;
    //是否进行自动白平衡
    int BalanceAuto = 1;
    //选择白平衡通道
    int BalanceSelector = 1;
    //设置白平衡系数
    double BalanceRatio = 1;
    //是否进行自动增益
    int GainAuto = 1;
    //选择增益通道
    int GainSelector = 0;
    //设置增益值
    double Gain = 15.0;
};


//@brief Camera configuration
GX_STATUS Config();


/**
* @brief Industrial Mercure Camera class, product of the camera factory inherited from CameraBase
*/
class MercureDriver {
public:
    /**
     * @brief Constructor of MercureDriver
     * @param camera_info  Information and parameters of camera
     */
    MercureDriver(camera_config camera_info);

    /**
     * @brief Start to read Mercure camera
     * @param img Image data in form of cv::Mat to be read
     */
    void InitCamera();

    /**
     * @brief Stop to read Mercure camera
     */
    void StopCamera();


    static MercureDriver *pthis;

    ~MercureDriver();

    GX_STATUS status;
    GX_DEV_HANDLE hDevice_;
private:
    //! device handle
    GX_OPEN_PARAM stOpenParam_;
    const camera_config camera_info_;

};

#endif //CAMERA_API_H
