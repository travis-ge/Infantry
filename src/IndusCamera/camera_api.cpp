#include "camera_api.h"

GX_STATUS Config() {
    GX_STATUS status = GX_STATUS_SUCCESS;
    //在起始位置调用GXInitLib()进行初始化，申请资源
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
        printf("GxInit Fail... err code: %d\n", status);
        return status;
    } else {
        printf("GxInit Success...\n");

    }
    uint32_t nDeviceNum = 0;
    status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if (status == GX_STATUS_SUCCESS && nDeviceNum > 0) {
        GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        //获取所有设备的基础信息
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
        if (status == GX_STATUS_SUCCESS) {
            for (unsigned int i = 0; i < nDeviceNum; i++) {
                std::cout << "pBaseinfo[" << i << "]=" << pBaseinfo[i].szSN << std::endl;
            }
            if (pBaseinfo != NULL) {
                delete[]pBaseinfo;
                pBaseinfo = NULL;
            }
            return status;
        } else {
            std::cout << "Get Device Base Info Failure ..." << std::endl;
            if (pBaseinfo != NULL) {
                delete[]pBaseinfo;
                pBaseinfo = NULL;
            }
            return status;
        }
    } else {
        printf("Update Device List Failed...\n");
        return status;
    }
}

MercureDriver::MercureDriver(camera_config camera_info) : camera_info_(camera_info) {
    hDevice_ = NULL;
    stOpenParam_.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam_.openMode = GX_OPEN_SN;
    stOpenParam_.pszContent = camera_info.SN;
    status = GX_STATUS_SUCCESS;
}

void MercureDriver::InitCamera() {
    //    GX_STATUS status = GX_STATUS_SUCCESS;
    if (status != GX_STATUS_SUCCESS)
        return;
    status = GXOpenDevice(&stOpenParam_, &hDevice_);
    if (status == GX_STATUS_SUCCESS) {
        printf("Camera Open Success...\n");
        //设置图像尺寸
        status = GXSetInt(hDevice_, GX_INT_WIDTH, int64_t(camera_info_.nWidth));
        status = GXSetInt(hDevice_, GX_INT_HEIGHT, int64_t(camera_info_.nHeight));
        status = GXSetInt(hDevice_, GX_INT_OFFSET_X, int64_t(camera_info_.nOffsetX));
        status = GXSetInt(hDevice_, GX_INT_OFFSET_Y, int64_t(camera_info_.nOffsetY));
        //是否打开触发模式
        if (camera_info_.TriggerMode) {
            printf("GX_TRIGGER_MODE_ON\n");
            //开启触发模式
            status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
            //开启外部触发
            status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_SWITCH, GX_TRIGGER_SWITCH_ON);
            //选择触发源
            status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_SOURCE, int64_t(camera_info_.TriggerSource));
            //选择上升沿还是下降沿触发
            status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_ACTIVATION, int64_t(camera_info_.TriggerActivation));
        } else {
            //关闭触发模式
            status = GXSetEnum(hDevice_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
        }

        if (camera_info_.ExposureAuto) {
            //获取自动曝光上下限的范围
            GX_FLOAT_RANGE autoExposureMinRange;
            GX_FLOAT_RANGE autoExposureMaxRange;
            status = GXGetFloatRange(hDevice_,
                                     GX_FLOAT_AUTO_EXPOSURE_TIME_MIN,
                                     &autoExposureMinRange);
            status = GXGetFloatRange(hDevice_,
                                     GX_FLOAT_AUTO_EXPOSURE_TIME_MAX,
                                     &autoExposureMaxRange);
            printf("Auto Exposure Min Range: %lfus to %lfus\n",
                   autoExposureMinRange.dMin,
                   autoExposureMinRange.dMax);
            printf("Auto Exposure Max Range: %lfus to %lfus\n",
                   autoExposureMaxRange.dMin,
                   autoExposureMaxRange.dMax);
            status = GXSetFloat(hDevice_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, camera_info_.MinExposure);
            status = GXSetFloat(hDevice_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, camera_info_.MaxExposure);
            //设置自动曝光方式(单次or连续)
            status = GXSetEnum(hDevice_, GX_ENUM_EXPOSURE_AUTO, int64_t(camera_info_.ExposureAuto));
        } else {
            //关闭自动曝光
            status = GXSetEnum(hDevice_, GX_ENUM_EXPOSURE_AUTO, int64_t(camera_info_.ExposureAuto));
            //获取曝光调节范围
            GX_FLOAT_RANGE shutterRange;
            status = GXGetFloatRange(hDevice_, GX_FLOAT_EXPOSURE_TIME, &shutterRange);
            printf("Min Exposure Time %lf us\n", shutterRange.dMin);
            printf("Max Exposure Time %lf us\n", shutterRange.dMax);
            //设置曝光值
            status = GXSetFloat(hDevice_, GX_FLOAT_EXPOSURE_TIME, camera_info_.Exposure);
        }

        //是否进行自动白平衡
        if (camera_info_.BalanceAuto) {
            //设置自动白平衡光照环境
            status = GXSetEnum(hDevice_, GX_ENUM_AWB_LAMP_HOUSE, GX_AWB_LAMP_HOUSE_ADAPTIVE);
            //设置自动白平衡(连续or单次)
            status = GXSetEnum(hDevice_, GX_ENUM_BALANCE_WHITE_AUTO, int64_t(camera_info_.BalanceAuto));

        } else {
            //关闭自动白平衡
            status = GXSetEnum(hDevice_, GX_ENUM_BALANCE_WHITE_AUTO, int64_t(camera_info_.BalanceAuto));
            //选择白平衡通道
            status = GXSetEnum(hDevice_, GX_ENUM_BALANCE_RATIO_SELECTOR, int64_t(camera_info_.BalanceSelector));
            //获取白平衡调节范围
            GX_FLOAT_RANGE ratioRange;
            status = GXGetFloatRange(hDevice_, GX_FLOAT_BALANCE_RATIO, &ratioRange);
            printf("Min Balance White %lf\n", ratioRange.dMin);
            printf("Max Balance White %lf\n", ratioRange.dMax);

            //设置白平衡系数
            status = GXSetFloat(hDevice_, GX_FLOAT_BALANCE_RATIO, camera_info_.BalanceRatio);
            //设置自动白平衡感兴趣区域(设置成和ROI窗口一样)
            status = GXSetInt(hDevice_, GX_INT_AWBROI_WIDTH, int64_t(camera_info_.nWidth));
            status = GXSetInt(hDevice_, GX_INT_AWBROI_HEIGHT, int64_t(camera_info_.nHeight));
            status = GXSetInt(hDevice_, GX_INT_AWBROI_OFFSETX, int64_t(camera_info_.nOffsetX));
            status = GXSetInt(hDevice_, GX_INT_AWBROI_OFFSETY, int64_t(camera_info_.nOffsetY));
        }

        //是否进行自动增益
        if (camera_info_.GainAuto) {
            //设置自动增益(单次or连续)
            status = GXSetEnum(hDevice_, GX_ENUM_GAIN_AUTO, int64_t(camera_info_.GainAuto));
        } else {
            //关闭自动增益
            status = GXSetEnum(hDevice_, GX_ENUM_GAIN_AUTO, int64_t(camera_info_.GainAuto));
            //选择增益通道类型
            status = GXSetEnum(hDevice_, GX_ENUM_GAIN_SELECTOR, int64_t(camera_info_.GainSelector));
            printf("GX_ENUM_GAIN_SELECTOR %d\n", status);
            //获取增益调节范围
            GX_FLOAT_RANGE gainRange;
            status = GXGetFloatRange(hDevice_, GX_FLOAT_GAIN, &gainRange);
            printf("Min Gain %lf\n", gainRange.dMin);
            printf("Max Gain %lf\n", gainRange.dMax);
            //设置增益值
            status = GXSetFloat(hDevice_, GX_FLOAT_GAIN, camera_info_.Gain);
        }
        //操作设备：控制、采集
        //status = GXRegisterCaptureCallback(hDevice_, NULL, OnFrameCallbackFun);
        //发送开采命令
        //status = GXSendCommand(hDevice_, GX_COMMAND_ACQUISITION_START);
    }
}

void MercureDriver::StopCamera() {
    //    GX_STATUS status = GX_STATUS_SUCCESS;
    if (status != GX_STATUS_SUCCESS)
        return;
    //发送停采命令
    status = GXSendCommand(hDevice_, GX_COMMAND_ACQUISITION_STOP);
    //注销采集回调
    status = GXUnregisterCaptureCallback(hDevice_);
    //关闭设备
    status = GXCloseDevice(hDevice_);
    if (status == GX_STATUS_SUCCESS) {
        //            printf("Camera Close Success");
        printf("Camera Close Success...\n");
    }
    //在结束的时候调用GXCLoseLib()释放资源
    status = GXCloseLib();
    return;
}

MercureDriver::~MercureDriver() {
    StopCamera();
    printf("The camera has closed...\n");
}


