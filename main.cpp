/*********    NEFU-Ares2022-INFANTRY-CV     *********/
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include "armour.h"
#include "serial_port.h"
#include "common.h"

SerialPort port;
Armour armour;
Ptz_infor stm;
Send send_data;
bool expose_time_reset = false;
bool expose_time_set = true;
//#define USE_ID
int main() {
    GX_STATUS status = Config();
    if (status != GX_STATUS_SUCCESS) {
        std::cout << "config Camera Faile ..." << std::endl;
        return 0;
    }
    camera_config cam0_info;
    cam0_info.sn_str = cameraSN;
    cam0_info.SN = &cam0_info.sn_str[0];
    MercureDriver *cam0 = new MercureDriver(cam0_info);
    cam0->InitCamera();
    if (cam0->status != GX_STATUS_SUCCESS) {
        std::cout << "Initial Camera Faile ..." << std::endl;
        return 0;
    }
    status = GXRegisterCaptureCallback(cam0->hDevice_, NULL, Frame_0_ProcessRGB);
    status = GXSendCommand(cam0->hDevice_, GX_COMMAND_ACQUISITION_START);

    if (status != GX_STATUS_SUCCESS) {
        std::cout << "Cam0 Start Read Faile ..." << std::endl;
        return 0;
    }
    status = GXRegisterDeviceOfflineCallback(cam0->hDevice_,NULL,Camera_offline_deal,&cam0->hCB_);
    if(status != GX_STATUS_SUCCESS)
        std::cout<<"camera offline function inited failed ... "<<std::endl;
    while (!port.PortInit(0, 115200));
    std::thread serial_receive_thread(&SerialPort::port_receive, port);
    std::thread armour_auto_shoot(&Armour::run, armour);
    GX_STATUS line_status;
    while(1){
        uint32_t device_num = 0;
        line_status = GXUpdateDeviceList(&device_num,5);
        if(line_status != GX_STATUS_SUCCESS){
            std::cout<<" check camera num failed  "<<std::endl;
            continue;
        }
        if(!device_num){
            std::cout<<" warming:: camera has offline, application will restart !!"<<std::endl;
            cam0->StopCamera();
            delete []cam0;
            exit(-1);
        }
    }
#ifdef USE_ID
    while (1) {
        if ((port.receive[1] == 'a' || port.receive[1] == 'b') && (!expose_time_reset)) {
            std::cout<<"Industry expose time resetting..."<<std::endl;
            status = GXSetFloat(cam0->hDevice_, GX_FLOAT_EXPOSURE_TIME, 1000);
            if (status == GX_STATUS_SUCCESS)
                expose_time_reset = true;
        }
        if(!expose_time_set){
            std::cout<<"Industry expose time setting..."<<std::endl;
            status = GXSetFloat(cam0->hDevice_, GX_FLOAT_EXPOSURE_TIME, cam0_info.Exposure);
            if (status == GX_STATUS_SUCCESS)
                expose_time_set = true;
        }
    }
#endif
    serial_receive_thread.join();
    armour_auto_shoot.join();
    cam0->InitCamera();
    return 0;
}
