#include <vector>
#include <string>
#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include "armour.h"
#include "serial_port.h"
#include "common.h"
#include "base.h"

std::chrono::time_point<std::chrono::steady_clock> start_time;
SerialPort port;
Ptz_infor stm;

Armour armour;
[[noreturn]] void armour_detect(){
    armour.armour_detect();
}
[[noreturn]] void armour_sort(){
    armour.armourSort();
}
[[noreturn]] void armour_tracking(){
    armour.armour_tracking();
}
int main() {

    start_time = chrono::steady_clock::now();
    GX_STATUS status = Config();
    if (status != GX_STATUS_SUCCESS) {
        std::cout << "config Camera Faile ..." << std::endl;
        return 0;
    }
    camera_config cam0_info;
    cam0_info.sn_str = "KE0200120157";
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
    while (!port.PortInit(0, 115200));
    std::thread serial_receive_thread(port_receive);
    std::thread armour_detect_thread(armour_detect);
    std::thread armour_sort_thread(armour_sort);
    std::thread armour_tracking_tread(armour_tracking);
    serial_receive_thread.join();
    armour_detect_thread.join();
    armour_sort_thread.join();
    armour_tracking_tread.join();


    return 0;
}
