//
// Created by ares on 22-6-4.
//

#include "shooter.h"
using namespace std;
Shooter::Shooter() {
    period_cnt = 0;
    single_cnt = 0;
}
Shooter::~Shooter() {

}
/**
 *
 * @param pitch
 * @param yaw
 * @param dis
 * @return
 */
bool Shooter::ifShoot(float pitch, float yaw, float dis) {
    double width_diff = yaw * dis;
    double height_diff = pitch * dis;
//    std::cout<<"--------------------shooter-------------"<<std::endl;
//    std::cout<<"    width_diff "<<width_diff<<std::endl;
//    std::cout<<"    height_diff"<<height_diff<<std::endl;
    uint8_t tmp_flg = 0;
    if( std::abs(width_diff)< 0.12 && std::abs(height_diff) < 0.06 ){
        tmp_flg = 1;
    }else{
        tmp_flg = 0;
    }
    return prepareShoot(tmp_flg);
}

bool Shooter::prepareShoot(uint8_t tmp_flag) {
    period_cnt++;
    if(period_cnt > 5){
        period_cnt = single_cnt = 0;
    }
    if(tmp_flag){
        single_cnt++;
    }
    if(single_cnt > 3)
        return true;
}