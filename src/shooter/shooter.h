//
// Created by ares on 22-6-4.
//

#ifndef ARMOURSHOOT_SHOOTER_H
#define ARMOURSHOOT_SHOOTER_H
#include <iostream>

class Shooter{
public:
    Shooter();
    ~Shooter();

    bool prepareShoot(uint8_t tmp_flag);
    bool ifShoot(float pitch, float yaw, float dis);

private:
    int period_cnt;
    int single_cnt;

    int armour_width = 0.06;
    int armour_height = 0.06;

};
#endif //ARMOURSHOOT_SHOOTER_H
