//
// Created by ares on 22-6-14.
//

#ifndef ARMOURSHOOT_DATA_TPYE_H
#define ARMOURSHOOT_DATA_TPYE_H

#include <opencv2/core/core.hpp>

struct time_angle{
    long long x;
    double y;
    time_angle( long long t, double a): x(t), y(a) {;}
};

#endif //ARMOURSHOOT_DATA_TPYE_H
