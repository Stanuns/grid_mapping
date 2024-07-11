//
// Created by wei.sun on 19-6-21.
//

#ifndef SRC_RANGE_READING_HPP
#define SRC_RANGE_READING_HPP
#include <vector> //适配fedora编译
using namespace std;

namespace grid_mapping{

struct RangeReading{
    int range_count;
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    vector<double> ranges;
};

}



#endif //SRC_RANGE_READING_HPP
