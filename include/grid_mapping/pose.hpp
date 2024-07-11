//
// Created by wei.sun on 19-6-21.
//

#ifndef SRC_POSE_HPP
#define SRC_POSE_HPP

#include <math.h>
using namespace std;

namespace grid_mapping{

    struct Pose{
        double x, y, th;
        Pose(double a=0.0, double b=0.0, double c=0.0){
            x = a;
            y = b;
            th = c;
        }
    };

    double computeDistance(const Pose& x1,const Pose& x2);
    double computeDistanceSquare(const Pose& x1,const Pose& x2);

}




#endif //SRC_POSE_HPP
