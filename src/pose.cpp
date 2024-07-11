//
// Created by wei.sun on 19-7-2.
//

#include <math.h>
#include "grid_mapping/pose.hpp"

namespace grid_mapping{

    double computeDistance(const Pose& x1,const Pose& x2){
        double dist ;
        dist = sqrt(pow(x2.x-x1.x,2)+pow(x2.y-x1.y,2));
        return dist;
    }

    double computeDistanceSquare(const Pose& x1,const Pose& x2){
        double dist ;
        dist = pow(x2.x-x1.x,2)+pow(x2.y-x1.y,2);
        return dist;
    }
}