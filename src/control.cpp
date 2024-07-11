//
// Created by wei.sun on 19-6-25.
//

#include "grid_mapping/control.hpp"
#include <math.h>
namespace grid_mapping{

Control::Control(Pose& p1, Pose& p2){
    pose1 = p1;
    pose2 = p2;
}

Control::~Control(){;}


void Control::delta(double& delta_rot1, double& delta_trans, double& delta_rot2){
    delta_rot1 = atan2(pose2.y - pose1.y, pose2.x - pose1.x) - pose1.th;
    delta_trans = sqrt(pow(pose2.y - pose1.y,2)+pow(pose2.x - pose1.x,2));
    delta_rot2 = pose2.th - pose1.th - delta_rot1 ;


    if(fabs(delta_rot2)>M_PI){
        delta_rot2 = atan2(sin(delta_rot2), cos(delta_rot2));
    }
}

}