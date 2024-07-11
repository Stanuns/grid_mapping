//
// Created by wei.sun on 19-6-20.
//
/***
 * motion model from probabilitic robotics
 * **/

#ifndef SRC_MOTION_HPP
#define SRC_MOTION_HPP

#include "grid_mapping/pose.hpp"
#include "grid_mapping/control.hpp"

namespace grid_mapping{

class Motion{
public:
    Motion();
    Motion(double a1,double a2, double a3, double a4, double sigma_x, double sigma_y, double sigma_th);
    ~Motion();
    void closeForm(Pose& next_p, Pose curr_p, Control u);
    void sample(Pose& next_p, Pose curr_p, Control u);

protected:
    double rad2deg(double rad);
    double deg2rad(double deg);

private:
    double a1,a2,a3,a4,sigma_x,sigma_y,sigma_th;






};

}



#endif //SRC_MOTION_HPP
