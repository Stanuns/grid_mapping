//
// Created by wei.sun on 19-6-20.
//

#include "grid_mapping/motion.hpp"
#include <math.h>
#include <random>
using namespace std;

namespace grid_mapping{
Motion::Motion(){;};
Motion::Motion(double a,double b, double c, double d, double e, double f, double g){
    a1 = a;
    a2 = b;
    a3 = c;
    a4 = d;
    sigma_x = e;
    sigma_y = f;
    sigma_th = g;
}

Motion::~Motion(){;}

void Motion::closeForm(Pose& next_p, Pose curr_p, Control u){

    double delta_rot1, delta_trans, delta_rot2;
    u.delta(delta_rot1, delta_trans, delta_rot2);

    next_p.x = curr_p.x + delta_trans*cos(delta_rot1+curr_p.th);
    next_p.y = curr_p.y + delta_trans*sin(delta_rot1+curr_p.th);
    next_p.th = curr_p.th + delta_rot1 + delta_rot2;

    if(fabs(next_p.th)>M_PI){
        next_p.th = atan2(sin(next_p.th), cos(next_p.th));
    }
}

void Motion::sample(Pose& next_p, Pose curr_p, Control u){
    double delta_rot1, delta_trans, delta_rot2;
    u.delta(delta_rot1, delta_trans, delta_rot2);

    //sample from added noises regard with degree
    double delta_rot1_deg = rad2deg(delta_rot1);
    double delta_rot2_deg = rad2deg(delta_rot2);

    double mu{0.0};
    random_device rd;
    default_random_engine rng {rd()};

    double sigma_rot1{a1*fabs(delta_rot1_deg)+a2*fabs(delta_trans)};
    normal_distribution<> norm_rot1 {mu, sigma_rot1};
    double delta_rot1_hat = delta_rot1 - deg2rad(norm_rot1(rng));

    double sigma_trans{a3*fabs(delta_trans)+a4*fabs(delta_rot1_deg)+a4*fabs(delta_rot2_deg)};
    normal_distribution<> norm_trans {mu, sigma_trans};
    double delta_trans_hat = delta_trans - norm_trans(rng);

    double sigma_rot2{a1*fabs(delta_rot2_deg)+a2*fabs(delta_trans)};
    normal_distribution<> norm_rot2 {mu, sigma_rot2};
    double delta_rot2_hat = delta_rot2 - deg2rad(norm_rot2(rng));

    normal_distribution<> norm_x {mu, sigma_x};
    next_p.x = curr_p.x + delta_trans_hat*cos(curr_p.th+delta_rot1_hat)+norm_x(rng);
    normal_distribution<> norm_y {mu, sigma_y};
    next_p.y = curr_p.y + delta_trans_hat*sin(curr_p.th+delta_rot1_hat)+norm_y(rng);
    normal_distribution<> norm_th {mu, sigma_th};
    next_p.th = curr_p.th + delta_rot1_hat + delta_rot2_hat + deg2rad(norm_th(rng));

    if(fabs(next_p.th)>M_PI){
        next_p.th = atan2(sin(next_p.th), cos(next_p.th));
    }

}

double Motion::rad2deg(double rad){
    double deg;
    deg = (rad/M_PI)*180;
    return deg;
}

double Motion::deg2rad(double deg){
    double rad;
    deg = (deg/180)*M_PI;
    return rad;
}


}