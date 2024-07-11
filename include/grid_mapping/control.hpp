//
// Created by wei.sun on 19-6-25.
//
#ifndef SRC_CONTROL_HPP
#define SRC_CONTROL_HPP

#include "grid_mapping/pose.hpp"

namespace grid_mapping{

class Control {
public:
    Control(Pose& p1, Pose& p2);
    ~Control();
    void delta(double& delta_rot1, double& delta_trans, double& delta_rot2);

protected:

private:
    Pose pose1, pose2;

};

}



#endif //SRC_CONTROL_HPP
