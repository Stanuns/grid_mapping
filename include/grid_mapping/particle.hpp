//
// Created by wei.sun on 19-6-20.
//

#ifndef SRC_PARTICLE_HPP
#define SRC_PARTICLE_HPP

#include <Eigen/Dense>
#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/pose.hpp"
#include "grid_mapping/range_reading.hpp"

using namespace Eigen;
namespace grid_mapping{

class Particle {
public:
    Particle();
    Particle(GridMap& map);
    ~Particle();

    void init(int& number_order, Pose& initial_pose, RangeReading& range_reading);
    void integrateRangeReading(Pose& initial_pose, RangeReading& range_reading);
    void update(RangeReading& range_reading, Pose &p, double& increll, int& t, int& hit); //increll incremental log likelihood
    Pose currPose();
    GridMap getMap() const { return m_map;};
    Pose getTraceLast() const { return m_trace_last;};
    int getTraceIdx() const { return m_traceIdx;};
    int getTimeIdx() const { return m_timeIdx;};
    int getHitSmLast() const { return m_hitSm_last;};
    double getWeight() const { return m_weight;};
    void copy(Particle& pa);
    void setNo(int i){m_no = i;};
    void setWeight(double w){m_weight = w;};

protected:


private:
    int m_no; //number order
    double m_weight;
    GridMap m_map;
//    Matrix<double, 3, Dynamic> m_trace; //每个particle的行走路径点的全记录
    Pose m_trace_last; //只记录该粒子当前时刻的trace位姿

    int m_traceIdx; //particle 路径点的数目
    int m_timeIdx; //particle 迭代的次数


//    VectorXi m_hitSm; //每个particle的行走路径点的scan_matcher的hit值全记录
    int m_hitSm_last; //只记录该粒子当前时刻的an_matcher的hit值


};

}



#endif //SRC_PARTICLE_HPP
