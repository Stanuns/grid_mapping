//
// Created by wei.sun on 19-6-20.
//
#ifndef SRC_SCAN_MATCHER_HPP
#define SRC_SCAN_MATCHER_HPP

#include <Eigen/Dense>
#include "grid_mapping/pose.hpp"
#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/range_reading.hpp"
//#define DELTA 10
//#define KERNEL_SIZE 1
//#define LIKELIHOOD_THRESHOLD 0.8


namespace grid_mapping{

class ScanMatcher{
public:
    ScanMatcher();
    ScanMatcher(double translation_step, double angle_step, int iteration, int delta, double kernel_size_sm, double likelihood_threshold, bool debug);
    ~ScanMatcher();
    void optimize(Pose& optimalPose, int& hit, double& score, GridMap& map, RangeReading& cur_rr, Pose& estimatedPose);

protected:

    void getScore(int& hit, double& score, Pose& bestPose, RangeReading& cur_rr,  GridMap& map);
    Pose computePose(Pose p, MatrixXd m);

private:
    double m_translation_step, m_angle_step;
    int m_iteration; //迭代次数
    int m_delta; // 10
    int m_kernel_size_sm; //1
    double m_likelihood_threshold_sm; //0.8
    bool m_debug;




};

}


#endif //SRC_SCAN_MATCHER_HPP
