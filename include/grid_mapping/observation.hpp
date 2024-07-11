//
// Created by wei.sun on 19-6-20.
//

#ifndef SRC_OBSERVATION_HPP
#define SRC_OBSERVATION_HPP

#include "grid_mapping/grid_map.hpp"
//#define KERNEL_SIZE_OB 1
//#define LIKELIHOOD_THRESHOLD_OB 0.8
namespace grid_mapping{

class Observation{
public:
    Observation();
    Observation(double sigma, double miss, double n, int kernel_size, double likelihood_threshold);
    ~Observation();
    void prob(double& pr, double& l, int& hit, GridMap& map, RangeReading &rr, Pose& pose);

protected:

private:
    double m_sigma;
    double m_missLikelihood;
    double m_n; //调节observation的方差
    int m_kernel_size_ob;
    double m_likelihood_threshold_ob;

};

}




#endif //SRC_OBSERVATION_HPP
