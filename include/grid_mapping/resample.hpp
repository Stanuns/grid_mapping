//
// Created by wei.sun on 19-6-28.
//

#ifndef SRC_RESAMPLE_HPP
#define SRC_RESAMPLE_HPP

#include "grid_mapping/particle.hpp"
#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

namespace grid_mapping{

class Resample{
public:
    Resample();
    ~Resample();
    static vector<grid_mapping::Particle> low_variance_sampler(vector<grid_mapping::Particle>& particles, VectorXd& npw, int& N);

protected:

private:


};

}


#endif //SRC_RESAMPLE_HPP
