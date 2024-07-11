//
// Created by wei.sun on 19-6-20.
//
#ifndef SRC_PARTICLE_FILTER_HPP
#define SRC_PARTICLE_FILTER_HPP

#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/particle.hpp"
#include "grid_mapping/motion.hpp"
#include "grid_mapping/observation.hpp"
#include "grid_mapping/scan_matcher.hpp"
#include <math.h>

//#define THRESHOLD_DISTANCE 0.02
//#define THRESHOLD_ANGLE 25*M_PI/180
//#define THRESHOLD_HIT 40

namespace grid_mapping{

class ParticleFilter{
public:
    ParticleFilter();
    ParticleFilter(GridMap& ma, Motion& mo, Observation& ob, ScanMatcher& sm);
    ~ParticleFilter();

    void initParticleFilter(int n, double td, double ta, int th);
    void doParticleFilter(Pose& initPose, RangeReading& rangeReading, Pose& odomPose);
//    void mainProcess(Pose& initPose, RangeReading& rr, Pose& odomPose);
    void getBestParticle(Particle &bp){bp = best_particle;};

protected:


private:
    int N; //number of particles
    GridMap map;
    Motion motion;
    Observation observation;
    ScanMatcher scanMatcher;

    vector<grid_mapping::Particle> particles;
    int loop;
    int t;
    double distanceTravelled, angleTurned;

    Particle best_particle;
    Pose prevOdomPose, currOdomPose, lastProcessOdomPose;
    RangeReading currRangeReading;

    double threshold_distance;
    double threshold_angle;
    int threshold_hit;

};


}




#endif //SRC_PARTICLE_FILTER_HPP
