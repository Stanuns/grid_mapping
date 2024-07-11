//
// Created by wei.sun on 19-6-20.
//

#include "grid_mapping/particle.hpp"
#include "grid_mapping/grid_map.hpp"

namespace grid_mapping{
Particle::Particle(){;}

Particle::Particle(GridMap& map){
    m_map = map;
}

Particle::~Particle(){;}

void Particle::init(int& number_order, Pose& initial_pose, RangeReading& range_reading){
    m_no = number_order;
    m_weight = 0.0;
    m_traceIdx = 0;
    m_timeIdx = 0;

    m_trace_last = initial_pose;
    integrateRangeReading(initial_pose, range_reading);
    m_hitSm_last = 0;
}


void Particle::integrateRangeReading(Pose& pose, RangeReading& range_reading){
    m_map.updateBresenham(pose, range_reading);
}

void Particle::update(RangeReading& range_reading, Pose &p, double& increll, int& t, int& hit){

    m_traceIdx = m_traceIdx + 1;
    m_trace_last = p;
    m_weight = m_weight + increll;
    integrateRangeReading(p, range_reading);
    m_timeIdx = t;
    m_hitSm_last = hit;
}


Pose Particle::currPose(){
    Pose p = m_trace_last;
    return p;
}

void Particle::copy(Particle& pa){
    m_map = pa.getMap();
    m_trace_last = pa.getTraceLast();
    m_traceIdx = pa.getTraceIdx();
    m_timeIdx = pa.getTimeIdx();
    m_hitSm_last = pa.getHitSmLast();
}


}
