//
// Created by wei.sun on 19-7-2.
//

#include "grid_mapping/point.hpp"

namespace grid_mapping{

    PointCell pose2cellCoor(const Pose& p, const PointCell& m_center, const double& m_resolution){
        PointCell p_cell;
        p_cell.x = (int)floor( ((m_center.x + 0.5)*m_resolution + p.x)/m_resolution);
        p_cell.y = (int)floor( ((m_center.y + 0.5)*m_resolution + p.y)/m_resolution) ;
        return p_cell;
    }
}