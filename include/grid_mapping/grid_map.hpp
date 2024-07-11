//
// Created by wei.sun on 19-6-21.
//
#ifndef SRC_GRID_MAP_HPP
#define SRC_GRID_MAP_HPP

#include <Eigen/Dense>
#include "grid_mapping/pose.hpp"
#include "grid_mapping/range_reading.hpp"
#include "grid_mapping/point.hpp"
//#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace Eigen;
using namespace grid_mapping;


namespace grid_mapping{

//#define THICKNESS 1
//#define L_OCC 0.9
//#define L_FREE -0.85

class GridMap {
public:
    GridMap();
    GridMap(int& w,int& h, double& res, double& l_occ, double& l_free, int& thickness);
    ~GridMap();
    int getSizeW() const {return m_sizeW;};
    int getSizeH() const {return m_sizeH;};
    PointCell getCenter()  {return m_center;};
    MatrixXd getCells() {return m_cells;};
//    MatrixXi getExploredArea() {return m_exploredArea;};
    double getResolution() {return m_resolution;};

    void setCenter(int& x, int& y);
    void setCells(MatrixXd& cells){m_cells = cells;};


    void updateBresenham(Pose& pose, RangeReading& rr);
    void clone(GridMap& newmap);

protected:

private:
    int m_sizeW, m_sizeH;
    PointCell m_center; //
    double m_resolution;
    MatrixXd m_cells;
//    MatrixXi m_exploredArea;

    double m_LikelihoodOcc,m_LikehoodFree;
    int m_thickness;

};

}



#endif //SRC_GRID_MAP_HPP
