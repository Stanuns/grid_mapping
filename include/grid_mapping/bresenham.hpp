//
// Created by wei.sun on 19-6-25.
//

#ifndef SRC_BRESENHAM_HPP
#define SRC_BRESENHAM_HPP
#include <Eigen/Dense>
using  namespace Eigen;

namespace grid_mapping{

class Bresenham{
public:
    Bresenham(PointCell& a, PointCell& b, int& h);
    ~Bresenham();
    void doBresenham(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line);

protected:
    void plotLineLow(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line, int x0_temp, int y0_temp, int x1_temp, int y1_temp, int map_h_temp);
    void plotLineHigh(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line, int x0_temp, int y0_temp, int x1_temp, int y1_temp, int map_h_temp);

private:
    int x0,y0; //起点的cell位置
    int x1,y1; //终点的cell位置
    int map_h; //地图的高度
};


}

#endif //SRC_BRESENHAM_HPP
