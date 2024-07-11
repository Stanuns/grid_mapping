//
// Created by wei.sun on 19-6-26.
//

#ifndef SRC_POINT_HPP
#define SRC_POINT_HPP

#include <math.h>
#include "pose.hpp"

namespace grid_mapping{

    struct PointCell{
        int row,column; //左上角为(0,0) 向右向下为正, 矩阵的读取方式
        int x,y; //左下角为(0,0) 向右向上为正
        PointCell(int r=0, int c=0, int xx=0, int yy=0){
            row = r;
            column = c;
            x = xx;
            y = yy;
        }

    };

    PointCell pose2cellCoor(const Pose& p, const PointCell& m_center, const double& m_resolution);

}


#endif //SRC_POINT_HPP
