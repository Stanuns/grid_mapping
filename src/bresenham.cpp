//
// Created by wei.sun on 19-6-24.
//
/**
 * input:x0,y0, x1,y1, map_h
 * output:
 * MatrixXi  cell_loc, //以左下角为原点(0,0)，向右向上为正
 * cellmat_loc, //以左上角为原点(0,0)，向右向下为正
 * int n_line;
 * */


#include <math.h>
#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/bresenham.hpp"
#include "Eigen/Core"

using namespace std;
using namespace Eigen;

namespace grid_mapping{

Bresenham::Bresenham(PointCell& a, PointCell& b, int& h){
    x0 = a.x;
    y0 = a.y;
    x1 = b.x;
    y1 = b.y;
    map_h = h;
}
Bresenham::~Bresenham(){;}

void Bresenham::doBresenham(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line){
    if (abs(y1-y0)<abs(x1-x0)){
        //斜率|k|<1
        if (x0>x1){
            MatrixXi cellmat_loc_temp;
            MatrixXi cell_loc_temp;
            plotLineLow(cellmat_loc_temp, cell_loc_temp, n_line, x1, y1, x0, y0, map_h);
            cell_loc = cell_loc_temp.rowwise().reverse();
            cellmat_loc = cellmat_loc_temp.rowwise().reverse();

//            plotLineLow(cellmat_loc, cell_loc, n_line, x1, y1, x0, y0, map_h);
//            //颠倒首尾端点 注意以下直接赋值是错误的,存在eigen矩阵混淆，需要.eval()
//            cell_loc = cell_loc.rowwise().reverse().eval();
//            cellmat_loc = cellmat_loc.rowwise().reverse().eval();

        }else{
            plotLineLow(cellmat_loc, cell_loc, n_line, x0, y0, x1, y1, map_h);
        }

    }else{
        //斜率|k|>=1
        if (y0>y1){
            MatrixXi cellmat_loc_temp;
            MatrixXi cell_loc_temp;
            plotLineHigh(cellmat_loc_temp, cell_loc_temp, n_line, x1, y1, x0, y0, map_h);
            cell_loc = cell_loc_temp.rowwise().reverse();
            cellmat_loc = cellmat_loc_temp.rowwise().reverse();

//            plotLineHigh(cellmat_loc, cell_loc, n_line, x1, y1, x0, y0, map_h);
//            //颠倒首尾端点 注意以下直接赋值是错误的,存在eigen矩阵混淆，需要.eval()
//            cell_loc = cell_loc.rowwise().reverse().eval();
//            cellmat_loc = cellmat_loc.rowwise().reverse().eval();
        }else{
            plotLineHigh(cellmat_loc, cell_loc, n_line, x0, y0, x1, y1, map_h);
        }
    }
}


void Bresenham::plotLineLow(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line, int x0_temp, int y0_temp, int x1_temp, int y1_temp, int map_h_temp){
    int dx = x1_temp - x0_temp;
    int dy = y1_temp - y0_temp;
    int yi = 1;
    if(dy<0){
        yi = -1;
        dy = -dy;
    }
    int D = 2*dy - dx;
    int yy = y0_temp;

    //给出该条直线共有的点数
    n_line = x1_temp-x0_temp+1;

    cell_loc = MatrixXi::Zero(2,n_line);
    cellmat_loc = MatrixXi::Zero(2,n_line);

    int k = 1;
    for (int xx=x0_temp; xx<=x1_temp; xx++){
        cell_loc(0, k-1) = xx ;
        cell_loc(1, k-1) = yy;

        cellmat_loc(0, k-1) = map_h_temp-yy-1;
        cellmat_loc(1, k-1) = xx;

        if(D>0){
            yy = yy + yi;
            D = D - 2*dx;
        }
        D = D + 2*dy;
        k++;
    }

}

void Bresenham::plotLineHigh(MatrixXi& cellmat_loc, MatrixXi& cell_loc, int& n_line, int x0_temp, int y0_temp, int x1_temp, int y1_temp, int map_h_temp){
    int dx = x1_temp - x0_temp;
    int dy = y1_temp - y0_temp;
    int xi = 1;
    if(dx<0){
        xi = -1;
        dx = -dx;
    }
    int D = 2*dx - dy;
    int xx = x0_temp;

    //给出该条直线共有的点数
    n_line = y1_temp-y0_temp+1;

    cell_loc = MatrixXi::Zero(2,n_line);
    cellmat_loc = MatrixXi::Zero(2,n_line);

    int k = 1;
    for (int yy=y0_temp; yy<=y1_temp; yy++){
        cell_loc(0, k-1) = xx ;
        cell_loc(1, k-1) = yy;

        cellmat_loc(0, k-1) = map_h_temp-yy-1;
        cellmat_loc(1, k-1) = xx;

        if(D>0){
            xx = xx + xi;
            D = D - 2*dy;
        }
        D = D + 2*dx;
        k++;
    }
}


}

