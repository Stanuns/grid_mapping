//
// Created by wei.sun on 19-6-21.
//
/**
 * grid_map
 *
 * 地图的像素单元坐标 cell coordinate: 左上角为(0,0) 向右向下为正
 *
 * */

#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/range_reading.hpp"
#include "grid_mapping/bresenham.hpp"
#include <math.h>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace grid_mapping;

namespace grid_mapping{
GridMap::GridMap(){;}

GridMap::GridMap(int& w,int& h, double& res, double& l_occ, double& l_free, int& thickness):
    m_sizeW(w),
    m_sizeH(h),
    m_resolution(res),
    m_LikelihoodOcc(l_occ),
    m_LikehoodFree(l_free),
    m_thickness(thickness)
{
    m_cells  = MatrixXd::Zero(m_sizeH, m_sizeW);
//    m_exploredArea = MatrixXi::Zero(m_sizeW, m_sizeH);
//    m_expand = false;
}

GridMap::~GridMap(){
    ;
}

void GridMap::setCenter(int& x, int& y){
    m_center.x = x;
    m_center.y = y;
}

void GridMap::clone(GridMap& newmap){
    newmap.m_sizeW = m_sizeW;
    newmap.m_sizeH = m_sizeH;
    newmap.m_center = m_center;
    newmap.m_resolution = m_resolution;
    newmap.m_cells = m_cells; //比MatrixXd::Zero(m_sizeW, m_sizeH)快
//    newmap.m_exploredArea = m_exploredArea;

    newmap.m_LikelihoodOcc = m_LikelihoodOcc;
    newmap.m_LikehoodFree = m_LikehoodFree;
    newmap.m_thickness = m_thickness;
}

void GridMap::updateBresenham(Pose& p1, RangeReading& rr){

    double theta = p1.th;

    MatrixXi mark = MatrixXi::Zero(m_sizeH, m_sizeW);
//    MatrixXd likehoods = m_cells;

    //expand map using
    double max_x = m_sizeW, max_y = m_sizeH;
    double min_x=0, min_y=0;
    bool m_expand = false;
    int m_sizeW_expand, m_sizeH_expand;
    MatrixXd m_cells_expand;


    for(int i=0; i<rr.range_count; i++){
        double r =  rr.ranges[i];
        if(r > rr.range_max || r < rr.range_min){
            continue;
        }

        //激光束相对于机器人正方向的夹角
        double alpha  = rr.angle_min + rr.angle_increment * i;
        Pose p2;
        p2.x = r*cos(theta+alpha)+p1.x;
        p2.y = r*sin(theta+alpha)+p1.y;

        //激光束的起点p1与终点p2转换成地图的像素单元坐标 cell coordinate
        PointCell p1_cell, p2_cell;
        p1_cell = pose2cellCoor(p1, m_center, m_resolution);
        p2_cell = pose2cellCoor(p2, m_center, m_resolution);

        //judge whether expand the map
        if(p2_cell.x>m_sizeW || p2_cell.x<0 || p2_cell.y > m_sizeH || p2_cell.y < 0) {
            m_expand = true;
            if (p2_cell.x > max_x)
                max_x = p2_cell.x;
            if (p2_cell.y > max_y)
                max_y = p2_cell.y;
            if (p2_cell.x < min_x)
                min_x = p2_cell.x;
            if (p2_cell.y < min_y)
                min_y = p2_cell.y;

        }

        //直线算法

        Bresenham bresenham_(p1_cell, p2_cell, m_sizeH);
        MatrixXi  cell_loc,cellmat_loc;
        int length;
        bresenham_.doBresenham(cellmat_loc, cell_loc, length);
        for(int k=0; k<length; k++){

            //超出此时地图的范围
            if(cell_loc(0,k)<0 || cell_loc(1,k)<0 || cell_loc(0,k)> m_sizeW-1  || cell_loc(1,k)> m_sizeH-1){
                continue;
            }

            int r = cellmat_loc(0, k);
            int c = cellmat_loc(1, k);

            //debug
//          cout<<"------r:"<<r<<" c:"<<c<<" m_sizeW:"<<m_sizeW<<" m_sizeH:"<<m_sizeH<<"---------"<<endl;

            double update = m_LikelihoodOcc;
            if(k < length - m_thickness){
                update = m_LikehoodFree;
            }
            if (mark(r,c)==0){ //
                //debug
                double m_cell_test = m_cells(r,c);

                m_cells(r,c) = m_cells(r,c) + update;
                mark(r,c) = 1;
            }

            //debug
//            m_cells(400,300) = 0.8;
//            m_cells(400,301) = 0.8;
        }

    }

//    cout<<"m_expand:"<<m_expand<<endl;
    //expand the map
    if(m_expand){
        //一定程度的膨胀
        max_x = max_x + 100;
        max_y = max_y + 100;
        min_x = min_x - 100;
        min_y = min_y - 100;
        m_sizeW_expand = max_x - min_x;
        m_sizeH_expand = max_y - min_y;
        m_cells_expand = MatrixXd::Zero(m_sizeH_expand, m_sizeW_expand);
        m_cells_expand.block(max_y-m_sizeH,0-min_x,m_sizeH,m_sizeW) = m_cells;

        m_cells = m_cells_expand;
        m_sizeW = m_sizeW_expand;
        m_sizeH = m_sizeH_expand;
        m_center.x = m_center.x - min_x;
        m_center.y = m_center.y - min_y;

        m_expand = false;
    }

    //test
//    m_cells_expand = MatrixXd::Zero(600, 600);
//    m_cells = m_cells_expand;

}



}