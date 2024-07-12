//
// Created by wei.sun on 19-6-20.
//

#include "grid_mapping/observation.hpp"
#include "grid_mapping/grid_map.hpp"
#include "Eigen/Core"
#include "grid_mapping/point.hpp"
#include "grid_mapping/pose.hpp"
#include <math.h>

using namespace Eigen;
using namespace grid_mapping;

#define SIGMA m_sigma*m_n
namespace grid_mapping{
Observation::Observation(){;}
Observation::Observation(double sigma, double miss, double n, int kernel_size, double likelihood_threshold){
    m_sigma = sigma;
    m_missLikelihood = miss;
    m_n = n;

    m_kernel_size_ob = kernel_size;
    m_likelihood_threshold_ob = likelihood_threshold;
}

Observation::~Observation(){;}

void Observation::prob(double& pr, double& l, int& hit, GridMap& map, RangeReading &rr, Pose& curr_pose){

    pr = 1;
    l = 0;
    hit = 0;

    MatrixXd likelihoods = map.getCells();
    int sizeW = map.getSizeW();
    int sizeH = map.getSizeH();
    PointCell center = map.getCenter();
    double res = map.getResolution();

    Pose p_end;
    PointCell pCell_end;
    VectorXd distances = VectorXd::Zero(rr.range_count);
    for(int i=0; i<rr.range_count; i++){
        double r = rr.ranges[i];
        if(r >= rr.range_max || r <= rr.range_min){
            continue;
        }

        double alpha = rr.angle_min+rr.angle_increment*i;
        //光束终点端 转成图片cell坐标(row,column)
        p_end.x = curr_pose.x + r*cos(alpha + curr_pose.th);
        p_end.y = curr_pose.y + r*sin(alpha + curr_pose.th);
        pCell_end = pose2cellCoor(p_end, center, res);

        bool found = 0;
        double distance = 0;
        double llh = 0;
        Pose p_temp;
        double dis_temp = 0;
        //找出距离本次range_reading端点pCell_end的最近的占据格栅（p > LIKELIHOOD_THRESHOLD_OB）
        for(int a = pCell_end.x-m_kernel_size_ob; a<=pCell_end.x+m_kernel_size_ob; a++){
            for(int b = pCell_end.y-m_kernel_size_ob; b<=pCell_end.y+m_kernel_size_ob; b++){
                if(a < 0 || b < 0 || a > sizeW-1 || b > sizeH-1){
                    continue;
                }

                llh = likelihoods(sizeH-b-1,a);
                if(llh > m_likelihood_threshold_ob){
                    p_temp.x = (a - center.x)*res;
                    p_temp.y = (b - center.y)*res;
                    dis_temp = computeDistanceSquare(p_end,p_temp);

                    if (found == 0){
                        distance = dis_temp;
                        found = 1;
                    }else{
                        if(distance > dis_temp){
                            distance = dis_temp;
                        }
                    }
                }

            }
        }

        if(found){
            l = l - distance/(2*SIGMA*SIGMA);
            distances(i) = distance;
            hit = hit + 1;
        }else{
            l = l + m_missLikelihood/pow(m_n,2);
        }

    }

    pr = 1.0-1.0/(1.0+exp(l));

}

//void Observation::prob(double& pr, double& l, int& hit, GridMap& map, RangeReading &rr, Pose& curr_pose){
//
//    pr = 1;
//    l = 0;
//    hit = 0;
//
//    MatrixXd likelihoods = map.m_cells;
//    double sizeW = map.m_sizeW;
//    double sizeH = map.m_sizeH;
//    PointCell center = map.m_center;
//    double res = map.m_resolution;
//
//    Pose p_end;
//    PointCell pCell_end;
//    for(int i=0; i<rr.range_count; i++){
//        double r = rr.ranges[i];
//        if(r >= rr.range_max || r <= rr.range_min){
//            continue;
//        }
//
//        double alpha = rr.angle_min+angle_increment*i;
//        //光束终点端 转成图片cell坐标(row,column)
//        p_end.x = curr_pose.x + r*cos(alpha + curr_pose.th);
//        p_end.y = curr_pose.y + r*sin(alpha + curr_pose.th);
//        pCell_end = pose2cellCoor(p_end, center, res);
//
//        bool found = 0;
//        double distance = 0;
//        double llh = 0;
//        Pose p_temp;
//        double dis_temp = 0;
//        //找出距离本次range_reading端点pCell_end的最近的占据格栅（p > LIKELIHOOD_THRESHOLD_OB）
//        for(int a = pCell_end.x-KERNEL_SIZE_OB; a<=pCell_end.x+KERNEL_SIZE_OB; a++){
//            for(int b = pCell_end.y-KERNEL_SIZE_OB; b<=pCell_end.y+KERNEL_SIZE_OB; b++){
//                if(a < 1 || b < 1 || a > sizeW || b > sizeH){
//                    continue;
//                }
//
//                llh = likelihoods(sizeH-b,a-1);
//                if(llh > LIKELIHOOD_THRESHOLD_OB){
//                    p_temp.x = (a - center.x)*res;
//                    p_temp.y = (b - center.y)*res;
//                    dis_temp = distance(p_end,p_temp);
//                }
//
//                if (found == 0){
//                    distance = dis_temp;
//                    found = 1;
//                }else{
//                    if(distance > dis_temp){
//                        distance = dis_temp;
//                    }
//                }
//
//            }
//        }
//
//        if(found){
//            pr = pr * (1/(sqrt(2*M_PI)*SIGMA))*exp(-distance/(2*SIGMA*SIGMA));
//            hit = hit + 1;
//        }else{
//            distance = 2*sqrt(SIGMA);
//            pr = pr * (1/(sqrt(2*M_PI)*SIGMA))*exp(-distance/(2*SIGMA*SIGMA));
//        }
//
//    }
//
//    l = l + log(pr/(1-pr));
//}


}
