//
// Created by wei.sun on 19-6-20.
//

#include "grid_mapping/scan_matcher.hpp"
#include "Eigen/Core"
#include <math.h>
#include <iostream>
#include "grid_mapping/pose.hpp"
using namespace Eigen;
using namespace std;

namespace grid_mapping{
ScanMatcher::ScanMatcher(){;}
ScanMatcher::ScanMatcher(double translation_step, double angle_step, int iteration, int delta, double kernel_size_sm, double likelihood_threshold, bool debug)
{
    m_translation_step = translation_step;
    m_angle_step = angle_step;
    m_iteration = iteration;

    m_delta = delta;
    m_kernel_size_sm = kernel_size_sm;
    m_likelihood_threshold_sm =  likelihood_threshold;

    m_debug = debug;
}

ScanMatcher::~ScanMatcher(){;}

void ScanMatcher::optimize(Pose& optimalPose, int& hit, double& score, GridMap& map, RangeReading& cur_rr, Pose& estimatedPose){
    if(m_debug){
        std::cout<<"Scan matcher start..."<<std::endl;
        std::cout<<"Before scan matcher, Pose.x="<<estimatedPose.x<<" Pose.y="<<estimatedPose.y<<" Pose.th="<<estimatedPose.th<<std::endl;
    }

    double translation_step = m_translation_step;
    double angle_step = m_angle_step;

    Pose bestPose;
    bestPose.x = estimatedPose.x;
    bestPose.y = estimatedPose.y;
    bestPose.th = estimatedPose.th;
    int bestHit = 0;
    double bestScore = 0;
    getScore(bestHit, bestScore, bestPose, cur_rr, map);

    if(m_debug){
        std::cout<<"INIT score="<<bestScore<<", hit="<<bestHit<<std::endl;
    }

    Matrix<double,6,3> moves;
    int moveNum = moves.rows();

    int iter = 0,tmpHit = 0;
    while(iter<m_iteration){
        double maxMoveScore = bestScore;
        Pose bestMovePose = bestPose;
        int bestMoveHit = bestHit;

        moves << 0,-translation_step,0,
                0,translation_step,0,
                -translation_step,0,0,
                translation_step,0,0,
                0,0,angle_step*M_PI/180,
                0,0,-angle_step*M_PI/180;

        Pose testPose;
        double score = 0;
        int m = 0;
        for (int j=0; j<moveNum; j++){
            testPose = computePose(bestPose, moves.row(j));
            getScore(tmpHit, score, testPose, cur_rr, map);
            if(score>maxMoveScore){
                m = j;
                maxMoveScore = score;
                bestMovePose = testPose;
                bestMoveHit = tmpHit;
            }
        }
        if(maxMoveScore > bestScore){
            bestScore = maxMoveScore;
            bestPose = bestMovePose;
            bestHit = bestMoveHit;


            //
            translation_step = max(translation_step*0.5, 0.0);
            angle_step = max(angle_step*0.5, 0.5);
            
            if(m_debug){
                std::cout<<"Take move="<<moves.row(m)<<", score="<<bestScore<<", translation_step="<<translation_step<<", angle_step="<<angle_step<<std::endl;
            }
        }else{
            translation_step = max(translation_step*0.5, 0.0);
            angle_step = max(angle_step*0.5, 0.5);

            if(m_debug){
                std::cout<<"translation_step="<<translation_step<<", angle_step="<<angle_step<<std::endl;
            }
        }

        iter++;
    }

    optimalPose = bestPose;
    optimalPose.th = atan2(sin(optimalPose.th),cos(optimalPose.th));
    score = bestScore;
    hit = bestHit;

    if(m_debug){
       std::cout<<"After scan matcher, Pose.x="<<optimalPose.x<<" Pose.y="<<optimalPose.y<<" Pose.th="<<optimalPose.th<<std::endl;
       std::cout<<"Scan matcher end..."<<std::endl;
    }
}

void ScanMatcher::getScore(int& hit, double& score, Pose& bestPose, RangeReading& cur_rr,  GridMap& map){

    MatrixXd likelihoods = map.getCells();
    int sizeW = map.getSizeW();
    int sizeH = map.getSizeH();
    PointCell center = map.getCenter();
    double res = map.getResolution();

    score = 0;
    hit = 0;
    Pose p_end;
    PointCell pCell_end;
    Pose curr_pose = bestPose;

    for(int i=0; i<cur_rr.range_count; i++){
        double r = cur_rr.ranges[i];
        if(r >= cur_rr.range_max || r <= cur_rr.range_min){
            continue;
        }

        double alpha = cur_rr.angle_min+cur_rr.angle_increment*i;
        //光束终点端 转成图片cell坐标(row,column)
        p_end.x = curr_pose.x + r*cos(alpha + curr_pose.th);
        p_end.y = curr_pose.y + r*sin(alpha + curr_pose.th);
        pCell_end = pose2cellCoor(p_end, center, res);

        bool found = 0;
        double distance = 0;
        double llh = 0;
        Pose p_temp;
        double dis_temp = 0;
        //找出距离本次range_reading端点pCell_end的最近的占据格栅（p > LIKELIHOOD_THRESHOLD）
        for(int a = pCell_end.x - m_kernel_size_sm; a <= pCell_end.x + m_kernel_size_sm; a++){
            for(int b = pCell_end.y-m_kernel_size_sm; b<=pCell_end.y+m_kernel_size_sm; b++){
                if(a < 0 || b < 0 || a > sizeW-1 || b > sizeH-1){
                    continue;
                }

                llh = likelihoods(sizeH-b-1,a);
                if(llh > m_likelihood_threshold_sm){
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
            score = score + exp(-distance/m_delta);
            hit = hit + 1;
        }else{
            //todo
        }

    }

}

Pose ScanMatcher::computePose(Pose p, MatrixXd m){
    Pose testPose;
    testPose.x = p.x + m(0,0);
    testPose.y = p.y + m(0,1);
    testPose.th = p.th + m(0,2);
    return testPose;
}


}