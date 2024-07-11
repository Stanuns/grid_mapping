//
// Created by wei.sun on 19-6-20.
//

#include "grid_mapping/particle_filter.hpp"
#include "grid_mapping/control.hpp"
#include "grid_mapping/motion.hpp"
#include "grid_mapping/observation.hpp"
#include "grid_mapping/scan_matcher.hpp"
#include "grid_mapping/grid_map.hpp"
#include "grid_mapping/pose.hpp"
#include "grid_mapping/resample.hpp"
#include <iostream>

#include <Eigen/Dense>
using namespace grid_mapping;
using namespace Eigen;
using namespace std;

namespace grid_mapping{
ParticleFilter::ParticleFilter(){;}

ParticleFilter::ParticleFilter(GridMap& ma, Motion& mo, Observation& ob, ScanMatcher& sm){
    map = ma;
    motion = mo;
    observation = ob;
    scanMatcher = sm;
}

ParticleFilter::~ParticleFilter(){;}

void ParticleFilter::initParticleFilter(int n, double td, double ta, int th){
    N = n;
    //初始化各个粒子
    for(int i=0; i<N; i++){
        GridMap new_map;
        map.clone(new_map);
        Particle pa(new_map);
        particles.push_back(pa);
    }


    loop = 0;
    t = 0;
    threshold_distance = td; //0.02
    threshold_angle = ta; //25*M_PI/180
    threshold_hit = th; //40


    distanceTravelled = 0;
    angleTurned = 0;
}


//void ParticleFilter::doParticleFilter(Pose& initPose, RangeReading& rr, Pose& odomPose){
//    mainProcess(initPose, rr, odomPose);
//}

//迭代执行
void ParticleFilter::doParticleFilter(Pose& initPose, RangeReading& rangeReading, Pose& odomPose){
    currOdomPose = odomPose;
    currRangeReading = rangeReading;

    if(loop > 0){
        distanceTravelled = distanceTravelled + computeDistance(currOdomPose, prevOdomPose);
        angleTurned = angleTurned + fabs(currOdomPose.th - prevOdomPose.th);

        if(distanceTravelled > threshold_distance || angleTurned > threshold_angle){
            Control u = Control(lastProcessOdomPose, currOdomPose);

            /***
             * particle filter
             * */
            VectorXd llw = VectorXd::Zero(N);
            VectorXi hits = VectorXi::Zero(N);
//            MatrixXd samples = MatrixXd::Zero(4,N) ;

            //test time
            clock_t start, finish;
            start = clock();
            double duration;
            Pose guessPose; //pose after motion sample
            for(int n=0; n<N; n++){
                motion.sample(guessPose, particles[n].currPose(), u);
                //test
//                motion.closeForm(guessPose, particles[n].currPose(), u);

//                samples.topRows<3>().col(n)  <<  guessPose.x, guessPose.y, guessPose.th; //samples(1:3,n) = [guessPose.x,guessPose.y,guessPose.th]

                double pr_temp, l;
                int hit_temp;
                GridMap map_temp = particles[n].getMap();
//                observation.prob(pr_temp, l, hit_temp, map_temp, rangeReading, guessPose);
//                samples(3, n) = l;

                Pose optimalPose;
                int hit;
                double score;
                scanMatcher.optimize(optimalPose, hit, score, map_temp, rangeReading, guessPose);
                //test time
//                optimalPose = guessPose;
//                hit = 1000;


                if(hit < threshold_hit){
                    observation.prob(pr_temp, l, hit_temp, map_temp, rangeReading, guessPose);
                    particles[n].update(rangeReading, guessPose, l, loop, hit);
                }else{
                    observation.prob(pr_temp, l, hit_temp, map_temp, rangeReading, optimalPose);
                    particles[n].update(rangeReading, optimalPose, l, loop, hit);
                }

                llw(n) = particles[n].getWeight();
                hits(n) = hit;
            }
            finish = clock();
            duration = (double)(finish - start) / CLOCKS_PER_SEC;
            cout<<"update all the particles, time consuming:"<<duration<<endl;

            //compute normalized weight
            VectorXd::Index  maxRow_temp,maxCol_temp;
            double max_value = llw.maxCoeff(&maxRow_temp,&maxCol_temp);
            VectorXd pw = (llw.array()-max_value + 0.00001).array().exp();

            //normalized
            VectorXd norm = pw.array()/pw.sum();
            double Neff = 1/((norm.adjoint()*norm)(0,0));

            //debug
            cout << "loop = "<<loop<<", T="<<t<<", Neff="<<Neff<<", Avg hits ="<<hits.sum()/N<<endl;


            VectorXd::Index  maxRow,maxCol;
            double max_value2 = norm.maxCoeff(&maxRow,&maxCol);
            best_particle = particles[maxRow];


            //resample
            if(Neff < (N/5)*2){
                cout << "..........................resmaple...................................." <<endl;

                //test time
                clock_t start1, finish1;
                start1 = clock();
                double duration1;

                particles = Resample::low_variance_sampler(particles, norm, N);

                finish1 = clock();
                duration1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
                cout<<"Resample, time consuming:"<<duration1<<endl;
            }


            distanceTravelled = 0;
            angleTurned = 0;
            t = t + 1;
            lastProcessOdomPose = currOdomPose;
        }


    }else{
       for(int n=0; n<N; n++){
           particles[n].init(n, initPose, rangeReading);
       }

       //初始化时,所有particle的m_map是相同的.
       best_particle = particles[0];
       lastProcessOdomPose = currOdomPose;
    }

    prevOdomPose = currOdomPose;
    loop ++;
}


}