//
// Created by wei.sun on 19-6-28.
//

#include "grid_mapping/resample.hpp"
#include<stdlib.h>
#include<stdio.h>
#include <iostream>
#include<bits/stdc++.h>  //适配fedora编译
//#include <Eigen/Dense>

using namespace std;
//using namespace Eigen;

namespace grid_mapping{

Resample::Resample(){;}
Resample::~Resample(){;}

vector<grid_mapping::Particle> Resample::low_variance_sampler(vector<grid_mapping::Particle>& particles, VectorXd& npw, int& N)
{
    vector<grid_mapping::Particle> newGenerationParticles(N);

    //产生[0,1]之间均匀分布的随机数
    std::mt19937 rng;
    rng.seed(std::random_device()());
    std::uniform_real_distribution<double> uniformDist(0, 1);

    double r = uniformDist(rng)*(1/N);
    double c = npw(0);
    int i=1, j=1;
    for(int n=1; n<=N; n++){
        double U = r + (n-1)*(1/N);
        while(U > c){
            i++;
            if(i < 1 || i > N){
                cout<<"Resample, there is a bug......"<<endl;
            }
            c = c + npw(i-1);
        }

        //clone the particle
        newGenerationParticles[j-1].copy(particles[i-1]);
        newGenerationParticles[j-1].setNo(j-1);
        newGenerationParticles[j-1].setWeight(0.0f);

        j++;
    }

    return newGenerationParticles;
}

}