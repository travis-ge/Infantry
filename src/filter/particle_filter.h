#pragma once
#include <iostream>

#include <random>
#include <vector>
#include <ctime>

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace Eigen;

class ParticleFilter
{
public:
    ParticleFilter(YAML::Node &config,const string param_name);
    ParticleFilter();
    ~ParticleFilter();

    Eigen::VectorXd predict();
    bool initParam(YAML::Node &config,const string param_name);
    bool initParam(ParticleFilter parent);
    bool update(Eigen::VectorXd measure);
    bool is_ready;
private:

    bool resample();

    int vector_len;
    int num_particle;

    Eigen::MatrixXd process_noise_cov;
    Eigen::MatrixXd observe_noise_cov;
    Eigen::MatrixXd weights;

    Eigen::MatrixXd matrix_estimate;
    Eigen::MatrixXd matrix_particle;
    Eigen::MatrixXd matrix_weights;

    template<typename T>
    bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
    {
        int cnt = 0;
        for(int row = 0;row < matrix.rows();row++)
        {
            for(int col = 0;col < matrix.cols();col++)
            {
                matrix(row,col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

};