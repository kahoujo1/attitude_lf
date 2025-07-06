#ifndef __MODEL_H__
#define __MODEL_H__

#include "eigen3/Eigen/Eigen"

#define N_STATES 10
#define N_INPUTS 3 // we do not control heading
#define N_OUTPUTS 6

namespace leader_follower_controller_plugin
{

using namespace Eigen;

struct UAVModelParameters
{
    double m = 1.70; // mass
    double g = 9.81; // gravity
    // time constants 
    double tau1 = 0.15; // roll 
    double tau2 = 0.15; // pitch
    double tau3 = 0.15; // yaw
    double tau4 = 0.1; // thrust
    // gain
    double K1 = 1; 
    double K2 = 1;
    double K3 = 1; 
    double K4 = 1; 
};

class UAVModel
{
public:
    UAVModel();
    UAVModel(UAVModelParameters params_);
    //~UAVModel();
    UAVModelParameters params;
    std::tuple<MatrixXd, MatrixXd, MatrixXd> getLinModel(double Ts = 0.01);   
    // parameter manipulation
    UAVModelParameters getParams();
    void set_params(UAVModelParameters params_);
};

} // namespace leader_follower_controller_plugin

#endif