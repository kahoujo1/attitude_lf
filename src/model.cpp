#include "model.h"

namespace leader_follower_controller_plugin
{
using namespace Eigen;

// ---------------------- UAV MODEL ----------------------
UAVModel::UAVModel()
{
    UAVModelParameters params_;
    params = params_;
}
UAVModel::UAVModel(UAVModelParameters params_)
{
    params = params_;
}

UAVModelParameters UAVModel::getParams()
{
    return params;
}

void UAVModel::set_params(UAVModelParameters params_)
{
    params = params_;
}

std::tuple<MatrixXd, MatrixXd, MatrixXd> UAVModel::getLinModel(double Ts)
/*
    Returns the linearized (and discretized) model of the UAV.
*/
{
    UAVModelParameters params = getParams();
    double m = params.m;
    double g = params.g;
    double tau1 = params.tau1;
    double tau2 = params.tau2;
    double tau3 = params.tau3;
    double tau4 = params.tau4;
    double K1 = params.K1;
    double K2 = params.K2;
    double K3 = params.K3;
    double K4 = params.K4;
    // state space matrices
    MatrixXd A = MatrixXd::Zero(N_STATES, N_STATES);
    MatrixXd B = MatrixXd::Zero(N_STATES, N_INPUTS);
    MatrixXd C = MatrixXd::Zero(N_OUTPUTS, N_STATES);
    // states: [x, y, z, vx, vy, vz, roll, pitch, yaw, thrust]
    // inputs: [roll, pitch, thrust] (we dont control yaw)
    // outputs: [x, y, z, roll, pitch, yaw]
    // create A
    // position
    A(0, 3) = 1;
    A(1, 4) = 1;
    A(2, 5) = 1;
    // velocity
    A(3, 7) = g;
    A(4, 6) = -g;
    A(5, 9) = 1/m;
    // references
    A(6, 6) = -1/tau1;
    A(7, 7) = -1/tau2;
    A(8, 8) = -1/tau3;
    A(9, 9) = -1/tau4;
    // create B
    B(6, 0) = K1/tau1;
    B(7, 1) = K2/tau2;
    B(9, 2) = K4/tau4;
    // create C
    C(0,0) = 1; // x
    C(1,1) = 1; // y
    C(2,2) = 1; // z
    C(3,6) = 1; // roll
    C(4,7) = 1; // pitch
    C(5,8) = 1; // yaw
    
    if (Ts <= 0) // continuous model
    {
        return std::make_tuple(A, B, C);
    }
    // discretize the model
    A = MatrixXd::Identity(N_STATES, N_STATES) + Ts * A;
    B = Ts * B;

    return std::make_tuple(A, B, C);
}

} // namespace leader_follower_controller_plugin