#include "lkf.h"

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

LKF::LKF()
{
    // std::tie(A,B,C) = model.getLinModel(0); // get continuous model
    std::tie(A, C) = get_lkf_model(); // get reduced model
    Q = MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES)*0.1;
    R = MatrixXd::Identity(LKF_N_OUTPUTS, LKF_N_OUTPUTS)*0.1;
    x_hat = VectorXd::Zero(LKF_N_STATES);
    P = MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES)*0.001;
    // reduced model
    x_hat_red = VectorXd::Zero(LKF_N_STATES);
    P_red = MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES)*0.001;
    R_red = MatrixXd::Identity(LKF_N_REDUCED_OUTPUTS, LKF_N_REDUCED_OUTPUTS)*0.001;
    C_red = MatrixXd::Zero(LKF_N_REDUCED_OUTPUTS, LKF_N_STATES);
    C_red(0, 0) = 1;
    C_red(1, 1) = 1;
    C_red(2, 2) = 1;
}

void LKF::predict_step(double dt)
{
    // discrete time model
    MatrixXd Ad = (MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES) + A*dt);
    // predict the state
    x_hat = Ad*x_hat;
    // predict the covariance
    P = Ad*P*Ad.transpose() + Q;
}

void LKF::update_step(VectorXd y)
{
    // calculate the kalman gain
    MatrixXd K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    // update the state estimate
    x_hat = x_hat + K*(y - C*x_hat);
    // update the covariance
    P = (MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES) - K*C)*P;
}

void LKF::predict_step_reduced(double dt)
{
    // discrete time model
    MatrixXd Ad = (MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES) + A*dt);
    // predict the state
    x_hat_red = Ad*x_hat_red;
    // predict the covariance
    P_red = Ad*P_red*Ad.transpose() + Q_red;
}

void LKF::update_step_reduced(VectorXd y)
{
    // calculate the kalman gain
    MatrixXd K = P_red*C_red.transpose()*(C_red*P_red*C_red.transpose() + R_red).inverse();
    // update the state estimate
    x_hat_red = x_hat_red + K*(y - C_red*x_hat_red);
    // update the covariance
    P_red = (MatrixXd::Identity(LKF_N_STATES, LKF_N_STATES) - K*C_red)*P_red;
}

void LKF::set_covariance(MatrixXd Q_, MatrixXd R_)
{
    this->Q = Q_;
    this->R = R_;
}

void LKF::set_reduced_covariance(MatrixXd Q_, MatrixXd R_)
{
    this->Q_red = Q_;
    this->R_red = R_;
}

void LKF::set_initial_state(VectorXd x0, MatrixXd P0)
{
    x_hat = x0;
    P = P0;
}

VectorXd LKF::get_state_prediction()
{
    return x_hat;
}


std::vector<std::pair<VectorXd, MatrixXd>> LKF::predict_future_states(double dt, int N)
{
    std::vector<std::pair<VectorXd, MatrixXd>> predictions;
    VectorXd curr_x = x_hat;
    MatrixXd curr_P = P;
    for (int i = 0; i <= N; ++i) {
        predictions.push_back(std::make_pair(x_hat, P));
        predict_step(dt);
    }
    x_hat = curr_x;
    P = curr_P;
    return predictions;
}

std::vector<std::pair<VectorXd, MatrixXd>> LKF::predict_future_states_nonlin(double dt, int N)
{
    std::vector<std::pair<VectorXd, MatrixXd>> predictions;
    VectorXd curr_x = x_hat;
    curr_x(12) = curr_x(12) + 9.81*LEADER_MASS;
    predictions = predict_future_states(dt, N);
    for (int i = 0; i < predictions.size(); ++i) {
        predictions[i].first = curr_x;
        curr_x = nonlinear_step(curr_x, dt);
    }
    return predictions;
}


std::vector<std::pair<VectorXd, MatrixXd>> LKF::predict_future_states_wrap(double dt, int N, std::vector<std::pair<VectorXd, MatrixXd>> lin_pred, VectorXd state)
{
    std::vector<std::pair<VectorXd, MatrixXd>> predictions;
    VectorXd curr_x = state;
    curr_x(12) = curr_x(12) + 9.81*LEADER_MASS;
    for (int i = 0; i < lin_pred.size(); ++i) {
        predictions.push_back(std::make_pair(curr_x, lin_pred[i].second));
        curr_x = nonlinear_step(curr_x, dt);
    }
    return predictions;
}

std::tuple<MatrixXd, MatrixXd> LKF::get_lkf_model()
{
    // x = [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, thrust, acc_offset_x, acc_offset_y]
    // u = none
    double g = 9.81;
    MatrixXd A = MatrixXd::Zero(LKF_N_STATES, LKF_N_STATES);
    A(0, 3) = 1; // x
    A(1, 4) = 1; // y
    A(2, 5) = 1; // z
    A(3, 7) = g; // vx
    A(3,13) = -1; // ax_offset
    A(4, 6) = -g; // vy
    A(4, 14) = -1; // ay_offset 
    A(5, 12) = 1/LEADER_MASS; // vz
    A(6, 9) = 1; // roll
    A(7, 10) = 1; // pitch
    A(8, 11) = 1; // yaw

    // the rest are zeros
    MatrixXd C = MatrixXd::Zero(LKF_N_OUTPUTS, LKF_N_STATES);
    // position
    C(0, 0) = 1;
    C(1, 1) = 1;
    C(2, 2) = 1;
    // attitude
    C(3, 6) = 1;
    C(4, 7) = 1;
    C(5, 8) = 1;
    return std::make_tuple(A, C);
}

VectorXd LKF::nonlinear_step(VectorXd x, double dt)
{
    // states = [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, thrust]
    // inputs = none
    double g = 9.81;
    // to help with the naming
    double roll = x(6);
    double pitch = x(7);
    double yaw = x(8);
    double T = x(12);
    double ax_offset = x(13);
    double ay_offset = x(14);
    double c_yaw = cos(yaw);
    double s_yaw = sin(yaw);
    double c_roll = cos(roll);
    double s_roll = sin(roll);
    double c_pitch = cos(pitch);
    double s_pitch = sin(pitch);
    double ax = (T/LEADER_MASS) * (c_yaw*c_roll*s_pitch + s_yaw*s_roll) - ax_offset;
    double ay = (T/LEADER_MASS) * (s_yaw*c_roll*s_pitch - c_yaw*s_roll) - ay_offset;
    double az = (T/LEADER_MASS) * (c_roll*c_pitch) - g;
    VectorXd x_next = VectorXd::Zero(LKF_N_STATES);
    // position
    x_next(0) = x(0) + x(3)*dt + 0.5*ax*dt*dt;
    x_next(1) = x(1) + x(4)*dt + 0.5*ay*dt*dt;
    x_next(2) = x(2) + x(5)*dt + 0.5*az*dt*dt;
    // velocity
    x_next(3) = x(3) + ax*dt;
    x_next(4) = x(4) + ay*dt;
    x_next(5) = x(5) + az*dt;
    // attitude
    x_next(6) = x(6) + x(9)*dt;
    x_next(7) = x(7) + x(10)*dt;
    x_next(8) = x(8) + x(11)*dt;
    // angular velocity
    x_next(9) = x(9);
    x_next(10) = x(10);
    x_next(11) = x(11);
    // thrust
    x_next(12) = x(12);
    // // acceleration offsets
    x_next(13) = x(13);
    x_next(14) = x(14);
    return x_next;
}

} // namespace leader_follower_controller

} // namespace leader_follower_controller_plugin
