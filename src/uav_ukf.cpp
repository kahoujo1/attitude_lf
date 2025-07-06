#include "uav_ukf.h"

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

UAVUKF::UAVUKF()
{
    // initialize the ukf
    ukf = ukf_t(transition_model, observation_model);
    Q = Q_t::Identity()*0.001;
    R = R_t::Identity()*0.001;
    statecov_initialized = false;
    this->set_covariance(Q, R);
    this->set_sigma_params(1e-3, 0);
    x_t initial = x_t::Zero();

    double g = 9.81;
    initial(12) = LEADER_MASS*g;
    this->set_initial_state(initial, P_t::Identity()*0.001);
}

void UAVUKF::predict_step(double dt)
{
    // predict the state
    u_t input = u_t::Zero();
    statecov = ukf.predict(statecov, input, Q, dt);
}

void UAVUKF::update_step(VectorXd y)
{
    // update the state
    auto start = std::chrono::high_resolution_clock::now();
    z_t measurement = z_t::Zero();
    measurement << y;
    auto after_init = std::chrono::high_resolution_clock::now();
    statecov = ukf.correct(statecov, measurement, R);
    auto after_update = std::chrono::high_resolution_clock::now();
}

void UAVUKF::set_covariance(Q_t Q_, R_t R_)
{
    Q = Q_;
    R = R_;
}

void UAVUKF::set_initial_state(x_t x0, P_t P0)
{
    statecov_t statecov_({x0, P0});
    statecov = statecov_;
    statecov_initialized = true;
}

std::vector<std::pair<x_t, P_t>>UAVUKF::predict_future_states(double dt, int N)
{
    std::vector<std::pair<x_t, P_t>> predictions;
    statecov_t current_state = statecov;
    x_t x_pred = current_state.x;
    P_t P_pred = current_state.P;
    for (int i = 0; i <= N; i++)
    {
        predictions.push_back(std::make_pair(x_pred, P_pred));
        // predict the state
        predict_step(dt);
        x_pred = statecov.x;
        P_pred = statecov.P;
        // predict the covariance
    }
    statecov = current_state;
    return predictions;
}
x_t transition_model(x_t x, u_t u, double dt)
{
    // states = [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, thrust, roll_offset, pitch_offset]
    // inputs = none
    double g = 9.81;
    // to help with the naming
    double roll = x(6) - x(13); // offset
    double pitch = x(7)- x(14); // offset
    double yaw = x(8);
    double T = x(12);
    double ax = (T/LEADER_MASS) * (cos(yaw)*cos(roll)*sin(pitch) + sin(yaw)*sin(roll));
    double ay = (T/LEADER_MASS) * (sin(yaw)*cos(roll)*sin(pitch) - cos(yaw)*sin(roll));
    double az = (T/LEADER_MASS) * (cos(roll)*cos(pitch)) - g;
    x_t x_next = x_t::Zero();
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
    // acceleration offsets
    x_next(13) = x(13);
    x_next(14) = x(14);
    return x_next;
}

z_t observation_model(x_t x)
{
    z_t output = z_t::Zero();
    // position
    output(0) = x(0);
    output(1) = x(1);
    output(2) = x(2);
    // attitude
    output(3) = x(6);
    output(4) = x(7);
    output(5) = x(8);
    return output;
}

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin