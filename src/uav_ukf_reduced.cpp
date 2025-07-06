#include "uav_ukf_reduced.h"

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

UAVUKFRED::UAVUKFRED()
{
    // initialize the ukf
    ukf = ukf_reduced_t(transition_model_red, observation_model_red);
    Q = ukf_reduced_t::Q_t::Identity()*0.001;
    R = ukf_reduced_t::R_t::Identity()*0.001;
    statecov_initialized = false;
    this->set_covariance(Q, R);
    this->set_sigma_params(1e-3, 0);
    ukf_reduced_t::x_t initial = ukf_reduced_t::x_t::Zero();
    double m = LEADER_MASS;
    double g = 9.81;
    initial(12) = m*g;
    this->set_initial_state(initial, ukf_reduced_t::P_t::Identity()*0.001);
}

void UAVUKFRED::predict_step(double dt)
{
    // predict the state
    ukf_reduced_t::u_t input = ukf_reduced_t::u_t::Zero();
    statecov = ukf.predict(statecov, input, Q, dt);
}

void UAVUKFRED::update_step(VectorXd y)
{
    // update the state
    ukf_reduced_t::z_t measurement = ukf_reduced_t::z_t::Zero();
    measurement << y;
    statecov = ukf.correct(statecov, measurement, R);
}

void UAVUKFRED::set_covariance(ukf_reduced_t::Q_t Q_, ukf_reduced_t::R_t R_)
{
    Q = Q_;
    R = R_;
}

void UAVUKFRED::set_initial_state(ukf_reduced_t::x_t x0, ukf_reduced_t::P_t P0)
{
    ukf_reduced_t::statecov_t statecov_({x0, P0});
    statecov = statecov_;
    statecov_initialized = true;
}

std::vector<std::pair<ukf_reduced_t::x_t, ukf_reduced_t::P_t>>UAVUKFRED::predict_future_states(double dt, int N)
{
    std::vector<std::pair<ukf_reduced_t::x_t, ukf_reduced_t::P_t>> predictions;
    ukf_reduced_t::statecov_t current_state = statecov;
    ukf_reduced_t::x_t x_pred = current_state.x;
    ukf_reduced_t::P_t P_pred = current_state.P;
    for (int i = 0; i < N; i++)
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
ukf_reduced_t::x_t transition_model_red(ukf_reduced_t::x_t x, ukf_reduced_t::u_t u, double dt)
{
    // states = [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, thrust]
    // inputs = none
    double g = 9.81;
    double m = LEADER_MASS;
    // to help with the naming
    double roll = x(6);
    double pitch = x(7);
    double yaw = 0; //x(8);
    double T = x(12);
    double ax = (T/m) * (cos(yaw)*cos(roll)*sin(pitch) + sin(yaw)*sin(roll));
    double ay = (T/m) * (sin(yaw)*cos(roll)*sin(pitch) - cos(yaw)*sin(roll));
    double az = (T/m) * (cos(roll)*cos(pitch)) - g;
    ukf_reduced_t::x_t x_next = ukf_reduced_t::x_t::Zero();
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
    return x_next;
}

ukf_reduced_t::z_t observation_model_red(ukf_reduced_t::x_t x)
{
    ukf_reduced_t::z_t output = ukf_reduced_t::z_t::Zero();
    // position
    output(0) = x(0);
    output(1) = x(1);
    output(2) = x(2);
    return output;
}

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin