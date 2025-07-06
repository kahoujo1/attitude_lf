#ifndef __UAV_UKF__
#define __UAV_UKF__

#include <eigen3/Eigen/Eigen>
#include <mrs_lib/ukf.h>
#include <chrono>
#include "lkf.h"
namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

const int n_states = 15;
const int n_inputs = 0;
const int n_measurements = 6;

using ukf_t = mrs_lib::UKF<n_states, n_inputs, n_measurements>;
 
// Some helpful aliases to make writing of types shorter
using Q_t = ukf_t::Q_t;
using tra_model_t = ukf_t::transition_model_t;
using obs_model_t = ukf_t::observation_model_t;
using x_t = ukf_t::x_t;
using P_t = ukf_t::P_t;
using u_t = ukf_t::u_t;
using z_t = ukf_t::z_t;
using R_t = ukf_t::R_t;
using statecov_t = ukf_t::statecov_t;

class UAVUKF
{
    public:
        UAVUKF();
        // methods for estimation
        void predict_step(double dt);
        void update_step(VectorXd y);
        void update_and_predict(VectorXd y, double dt)
        {
            predict_step(dt);
            update_step(y);
        }
        // methods for prediction
        std::vector<std::pair<x_t, P_t>>predict_future_states(double dt, int N);
        // methods for settings
        void set_covariance(Q_t Q, R_t R);
        void set_initial_state(x_t x0, P_t P0);

        bool is_initialized() { return statecov_initialized; }
        statecov_t get_statecov() { return statecov; }
        void set_sigma_params(double alpha, double kappa) { 
            ukf.setConstants(alpha,kappa, 2);
        }

    private:
        // mrs implementation of the unscented kalman filter
        bool statecov_initialized = false;
        ukf_t ukf;
        statecov_t statecov;
        Q_t Q;
        R_t R;
};



x_t transition_model(x_t x, u_t u, double dt);
z_t observation_model(x_t x);

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin


#endif // __UAV_UKF__