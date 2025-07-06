#ifndef __UAV_UKF_REDUCED__
#define __UAV_UKF_REDUCED__

#include <eigen3/Eigen/Eigen>
#include <mrs_lib/ukf.h>
#include "lkf.h"

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

const int n_red_states = 13;
const int n_red_inputs = 0;
const int n_red_measurements = 3; // just position

using ukf_reduced_t = mrs_lib::UKF<n_red_states, n_red_inputs, n_red_measurements>;

class UAVUKFRED
{
    public:
        UAVUKFRED();
        // methods for estimation
        void predict_step(double dt);
        void update_step(VectorXd y);
        void update_and_predict(VectorXd y, double dt)
        {
            predict_step(dt);
            update_step(y);
        }
        // methods for prediction
        std::vector<std::pair<ukf_reduced_t::x_t, ukf_reduced_t::P_t>>predict_future_states(double dt, int N);
        // methods for settings
        void set_covariance(ukf_reduced_t::Q_t Q, ukf_reduced_t::R_t R);
        void set_initial_state(ukf_reduced_t::x_t x0, ukf_reduced_t::P_t P0);
        bool is_initialized() { return statecov_initialized; }
        ukf_reduced_t::statecov_t get_statecov() { return statecov; }
        void set_sigma_params(double alpha, double kappa) { 
            ukf.setConstants(alpha,kappa, 2);
        }

    private:
        // mrs implementation of the unscented kalman filter
        bool statecov_initialized = false;
        ukf_reduced_t ukf;
        ukf_reduced_t::statecov_t statecov;
        ukf_reduced_t::Q_t Q;
        ukf_reduced_t::R_t R;
};



ukf_reduced_t::x_t transition_model_red(ukf_reduced_t::x_t x, ukf_reduced_t::u_t u, double dt);
ukf_reduced_t::z_t observation_model_red(ukf_reduced_t::x_t x);

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin


#endif // __UAV_UKF__