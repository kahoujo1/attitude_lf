#ifndef __LKF_H__
#define __LKF_H__

#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>

// #define LEADER_MASS 2.7
#define LEADER_MASS 1.7

#define LKF_N_STATES 15
#define LKF_N_OUTPUTS 6
#define LKF_N_REDUCED_OUTPUTS 3

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

class LKF
{
    public:
        LKF();

        
        void predict_step(double dt);
        void update_step(VectorXd y);

        void predict_step_reduced(double dt);
        void update_step_reduced(VectorXd y);

        void update_and_predict(VectorXd y, double dt)
        {
            predict_step(dt);
            update_step(y);
        }

        void update_and_predict_reduced(VectorXd y, double dt)
        {
            predict_step_reduced(dt);
            update_step_reduced(y);
        }
        
        void set_covariance(MatrixXd Q, MatrixXd R);
        void set_reduced_covariance(MatrixXd Q, MatrixXd R);
        void set_initial_state(VectorXd x0, MatrixXd P0);
        std::vector<std::pair<VectorXd, MatrixXd>> predict_future_states(double dt, int N);
        std::vector<std::pair<VectorXd, MatrixXd>> predict_future_states_nonlin(double dt, int N);
        std::vector<std::pair<VectorXd, MatrixXd>> predict_future_states_wrap(double dt, int N, std::vector<std::pair<VectorXd, MatrixXd>> lin_pred, VectorXd ukf_x);
        VectorXd get_state_prediction();
        std::tuple<MatrixXd, MatrixXd> get_lkf_model();
        VectorXd nonlinear_step(VectorXd x, double dt);
        // the state space representation of the system (D is zero matrix)
        MatrixXd A;
        MatrixXd C;
        // estimated state
        VectorXd x_hat;
        // estimated covariance
        MatrixXd P;
        // process noise covariance
        MatrixXd Q;
        // measurement noise covariance
        MatrixXd R;
        // reduced model:
        VectorXd x_hat_red;
        MatrixXd P_red;
        MatrixXd C_red;
        MatrixXd R_red;
        MatrixXd Q_red;
    };

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin





#endif