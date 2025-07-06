#ifndef __UTILS_H__
#define __UTILS_H__

#include <eigen3/Eigen/Eigen>
#include <random>

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;
// my utils

VectorXd add_noise(VectorXd y, double variance); // adds noise to the measurement
VectorXd add_noise(VectorXd y, VectorXd variance); // adds noise to the measurement
MatrixXd calculate_pseudoinverse(const MatrixXd& matrix); // calculates the pseudo-inverse of a matrix

} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin


#endif