#include "utils.h"

namespace leader_follower_controller_plugin
{
namespace leader_follower_controller
{
using namespace Eigen;

VectorXd add_noise(VectorXd y, double variance)
{
    // add noise to the measurement
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0, variance);
    for (int i = 0; i < y.size(); i++)
    {
        y(i) += dist(gen);
    }
    return y;
}

VectorXd add_noise(VectorXd y, VectorXd variance)
{
    // add noise to the measurement
    std::random_device rd;
    std::mt19937 gen(rd());
    float stdev = 0;
    for (int i = 0; i < y.size(); i++)
    {
        stdev = sqrt(variance(i));
        std::normal_distribution<double> dist(0, stdev);
        y(i) += dist(gen);
    }
    return y;
}

MatrixXd calculate_pseudoinverse(const MatrixXd& matrix) {
    // Calculate the pseudo-inverse of a matrix using SVD
    Eigen::JacobiSVD<MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd pseudo_inverse = svd.solve(MatrixXd::Identity(matrix.rows(), matrix.cols()));
    return pseudo_inverse;
  }


} // namespace leader_follower_controller
} // namespace leader_follower_controller_plugin