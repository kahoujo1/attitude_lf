#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

// for mrs_msgs
#include <mrs_uav_managers/controller.h>

#include "model.h"
#include "utils.h"

#include "acados_c/ocp_qp_interface.h"
#include "acados/utils/print.h"

#include <eigen3/Eigen/Eigen>

#include <mrs_lib/attitude_converter.h>

#include <linalg_def.h>
#include <control_def.h>

#include <optional>

namespace leader_follower_controller_plugin
{

namespace leader_follower_controller
{

using namespace Eigen;

class MPC{
	public:
	MPC(UAVModel &model);
	~MPC();

	std::tuple<VectorXd, bool, std::optional<VectorXd>> getControlAction();
	//void setReferenceTrajectory(mrs_msgs::MpcPredictionFullState mrs_traj);
	void updateState(VectorXd state_);
	void setReferenceConstant(Vector3d ref); // set constant reference
	void setReferenceCombined(std::vector<std::pair<VectorXd, MatrixXd>> predictions, double alpha, double dt, Vector3d offset = Vector3d::Zero()); // set reference based on the predictions
	// others
	bool referenceIsSet();
	void setParameter(std::string paramName, MatrixXd input);
	void setParameter(std::string paramName, double input);
	void updateModel(double mass);

	protected:
	bool refIsSet = false;
	VectorXd state;
	private:
	void updateConstraints();
	//void updateReference();

	// acados classes
	ocp_qp_solver* qpSolver;
	ocp_qp_in* qpIn;
	ocp_qp_out* qpOut;
	ocp_qp_xcond_solver_config* config;
	ocp_qp_xcond_solver_dims* solverDims;
	ocp_qp_dims* dims;
	void* opts;

	// parameters of the cost function
	Matrix3d R;
	MatrixXd Q;
	MatrixXd C;

	// constraints
		// model constraints
	UAVModel model_;

    // control action calculated in the previous iteration
  	Vector3d lastCtrlAction = Vector3d::Zero();

	// control actions constraints
      // lower bound 
	VectorXd lbu = -0.1*Vector3d::Ones();
      // upper bound 
	VectorXd ubu = 0.1*Vector3d::Ones();
      // lower bound on increment
	VectorXd lbui = 0.5*Vector3d::Ones();
      // upper bound on increment
	VectorXd ubui = 0.5*Vector3d::Ones();
	// INFO: dont mind the values, they get changed on the first iteration anyway
	// state constraints
	// maximal horizontal velocity
	double maxVelHrz = 5;
	// maximal angles for pitch and roll
	double max_angles = 0.3;

	// cost of the soft constraints
	double velSlackCost = 1000;


	// indicate the need to update constraints
	bool reqConstrUpdate = false;

	MatrixXd trajectory;
	double trajTimeStamp;

};
}  // namespace pendulum_controller

}  // namespace pendulum_controller_plugin

#endif
