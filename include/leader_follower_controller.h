#ifndef __LEADER_FOLLOWER_CONTROLLER__
#define __LEADER_FOLLOWER_CONTROLLER__

#include <ros/ros.h>
#include <ros/package.h>

#include <pid.h>
#include "model.h"
#include "mpc_params.h"
#include "controller.h"
#include "lkf.h"
#include "utils.h"
#include "control_def.h"
#include <chrono>

#include <leader_follower_controller_plugin/Attitude.h>
#include <leader_follower_controller_plugin/LKFState.h>
#include <leader_follower_controller_plugin/ControlAction.h>

#include <mrs_uav_managers/controller.h>

#include <dynamic_reconfigure/server.h>
#include <leader_follower_controller_plugin/leader_follower_controllerConfig.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>


namespace leader_follower_controller_plugin
{

namespace leader_follower_controller
{

/* //{ class LeaderFollowerController */

class LeaderFollowerController : public mrs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput& last_control_output);

  void deactivate(void);

  void updateInactive(const mrs_msgs::UavState& uav_state, const std::optional<mrs_msgs::TrackerCommand>& tracker_command);

  ControlOutput updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command);

  const mrs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const mrs_msgs::UavState& new_uav_state);

  void resetDisturbanceEstimators(void);

  const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& cmd);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ------------------------ uav state ----------------------- |

  mrs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                                      mutex_drs_;
  typedef leader_follower_controller_plugin::leader_follower_controllerConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>            Drs_t;
  boost::shared_ptr<Drs_t>                                    drs_;
  void                                                        callbackDrs(leader_follower_controller_plugin::leader_follower_controllerConfig& config, uint32_t level);
  DrsConfig_t                                                 drs_params_;
  std::mutex                                                  mutex_drs_params_;

  // | ----------------------- constraints ---------------------- |

  mrs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;

  // | --------- throttle generation and mass estimation -------- |

  double _uav_mass_;

  // Subscribe Handlers
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_leader_;

  // Publish Handlers
  // references
  mrs_lib::PublisherHandler<geometry_msgs::PoseStamped> ph_reference_pose_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry> ph_leader_att_;
  mrs_lib::PublisherHandler<geometry_msgs::Vector3> ph_ref_error_;
  // LKF publishers
  // state

  // predictions
  mrs_lib::PublisherHandler<LKFState> ph_lkf_state_;
  mrs_lib::PublisherHandler<LKFState> ph_lkf_reduced_state_;

  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_lkf_markers_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_lkf_covariance_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_lkf_reduced_markers_;
  mrs_lib::PublisherHandler<visualization_msgs::MarkerArray> ph_lkf_reduced_covariance_;
  // control_action
  mrs_lib::PublisherHandler<ControlAction> ph_control_action_;
  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  PIDController pidX;
  PIDController pidY;
  PIDController pidZ;

  UAVModel model_;
  std::shared_ptr<MPC> mpc;
  std::shared_ptr<LKF> lkf;

  ros::Time current_time;
  ros::Time last_attitude_time;
  ros::Time last_position_time;

  mpc_params default_params;
  geometry_msgs::Point leaderPos;
  geometry_msgs::Quaternion leaderAtt;
  double leader_roll, leader_pitch, leader_yaw;
  double leader_x, leader_y, leader_z;
  double last_ctrl_thrust = 0;
  double ctrlRoll = 0;
  double ctrlPitch = 0;
  double ctrlThrust = 0;
  void update_mpc_params(DrsConfig_t params);
  void update_lkf_params(DrsConfig_t params);

  // puslishing
  void publish_lkf_state(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance, bool reduced = false);

  void publish_lkf_prediction_markers(const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& predictions, double dt, bool reduced = false);
  void publish_lkf_covariance_markers(const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& predictions, double dt, bool reduced = false);
  void publish_reference_pose(const geometry_msgs::Point ref);

  void publish_control_action(double roll, double pitch, double thrust);
  void publish_leader_attitude(geometry_msgs::Quaternion leaderAtt);
  void publish_reference_error(const geometry_msgs::Point leader_pos, const geometry_msgs::Point uav_pos);

  std::tuple<double, double, double> calculate_acceleration(double roll, double pitch, double yaw, double thrust);
};

}  // namespace leader_follower_controller

}  // namespace leader_follower_controller_plugin
 
#endif // !__LEADER_FOLLOWER_CONTROLLER__
