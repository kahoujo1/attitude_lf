#define FOLLOWER_NAME "uav1"
// #define FOLLOWER_NAME "uav36"
#define LEADER_NAME "uav2"
// #define LEADER_NAME "uav40"
#define TESTING false


#include <leader_follower_controller.h>

namespace leader_follower_controller_plugin
{

namespace leader_follower_controller
{

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* //{ initialize() */

bool LeaderFollowerController::initialize(const ros::NodeHandle& nh, std::shared_ptr<mrs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                   std::shared_ptr<mrs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  last_update_time_ = ros::Time(0);

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  bool success = true;

  // FYI
  // This method will load the file using `rosparam get`
  //   Pros: you can the full power of the official param loading
  //   Cons: it is slower
  //
  // Alternatives:
  //   You can load the file directly into the ParamLoader as shown below.

  success *= private_handlers->loadConfigFile(ros::package::getPath("leader_follower_controller_plugin") + "/config/leader_follower_controller.yaml");

  if (!success) {
    return false;
  }

  mrs_lib::ParamLoader param_loader(nh_, "LeaderFollowerController");

  // This is the alternaive way of loading the config file.
  //
  // Files loaded using this method are prioritized over ROS params.
  //
  // param_loader.addYamlFile(ros::package::getPath("leader_follower_tracker_plugin") + "/config/leader_follower_tracker.yaml");

  // MPC LOADING
  param_loader.loadParam("mpc_enable", drs_params_.mpc_enable);

  param_loader.loadParam("cost_Q_pos_vertical", drs_params_.cost_Q_pos_vertical);
  param_loader.loadParam("cost_Q_pos_horizontal", drs_params_.cost_Q_pos_horizontal);
  param_loader.loadParam("cost_Q_velocity_vertical", drs_params_.cost_Q_velocity_vertical);
  param_loader.loadParam("cost_Q_velocity_horizontal", drs_params_.cost_Q_velocity_horizontal);
  param_loader.loadParam("cost_Q_input_reg", drs_params_.cost_Q_input_reg);
  param_loader.loadParam("cost_Q_thrust", drs_params_.cost_Q_thrust);
  
  param_loader.loadParam("cost_R_horizontal", drs_params_.cost_R_horizontal);
  param_loader.loadParam("cost_R_vertical", drs_params_.cost_R_vertical);

  param_loader.loadParam("max_input_angle", drs_params_.max_input_angle);
  param_loader.loadParam("slack_cost", drs_params_.slack_cost);

  param_loader.loadParam("max_vel_horizontal", drs_params_.max_vel_horizontal);

  param_loader.loadParam("max_attitude_ctrl_rate", drs_params_.max_attitude_ctrl_rate);
  param_loader.loadParam("max_thrust_rate", drs_params_.max_thrust_rate);
  
  param_loader.loadParam("attitude_refresh_rate", drs_params_.attitude_refresh_rate);
  param_loader.loadParam("position_refresh_rate", drs_params_.position_refresh_rate);

  param_loader.loadParam("time_decay_alpha", drs_params_.time_decay_alpha);

  param_loader.loadParam("use_reduced_lkf", drs_params_.use_reduced_lkf);

  // Leader estimation parameters ---------------------------------------------------------
  param_loader.loadParam("angle_variance", drs_params_.angle_variance);
  param_loader.loadParam("position_noise", drs_params_.position_noise);

  // LKF LOADING
  
  param_loader.loadParam("lkf_Q_pos_horizontal", drs_params_.lkf_Q_pos_horizontal);
  param_loader.loadParam("lkf_Q_pos_vertical", drs_params_.lkf_Q_pos_vertical);
  param_loader.loadParam("lkf_Q_vel_horizontal", drs_params_.lkf_Q_vel_horizontal);
  param_loader.loadParam("lkf_Q_vel_vertical", drs_params_.lkf_Q_vel_vertical);

  param_loader.loadParam("lkf_Q_attitude", drs_params_.lkf_Q_attitude);

  param_loader.loadParam("lkf_Q_ang_velocity", drs_params_.lkf_Q_ang_velocity);
  param_loader.loadParam("lkf_Q_thrust", drs_params_.lkf_Q_thrust);

  param_loader.loadParam("lkf_Q_ax_offset", drs_params_.lkf_Q_ax_offset);
  param_loader.loadParam("lkf_Q_ay_offset", drs_params_.lkf_Q_ay_offset);
  
  param_loader.loadParam("lkf_R_position", drs_params_.lkf_R_position);
  param_loader.loadParam("lkf_R_attitude", drs_params_.lkf_R_attitude);

  // LKF reduced 
  param_loader.loadParam("lkf_reduced_Q_pos_horizontal", drs_params_.lkf_reduced_Q_pos_horizontal);
  param_loader.loadParam("lkf_reduced_Q_pos_vertical", drs_params_.lkf_reduced_Q_pos_vertical);
  param_loader.loadParam("lkf_reduced_Q_vel_horizontal", drs_params_.lkf_reduced_Q_vel_horizontal);
  param_loader.loadParam("lkf_reduced_Q_vel_vertical", drs_params_.lkf_reduced_Q_vel_vertical);

  param_loader.loadParam("lkf_reduced_Q_attitude", drs_params_.lkf_reduced_Q_attitude);

  param_loader.loadParam("lkf_reduced_Q_ang_velocity", drs_params_.lkf_reduced_Q_ang_velocity);
  param_loader.loadParam("lkf_reduced_Q_thrust", drs_params_.lkf_reduced_Q_thrust);

  param_loader.loadParam("lkf_reduced_Q_ax_offset", drs_params_.lkf_reduced_Q_ax_offset);
  param_loader.loadParam("lkf_reduced_Q_ay_offset", drs_params_.lkf_reduced_Q_ay_offset);
  
  // PID LOADING
  param_loader.loadParam("desired_roll", drs_params_.roll);
  param_loader.loadParam("desired_pitch", drs_params_.pitch);
  param_loader.loadParam("desired_yaw", drs_params_.yaw);
  param_loader.loadParam("desired_thrust_force", drs_params_.force);

  param_loader.loadParam("ctrl_enable", drs_params_.ctrl_enable);
  param_loader.loadParam("hor_Kp", drs_params_.hor_Kp);
  param_loader.loadParam("vert_Kp", drs_params_.vert_Kp);
  param_loader.loadParam("hor_Ki", drs_params_.hor_Ki);
  param_loader.loadParam("vert_Ki", drs_params_.vert_Ki);
  param_loader.loadParam("hor_Kd", drs_params_.hor_Kd);
  param_loader.loadParam("vert_Kd", drs_params_.vert_Kd);
  param_loader.loadParam("hor_sat", drs_params_.hor_sat);
  param_loader.loadParam("vert_sat", drs_params_.vert_sat);
  // offset and follower name parameters:
  double x_offset, y_offset, z_offset;
  nh_.getParam("mrs_uav_managers/control_manager/LeaderFollowerController/offset/x", x_offset);
  nh_.getParam("mrs_uav_managers/control_manager/LeaderFollowerController/offset/y", y_offset);
  nh_.getParam("mrs_uav_managers/control_manager/LeaderFollowerController/offset/z", z_offset);
  offset = Eigen::Vector3d(x_offset, y_offset, z_offset);
  nh_.getParam("mrs_uav_managers/control_manager/LeaderFollowerController/follower_name", follower_name);

  // | ------------------ finish loading params ----------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[LeaderFollowerController]: could not load all parameters!");
    return false;
  }

  // | --------------- dynamic reconfigure server --------------- |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&LeaderFollowerController::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- publishers ----------------------- |
  // LKF 
  ph_lkf_state_ = mrs_lib::PublisherHandler<LKFState>(nh_, "lkf_state", 10, true);
  ph_lkf_reduced_state_ = mrs_lib::PublisherHandler<LKFState>(nh_, "lkf_reduced_state", 10, true);

  ph_lkf_markers_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "prediction_markers", 10, true);
  ph_lkf_covariance_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "prediction_covariance", 10, true);
  ph_lkf_reduced_markers_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "reduced_prediction_markers", 10, true);
  ph_lkf_reduced_covariance_ = mrs_lib::PublisherHandler<visualization_msgs::MarkerArray>(nh_, "reduced_prediction_covariance", 10, true);


  ph_reference_pose_ = mrs_lib::PublisherHandler<geometry_msgs::PoseStamped>(nh_, "reference_pose", 10, true);

  ph_leader_att_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "leader_attitude", 10, true);
  ph_ref_error_ = mrs_lib::PublisherHandler<geometry_msgs::Vector3>(nh_, "reference_error", 10, true);
  ph_control_action_ = mrs_lib::PublisherHandler<ControlAction>(nh_, "control_action", 10, true);
  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "LeaderFollowerController";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_leader_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, std::string("/") + std::string(LEADER_NAME) + "/estimation_manager/odom_main");

  // | ----------------------- finish init ---------------------- |
  UAVModelParameters model_params;
  model_params.m = _uav_mass_;
  ROS_INFO("[LeaderFollowerController]: UAV mass: %.3f kg", model_params.m);
  
  model_ = UAVModel(model_params);
  ROS_INFO("[LeaderFollowerController]: Model initialized");
  mpc = std::make_shared<MPC>(model_);
  ROS_INFO("[LeaderFollowerController]: MPC initialized");
  lkf = std::make_shared<LKF>();
  ROS_INFO("[LeaderFollowerController]: LKF initialized");  

  last_attitude_time = ros::Time::now();
  last_position_time = ros::Time::now();
  leaderPos = geometry_msgs::Point();
  leaderPos.x = 0;
  leaderPos.y = 0;
  leaderPos.z = -1;
  leaderAtt = geometry_msgs::Quaternion();
  leaderAtt.x = 0;
  leaderAtt.y = 0;
  leaderAtt.z = 0;
  leaderAtt.w = 1;
  leader_roll = 0;
  leader_pitch = 0;
  leader_yaw = 0;
  leader_x = 0;
  leader_y = 0;
  leader_z = 0;
  ROS_INFO("-----------------------------------");
  ROS_INFO("[LeaderFollowerController]: initialized");
  ROS_INFO("-----------------------------------");
  is_initialized_ = true;

  return true;
}

//}

/* //{ activate() */

bool LeaderFollowerController::activate(const ControlOutput& last_control_output) {

  activation_control_output_ = last_control_output;
  _uav_mass_ = last_control_output.diagnostics.total_mass;
  first_iteration_ = true;

  is_active_ = true;

  ROS_INFO("[LeaderFollowerController]: activated");

  return true;
}

//}

/* //{ deactivate() */

void LeaderFollowerController::deactivate(void) {

  is_active_       = false;
  first_iteration_ = false;

  ROS_INFO("[LeaderFollowerController]: deactivated");
}

//}

/* updateInactive() //{ */

void LeaderFollowerController::updateInactive(const mrs_msgs::UavState& uav_state, [[maybe_unused]] const std::optional<mrs_msgs::TrackerCommand>& tracker_command) {

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = uav_state.header.stamp;

  first_iteration_ = false;
}

//}

/* //{ updateActive() */
// INFO: This is the function that you can modify
LeaderFollowerController::ControlOutput LeaderFollowerController::updateActive(const mrs_msgs::UavState& uav_state, const mrs_msgs::TrackerCommand& tracker_command) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  mrs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  // clear all the optional parts of the result
  last_control_output_.desired_heading_rate          = {};
  last_control_output_.desired_orientation           = {};
  last_control_output_.desired_unbiased_acceleration = {};
  last_control_output_.control_output                = {};

  if (!is_active_) {
    return last_control_output_;
  }

  // | ---------- calculate dt from the last iteration ---------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (uav_state.header.stamp - last_update_time_).toSec();
  }

  last_update_time_ = uav_state.header.stamp;

  if (fabs(dt) < 0.001) {

    ROS_DEBUG("[LeaderFollowerController]: the last odometry message came too close (%.2f s)!", dt);
    dt = 0.01;
  }

  // update parameters of controllers
  // pidX.setParams(drs_params.hor_Kp, drs_params.hor_Kd, drs_params.hor_Ki, drs_params.hor_sat, -1);
  // pidY.setParams(-drs_params.hor_Kp, -drs_params.hor_Kd, -drs_params.hor_Ki, drs_params.hor_sat, -1);
  // pidZ.setParams(drs_params.vert_Kp, drs_params.vert_Kd, drs_params.vert_Ki, drs_params.vert_sat, -1);

  // get control related data
  // geometry_msgs::Point ctrlRefPos = tracker_command.position;
  // INFO: For now, discard the tracker entirely and use just the leader and follower odometry
  geometry_msgs::Point currPos = uav_state.pose.position;

  // update states of controllers
  // double ctrlElevator = pidX.update(ctrlRefPos.x - currPos.x, dt);
  // double ctrlAileron = pidY.update(ctrlRefPos.y - currPos.y, dt);
  // double ctrlThrust = pidZ.update(ctrlRefPos.z - currPos.z, dt);

  // | ---------------- prepare the control output --------------- |


  mrs_msgs::HwApiAttitudeCmd attitude_cmd;
  if (TESTING) {
    // print current attitude
    ROS_INFO("[LeaderFollowerController]: Current attitude: [%.3f, %.3f, %.3f]\n", mrs_lib::AttitudeConverter(uav_state.pose.orientation).getRoll(), mrs_lib::AttitudeConverter(uav_state.pose.orientation).getPitch(), mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw());
    ROS_INFO("[LeaderFollowerController]: Mass: %.3f, Compensation thrust: %.3f\n",_uav_mass_ ,_uav_mass_* common_handlers_->g);
    static ros::Time start_time = ros::Time::now();
    if ((ros::Time::now() - start_time).toSec() > 1.0) {
      ctrlThrust = 1.0;
    }
    ROS_INFO("[LeaderFollowerController]: Control action: [%.3f, %.3f, %.3f]\n", ctrlRoll, ctrlPitch, ctrlThrust);
    attitude_cmd.orientation = mrs_lib::AttitudeConverter(ctrlRoll, ctrlPitch, 0);
    attitude_cmd.throttle = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,_uav_mass_* common_handlers_->g + ctrlThrust);
    publish_control_action(ctrlRoll, ctrlPitch, ctrlThrust);
  }
  // Feedforward the leader attitude to the follower
  else if (drs_params.mpc_enable){
    current_time = ros::Time::now();
    ROS_INFO("[LeaderFollowerController]: dt value %.3f\n", dt);
    ROS_INFO("[LeaderFollowerController]: Prediction horizon %3d\n", MPC_PRED_HORIZON);
    if (sh_leader_.hasMsg()) {
      // ROS_INFO("[LeaderFollowerController]: Current time: %.5f\n, sh_leader_time %.5f", current_time.toSec(), sh_leader_.getMsg()->header.stamp.toSec());
      
      // ROS_INFO("[LeaderFollowerController]: mass %.5f\n", common_handlers_->getMass());
      // FEEDBACK
      // get the leader position and attitude
      ROS_INFO("[LeaderFollowerController]: Message timeout %.4f\n", current_time.toSec() - sh_leader_.getMsg()->header.stamp.toSec());
      if (abs(sh_leader_.getMsg()->header.stamp.toSec() - current_time.toSec()) < LEADER_MSG_TIMEOUT) {
        leaderPos = sh_leader_.getMsg()->pose.pose.position;
        leaderAtt = sh_leader_.getMsg()->pose.pose.orientation;
      } else {
        ROS_INFO("[LeaderFollowerController]: Leader message timeout %.3f\n", current_time.toSec() - sh_leader_.getMsg()->header.stamp.toSec());
      }
    }
    if (leaderPos.z == -1) {
      ROS_INFO("[LeaderFollowerController]: Leader position not available, using current position");
      leaderPos.x = currPos.x + offset.x();
      leaderPos.y = currPos.y + offset.y();
      leaderPos.z = currPos.z + offset.z();
    }
    // slow down the leader attitude data
    double frequency = drs_params.attitude_refresh_rate;
    if (ros::Time::now() - last_attitude_time > ros::Duration(1.0/frequency)) {
      leader_roll = mrs_lib::AttitudeConverter(leaderAtt).getRoll();
      leader_pitch = mrs_lib::AttitudeConverter(leaderAtt).getPitch();
      leader_yaw = mrs_lib::AttitudeConverter(leaderAtt).getYaw();
      last_attitude_time = ros::Time::now();
    }
    // slow down the leader position data
    frequency = drs_params.position_refresh_rate;
    if (ros::Time::now() - last_position_time > ros::Duration(1.0/frequency)) {
      leader_x = leaderPos.x;
      leader_y = leaderPos.y;
      leader_z = leaderPos.z;
      last_position_time = ros::Time::now();
    }
    publish_leader_attitude(leaderAtt);
    // ----------------------- LKF --------------------------------
    VectorXd y = VectorXd::Zero(LKF_N_OUTPUTS);
    y << leader_x, leader_y, leader_z, leader_roll, leader_pitch, 0;
    // add artificial noise to the leader position
    VectorXd variance = VectorXd::Zero(LKF_N_OUTPUTS);
    variance << drs_params.position_noise, drs_params.position_noise, drs_params.position_noise, drs_params.angle_variance, drs_params.angle_variance, 0;
    y = add_noise(y, variance);
    update_lkf_params(drs_params);
    lkf->update_and_predict(y, dt);
    lkf->update_and_predict_reduced(y.head(3), dt);
    // predict the future states
    double prediction_dt = MPC_TS;
    double prediction_n_steps = MPC_PRED_HORIZON;
    std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> predictions = lkf->predict_future_states_nonlin(prediction_dt, prediction_n_steps);
    std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>> reduced_predictions = lkf->predict_future_states_wrap(prediction_dt, prediction_n_steps, predictions, lkf->x_hat_red);
    // publish LKF
    publish_lkf_state(lkf->x_hat, lkf->P);
    publish_lkf_state(lkf->x_hat_red, lkf->P_red, true);
    publish_lkf_prediction_markers(predictions, prediction_dt);
    publish_lkf_covariance_markers(predictions, prediction_dt);
    publish_lkf_prediction_markers(reduced_predictions, prediction_dt, true);
    publish_lkf_covariance_markers(reduced_predictions, prediction_dt, true);
    // ----------------------- MPC --------------------------------
    // create reference position by offsetting the leader position
    publish_reference_error(leaderPos, currPos);
    publish_reference_pose(leaderPos);

    // update the PID controllers
    VectorXd state = VectorXd::Zero(N_STATES);
    // TODO: make the state update into a function
    // set position
    state(0) = currPos.x;
    state(1) = currPos.y;
    state(2) = currPos.z;
    // set attitude
    state(6) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getRoll();
    state(7) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getPitch();
    state(8) = mrs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
    // ROS_INFO("[LeaderFollowerController]: Current orientation: %.3f, %.3f, %.3f\n", state(5), state(2), state(8));
    // set velocity
    state(3) = uav_state.velocity.linear.x;
    state(4) = uav_state.velocity.linear.y;
    state(5) = uav_state.velocity.linear.z;
    // set thrust
    state(9) = last_ctrl_thrust;
    mpc->updateState(state);
    update_mpc_params(drs_params);
    //---------------- SET REFERENCE -----------------------
    // Vector3d test_ref = Vector3d(-20, -20, 2);
    // mpc->setReferenceConstant(test_ref);
    if (drs_params.use_reduced_lkf) {
      mpc->setReferenceCombined(reduced_predictions, drs_params.time_decay_alpha, dt, offset);
    } else {
      mpc->setReferenceCombined(predictions, drs_params.time_decay_alpha, dt, offset);
    }
    ROS_INFO("[LeaderFollowerController]: Offset: [%.3f, %.3f, %.3f]\n", offset.x(), offset.y(), offset.z());
    ROS_INFO("[LeaderFollowerController]: Follower name: %s\n", follower_name.c_str());
    // ---------------- MPC STEPS --------------------------
    bool MPC_success;
    VectorXd ctrlAct;
    std::optional<VectorXd> optional_result;
    std::tie(ctrlAct, MPC_success, optional_result) = mpc->getControlAction();
    if (MPC_success) {
      ctrlRoll = ctrlAct(0);
      ctrlPitch = ctrlAct(1);
      ctrlThrust = ctrlAct(2);
      last_ctrl_thrust = ctrlThrust;
      ROS_INFO("[LeaderFollowerController]: Control action: [%.3f, %.3f, %.3f]\n", ctrlRoll, ctrlPitch, ctrlThrust);
    }
    else {
      ROS_INFO("[LeaderFollowerController]: MPC failed\n");
    }
    
    // ROS_INFO("[LeaderFollowerController]: gravity compensation: %.3f, ctrlThrust %.3f\n",common_handlers_->getMass() * common_handlers_->g, ctrlThrust);
    attitude_cmd.orientation = mrs_lib::AttitudeConverter(ctrlRoll , ctrlPitch , leader_yaw);
    attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
                                                                               _uav_mass_ * common_handlers_->g + ctrlThrust);
    publish_control_action(ctrlRoll, ctrlPitch, _uav_mass_ * common_handlers_->g + ctrlThrust);
  } else {
    attitude_cmd.orientation = mrs_lib::AttitudeConverter(drs_params.roll, drs_params.pitch, drs_params.yaw);
    attitude_cmd.throttle    = mrs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model,
                                                                                  _uav_mass_ * common_handlers_->g  + drs_params.force);
  }


  // | ----------------- set the control output ----------------- |

  last_control_output_.control_output = attitude_cmd;

  // | --------------- fill in the optional parts --------------- |

  //// it is recommended to fill the optinal parts if you know them

  /// this is used for:
  // * plotting the orientation in the control_refence topic (optional)
  // * checking for attitude control error
  last_control_output_.desired_orientation = mrs_lib::AttitudeConverter(ctrlRoll, ctrlPitch, 0);

  /// IMPORANT
  double ax, ay, az;
  std::tie(ax, ay, az) = calculate_acceleration(ctrlRoll, ctrlPitch, 0, ctrlThrust + _uav_mass_ * common_handlers_->g);
  // The acceleration and heading rate in 3D (expressed in the "fcu" frame of reference) that the UAV will actually undergo due to the control action.
  last_control_output_.desired_unbiased_acceleration = Eigen::Vector3d(ax, ay, az);
  last_control_output_.desired_heading_rate          = 0;

  // | ----------------- fill in the diagnostics ---------------- |

  last_control_output_.diagnostics.controller = "LeaderFollowerController";

  return last_control_output_;
}

//}

/* //{ getStatus() */

const mrs_msgs::ControllerStatus LeaderFollowerController::getStatus() {

  mrs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void LeaderFollowerController::switchOdometrySource([[maybe_unused]] const mrs_msgs::UavState& new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void LeaderFollowerController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr LeaderFollowerController::setConstraints([
    [maybe_unused]] const mrs_msgs::DynamicsConstraintsSrvRequest::ConstPtr& constraints) {

  if (!is_initialized_) {
    return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse());
  }

  mrs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[LeaderFollowerController]: updating constraints");

  mrs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return mrs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new mrs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* //{ callbackDrs() */

void LeaderFollowerController::callbackDrs(leader_follower_controller_plugin::leader_follower_controllerConfig& config, [[maybe_unused]] uint32_t level) {

  mrs_lib::set_mutexed(mutex_drs_params_, config, drs_params_);

  ROS_INFO("[LeaderFollowerController]: dynamic reconfigure params updated");
}

void LeaderFollowerController::update_mpc_params(DrsConfig_t params) {
  // set Q
  MatrixXd Q = MatrixXd::Zero(N_STATES + N_INPUTS, N_STATES + N_INPUTS);
  // horizontal position:
  Q(0,0) = params.cost_Q_pos_horizontal;
  Q(1,1) = params.cost_Q_pos_horizontal;
  // vertical position:
  Q(2,2) = params.cost_Q_pos_vertical;
  // velocity
  Q(3,3) = params.cost_Q_velocity_horizontal;
  Q(4,4) = params.cost_Q_velocity_horizontal;
  Q(5,5) = params.cost_Q_velocity_vertical;
  // input regularization
  Q(10,10) = params.cost_Q_input_reg; // roll
  Q(11,11) = params.cost_Q_input_reg; // pitch
  Q(12,12) = params.cost_Q_thrust;
  // set R
  MatrixXd R = MatrixXd::Zero(N_INPUTS, N_INPUTS);
  R(0, 0) = params.cost_R_horizontal;
  R(1, 1) = params.cost_R_horizontal;
  R(2, 2) = params.cost_R_vertical;
  mpc->setParameter("Q", Q);
  mpc->setParameter("R", R);
  // set constraints
  Vector3d bu = Vector3d(params.max_input_angle, params.max_input_angle, 3*model_.getParams().g*model_.getParams().m);
  mpc->setParameter("bu", bu);

  Vector3d bui = Vector3d(params.max_attitude_ctrl_rate, params.max_attitude_ctrl_rate, params.max_thrust_rate*model_.getParams().g*model_.getParams().m);
  mpc->setParameter("bui", bui);

  mpc->setParameter("Z", params.slack_cost);
  mpc->setParameter("maxVelHrz", params.max_vel_horizontal);
  ROS_INFO("[LeaderFollowerController]: MPC mass %.3f kg", _uav_mass_);
  mpc->updateModel(_uav_mass_);
}

void LeaderFollowerController::update_lkf_params(DrsConfig_t params)
{
  MatrixXd process_noise = MatrixXd::Zero(15,15);
  // position
  process_noise(0, 0) = params.lkf_Q_pos_horizontal;
  process_noise(1, 1) = params.lkf_Q_pos_horizontal;
  process_noise(2, 2) = params.lkf_Q_pos_vertical;
  // velocity
  process_noise(3, 3) = params.lkf_Q_vel_horizontal;
  process_noise(4, 4) = params.lkf_Q_vel_horizontal;
  process_noise(5, 5) = params.lkf_Q_vel_vertical;
  // attitude
  process_noise(6, 6) = params.lkf_Q_attitude;
  process_noise(7, 7) = params.lkf_Q_attitude;
  process_noise(8, 8) = params.lkf_Q_attitude;
  // angular velocity
  process_noise(9, 9) = params.lkf_Q_ang_velocity;
  process_noise(10, 10) = params.lkf_Q_ang_velocity;
  process_noise(11, 11) = params.lkf_Q_ang_velocity;
  // thrust
  process_noise(12, 12) = params.lkf_Q_thrust;
  // acceleration offsets
  process_noise(13, 13) = params.lkf_Q_ax_offset;
  process_noise(14, 14) = params.lkf_Q_ay_offset;

  MatrixXd measurement_noise = MatrixXd::Zero(6,6);
  measurement_noise(0, 0) = params.lkf_R_position;
  measurement_noise(1, 1) = params.lkf_R_position;
  measurement_noise(2, 2) = params.lkf_R_position;
  measurement_noise(3, 3) = params.lkf_R_attitude;
  measurement_noise(4, 4) = params.lkf_R_attitude;
  measurement_noise(5, 5) = params.lkf_R_attitude;
  lkf->set_covariance(process_noise, measurement_noise);

  // Reduced LKF
  // position
  process_noise(0, 0) = params.lkf_reduced_Q_pos_horizontal;
  process_noise(1, 1) = params.lkf_reduced_Q_pos_horizontal;
  process_noise(2, 2) = params.lkf_reduced_Q_pos_vertical;
  // velocity
  process_noise(3, 3) = params.lkf_reduced_Q_vel_horizontal;
  process_noise(4, 4) = params.lkf_reduced_Q_vel_horizontal;
  process_noise(5, 5) = params.lkf_reduced_Q_vel_vertical;
  // attitude
  process_noise(6, 6) = params.lkf_reduced_Q_attitude;
  process_noise(7, 7) = params.lkf_reduced_Q_attitude;
  process_noise(8, 8) = params.lkf_reduced_Q_attitude;
  // angular velocity
  process_noise(9, 9) = params.lkf_reduced_Q_ang_velocity;
  process_noise(10, 10) = params.lkf_reduced_Q_ang_velocity;
  process_noise(11, 11) = params.lkf_reduced_Q_ang_velocity;
  // thrust
  process_noise(12, 12) = params.lkf_reduced_Q_thrust;
  // acceleration offsets
  process_noise(13, 13) = params.lkf_reduced_Q_ax_offset;
  process_noise(14, 14) = params.lkf_reduced_Q_ay_offset;
  lkf->set_reduced_covariance(process_noise, measurement_noise.block(0, 0, 3, 3));
}

void LeaderFollowerController::publish_lkf_prediction_markers(const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& predictions, double dt, bool reduced)
{
  ros::Time now = current_time;
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < predictions.size(); i++){
    visualization_msgs::Marker marker;
    marker.header.frame_id = std::string(FOLLOWER_NAME) + "/world_origin";
    marker.header.stamp = now+ros::Duration((i+1)*dt);
    marker.ns = "lkf_prediction";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = predictions[i].first(0);
    marker.pose.position.y = predictions[i].first(1);
    marker.pose.position.z = predictions[i].first(2);
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    if (reduced) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;  
    } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    markers.markers.push_back(marker);
  }
  if (reduced) {
    ph_lkf_reduced_markers_.publish(markers);
  } else {
    ph_lkf_markers_.publish(markers);
  }
}

void LeaderFollowerController::publish_lkf_covariance_markers(const std::vector<std::pair<Eigen::VectorXd, Eigen::MatrixXd>>& predictions, double dt, bool reduced)
{
  ros::Time now = current_time;
  double cov_scale_factor = 1;
  visualization_msgs::MarkerArray markers;
  for (int i = 0; i < predictions.size(); i++){
    MatrixXd cov = predictions[i].second;
    visualization_msgs::Marker marker;
    marker.header.frame_id = std::string(FOLLOWER_NAME)+"/world_origin";
    marker.header.stamp = now+ros::Duration((i+1)*dt);
    marker.ns = "lkf_covariance";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = predictions[i].first(0);
    marker.pose.position.y = predictions[i].first(1);
    marker.pose.position.z = predictions[i].first(2);
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = std::sqrt(cov(0,0)) * cov_scale_factor;
    marker.scale.y = std::sqrt(cov(1,1)) * cov_scale_factor;
    marker.scale.z = std::sqrt(cov(2,2)) * cov_scale_factor;
    marker.color.a = 0.1;
    if (reduced) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.5;
      marker.color.g = 0.0;
      marker.color.b = 0.3;
    }
    
    markers.markers.push_back(marker);
  }
  if (reduced) {
    ph_lkf_reduced_covariance_.publish(markers);
  } else {
    ph_lkf_covariance_.publish(markers);
  }
}

void LeaderFollowerController::publish_reference_pose(const geometry_msgs::Point ref)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = std::string(FOLLOWER_NAME)+"/world_origin";
  pose.header.stamp = current_time;
  pose.pose.position.x = ref.x - offset.x();
  pose.pose.position.y = ref.y - offset.y();
  pose.pose.position.z = ref.z - offset.z();
  ph_reference_pose_.publish(pose);
}

void LeaderFollowerController::publish_lkf_state(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance, bool reduced)
{
  // state = [x, y, z, vx, vy, vz, roll, pitch, yaw, v_roll, v_pitch, v_yaw, thrust, ax_offset, ay_offset]
  leader_follower_controller_plugin::LKFState msg;
  msg.header.frame_id = std::string(FOLLOWER_NAME)+"/world_origin";
  msg.header.stamp = current_time;

  msg.position.x = state(0);
  msg.position.y = state(1);
  msg.position.z = state(2);

  msg.velocity.x = state(3);
  msg.velocity.y = state(4);
  msg.velocity.z = state(5);

  // acceleration
  double ax,ay,az;
  std::tie(ax, ay, az) = calculate_acceleration(state(6), state(7), state(8), state(12) + LEADER_MASS*common_handlers_->g);
  msg.acceleration.x = ax-state(13);
  msg.acceleration.y = ay-state(14);
  msg.acceleration.z = az;

  msg.attitude.roll = state(6);
  msg.attitude.pitch = state(7);
  msg.attitude.yaw = state(8);

  msg.angular_vel.roll = state(9);
  msg.angular_vel.pitch = state(10);
  msg.angular_vel.yaw = state(11);
  msg.thrust = state(12) + LEADER_MASS*common_handlers_->g;
  msg.ax_offset = state(13);
  msg.ay_offset = state(14);
  
  if (reduced) {
    ph_lkf_reduced_state_.publish(msg);
  } else {
    ph_lkf_state_.publish(msg);
  }
}

void LeaderFollowerController::publish_leader_attitude(geometry_msgs::Quaternion leaderAtt)
{
  double leader_roll_ = mrs_lib::AttitudeConverter(leaderAtt).getRoll();
  double leader_pitch_ = mrs_lib::AttitudeConverter(leaderAtt).getPitch();
  double leader_yaw_ = mrs_lib::AttitudeConverter(leaderAtt).getYaw();
  nav_msgs::Odometry odom;
  odom.header.frame_id = std::string(FOLLOWER_NAME)+"/world_origin";
  odom.header.stamp = current_time;
  odom.pose.pose.position.x = leader_roll_;
  odom.pose.pose.position.y = leader_pitch_;
  odom.pose.pose.position.z = leader_yaw_;
  ph_leader_att_.publish(odom);
}

std::tuple<double, double, double> LeaderFollowerController::calculate_acceleration(double roll, double pitch, double yaw, double T)
{
  double g = 9.81;
  double c_yaw = cos(yaw);
  double s_yaw = sin(yaw);
  double c_roll = cos(roll);
  double s_roll = sin(roll);
  double c_pitch = cos(pitch);
  double s_pitch = sin(pitch);
  double ax = (T/LEADER_MASS) * (c_yaw*c_roll*s_pitch + s_yaw*s_roll);
  double ay = (T/LEADER_MASS) * (s_yaw*c_roll*s_pitch - c_yaw*s_roll);
  double az = (T/LEADER_MASS) * (c_roll*c_pitch) - g;
  std::tuple<double, double, double> acc(ax, ay, az);
  return acc;
}

void LeaderFollowerController::publish_reference_error(const geometry_msgs::Point leader_pos, const geometry_msgs::Point uav_pos)
{
  geometry_msgs::Vector3 ref_error;
  ref_error.x = leader_pos.x - offset.x() - uav_pos.x;
  ref_error.y = leader_pos.y - offset.y() - uav_pos.y;
  ref_error.z = leader_pos.z - offset.z() - uav_pos.z;
  ph_ref_error_.publish(ref_error);
}

void LeaderFollowerController::publish_control_action(double roll, double pitch, double thrust)
{
  leader_follower_controller_plugin::ControlAction msg;
  msg.header.frame_id = std::string(FOLLOWER_NAME)+"/world_origin";
  msg.header.stamp = current_time;
  msg.ref_attitude.roll = roll;
  msg.ref_attitude.pitch = pitch;
  msg.ref_thrust = thrust;
  ph_control_action_.publish(msg);
}
}  // namespace leader_follower_controller

}  // namespace leader_follower_controller_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(leader_follower_controller_plugin::leader_follower_controller::LeaderFollowerController, mrs_uav_managers::Controller)
