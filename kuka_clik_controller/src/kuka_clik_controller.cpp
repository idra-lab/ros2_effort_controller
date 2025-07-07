#include <fstream>
#include <kuka_clik_controller/kuka_clik_controller.h>
namespace kuka_clik_controller {

KukaClikController::KukaClikController()
    : Base::ControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaClikController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }
  m_max_linear_velocity =
      get_node()->get_parameter("max_linear_velocity").as_double();
  m_max_angular_velocity =
      get_node()->get_parameter("max_angular_velocity").as_double();
  m_max_linear_velocity = std::max(0.0, m_max_linear_velocity);
  m_max_angular_velocity = std::max(0.0, m_max_angular_velocity);
  RCLCPP_WARN(get_node()->get_logger(), "Max linear velocity: %f",
              m_max_linear_velocity);
  RCLCPP_WARN(get_node()->get_logger(), "Max angular velocity: %f",
              m_max_angular_velocity);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaClikController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }
  m_identity =
      ctrl::MatrixND::Identity(Base::m_joint_number, Base::m_joint_number);

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 3,
          std::bind(&KukaClikController::targetFrameCallback, this,
                    std::placeholders::_1));

  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);

#if LOGGING
  XBot::MatLogger2::Options opt;
  opt.default_buffer_size = 5e8; // set default buffer size
  // opt.enable_compression = true;
  RCLCPP_INFO(get_node()->get_logger(), "\n\nCreating logger\n\n");
  m_logger = XBot::MatLogger2::MakeLogger("/tmp/controller_log.mat", opt);
  // m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
  m_logger_appender = XBot::MatAppender::MakeInstance();
  m_logger_appender->add_logger(m_logger);
  m_logger_appender->start_flush_thread();

  // subscribe to lbr_fri_idl/msg/LBRState
  m_state_subscriber =
      get_node()->create_subscription<lbr_fri_idl::msg::LBRState>(
          "/lbr/state", 1,
          std::bind(&KukaClikController::stateCallback, this,
                    std::placeholders::_1));
#endif

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

#if LOGGING
void KukaClikController::stateCallback(
    const lbr_fri_idl::msg::LBRState::SharedPtr state) {
  m_state = *state;
  // m_logger->add("commanded_torque:", state->commanded_torque.data);
  // m_logger->
}
#endif

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaClikController::on_activate(const rclcpp_lifecycle::State &previous_state) {
  Base::on_activate(previous_state);

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);

  // Set the target frame to the current frame
  m_target_frame = m_filtered_frame = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  m_target_joint_position = Base::m_joint_positions.data;

  double roll, pitch, yaw;
  m_current_frame.M.GetRPY(roll, pitch, yaw);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Current frame: " << m_current_frame.p.x() << ", "
                                       << m_current_frame.p.y() << ", "
                                       << m_current_frame.p.z() << ", " << roll
                                       << ", " << pitch << ", " << yaw);
  m_last_time = get_node()->get_clock()->now().seconds();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KukaClikController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  // call logger destructor
#if LOGGING
  RCLCPP_WARN(get_node()->get_logger(), "\n\nFlushing logger\n\n");
  m_logger.reset();
#endif
  // Stop drifting by sending zero joint velocities
  Base::computeJointEffortCmds(ctrl::Vector6D::Zero());
  Base::on_deactivate(previous_state);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
KukaClikController::update(const rclcpp::Time &time,
                           const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  filterTargetFrame();

  computeTargetConfiguration();

  Base::writeJointCmds(m_target_joint_position);

  return controller_interface::return_type::OK;
}

void KukaClikController::filterTargetFrame() {
  if (KDL::Equal(m_target_frame, m_filtered_frame, 1e-3)) {
    return;
  }
  // Compute the time since the last update
  const double current_time = get_node()->get_clock()->now().seconds();
  double dt = current_time - m_last_time;
  m_last_time = current_time;

  // Compute the linear and angular velocity of the target frame
  KDL::Twist target_twist;
  target_twist.vel = (m_target_frame.p - m_filtered_frame.p) / dt;

  KDL::Rotation rot_diff = m_filtered_frame.M.Inverse() * m_target_frame.M;
  double angle;
  KDL::Vector axis;
  KDL::Vector rot_vec = rot_diff.GetRot();
  target_twist.rot = rot_vec / dt;

  // Clamp the linear and angular velocity
  target_twist.vel.x(std::clamp(target_twist.vel.x(), -m_max_linear_velocity,
                                m_max_linear_velocity));
  target_twist.vel.y(std::clamp(target_twist.vel.y(), -m_max_linear_velocity,
                                m_max_linear_velocity));
  target_twist.vel.z(std::clamp(target_twist.vel.z(), -m_max_linear_velocity,
                                m_max_linear_velocity));
  target_twist.rot.x(std::clamp(target_twist.rot.x(), -m_max_angular_velocity,
                                m_max_angular_velocity));
  target_twist.rot.y(std::clamp(target_twist.rot.y(), -m_max_angular_velocity,
                                m_max_angular_velocity));
  target_twist.rot.z(std::clamp(target_twist.rot.z(), -m_max_angular_velocity,
                                m_max_angular_velocity));

  // Integrate the twist to get the new target frame
  m_filtered_frame.p += target_twist.vel * dt;
  angle = target_twist.rot.Norm() * dt;
  if (angle > 1e-6) { // avoid divide-by-zero
    KDL::Vector axis = target_twist.rot / target_twist.rot.Norm();
    m_filtered_frame.M = m_filtered_frame.M * KDL::Rotation::Rot(axis, angle);
  }
}
ctrl::Vector6D KukaClikController::computeMotionError() {
  // Compute the cartesian error between the current and the target frame

  // Transformation from target -> current corresponds to error = target -
  // current
  KDL::Frame error_kdl;
  error_kdl.M = m_target_frame.M * m_current_frame.M.Inverse();
  error_kdl.p = m_target_frame.p - m_current_frame.p;

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle = error_kdl.M.GetRotAngle(rot_axis); // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.

  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle = std::clamp(angle, -max_angle, max_angle);
  distance = std::clamp(distance, -max_distance, max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error.head<3>() << error_kdl.p.x(), error_kdl.p.y(), error_kdl.p.z();
  error.tail<3>() << rot_axis(0), rot_axis(1), rot_axis(2);

  return error;
}

void KukaClikController::computeTargetConfiguration() {
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);
  KDL::Frame simulated_frame = m_current_frame;
  KDL::Jacobian J(Base::m_joint_number);

  KDL::JntArray q_clik(Base::m_joint_number);
  q_clik.data = Base::m_joint_positions.data;

  ctrl::VectorND q_kdl(Base::m_joint_number);

  KDL::JntArray dq(Base::m_joint_number);
  dq.data.setZero();

  // CLIK (closed loop inverse kinematics) algorithm
  ctrl::VectorND ns_task;

  const double ns_gain = 0.05;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err, dq_vec;
  ctrl::VectorND ns_err(Base::m_joint_number), ns_err_old(Base::m_joint_number);
  // Pre-allocate everything outside the loop
  ctrl::MatrixND J_pinv(Base::m_joint_number, 6);
  KDL::Twist delta_twist;
  size_t i = 0, j = 0;
  ns_err.setZero();
  ns_err_old.setZero();

  do {
    // Compute forward kinematics
    Base::m_fk_solver->JntToCart(q_clik, simulated_frame);
    ns_err = Base::m_q_ns.data - q_clik.data;

    // Compute error
    delta_twist = KDL::diff(simulated_frame, m_filtered_frame);
    for (j = 0; j < 6; ++j) {
      err(j) = delta_twist[j];
    }
    // Compute the Jacobian at the current confiation
    Base::m_jnt_to_jac_solver->JntToJac(q_clik, J);
    // Compute the damped pseudo-inverse of the Jacobian
    pseudoInverse(J.data, &J_pinv);
    // 6. Compute dq = J⁺ * err + (I - J⁺J)(q0 - q) / dt
    ns_task = ns_gain * (m_identity - J_pinv * J.data) * ns_err;
    dq.data = J_pinv * err + ns_task; // / DT;

    // Integrate joint velocities (Euler integration)
    for (j = 0; j < Base::m_joint_number; j++) {
      q_clik(j) += dq.data(j) * DT;
    }
    i++;
  } while ((int)i < IT_MAX && err.norm() > EPS);

  const double alpha = 1.0;
  auto filtered_q = alpha * q_clik.data + (1 - alpha) * m_target_joint_position;
  m_target_joint_position = filtered_q;

#if LOGGING
  // if (m_received_target_frame) {
  m_logger->add("time_sec", get_node()->get_clock()->now().seconds());
  m_logger->add("time_nsec", get_node()->get_clock()->now().nanoseconds());
  // add cartesian traj
  m_logger->add("cart_target_x", m_target_frame.p.x());
  m_logger->add("cart_target_y", m_target_frame.p.y());
  m_logger->add("cart_target_z", m_target_frame.p.z());
  Eigen::Matrix<double, 3, 3> rot_des(m_target_frame.M.data);
  m_logger->add("cart_target_M", rot_des);
  m_logger->add("cart_measured_x", m_current_frame.p.x());
  m_logger->add("cart_measured_y", m_current_frame.p.y());
  m_logger->add("cart_measured_z", m_current_frame.p.z());
  Eigen::Matrix<double, 3, 3> M_c(m_current_frame.M.data);
  m_logger->add("cart_measured_M", M_c);
  // solver outcome
  m_logger->add("joint_measured", Base::m_joint_positions.data);
  m_logger->add("joint_target_IK_kdl", q_kdl);
  m_logger->add("joint_target_filtered_kdl", m_target_joint_position);
  m_logger->add("joint_target_clik", q_clik.data);

  std::vector<double> measured_torque(std::begin(m_state.measured_torque),
                                      std::end(m_state.measured_torque));
  m_logger->add("measured_torque", measured_torque);
  std::vector<double> commanded_torque(std::begin(m_state.commanded_torque),
                                       std::end(m_state.commanded_torque));
  m_logger->add("commanded_torque", commanded_torque);
  m_logger->add("ns_task", ns_task);

  auto compute_condition_number = [](const Eigen::MatrixXd &matrix) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
    Eigen::VectorXd singular_values = svd.singularValues();
    return singular_values(0) / singular_values(singular_values.size() - 1);
  };
  m_logger->add("condition_number_jac", compute_condition_number(J.data));

  #endif
}

void KukaClikController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  if (target->header.frame_id != Base::m_robot_base_link) {
    auto &clock = *get_node()->get_clock();
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), clock, 3000,
        "Got target pose in wrong reference frame. Expected: %s but got %s",
        Base::m_robot_base_link.c_str(), target->header.frame_id.c_str());
    return;
  }

  m_target_frame =
      KDL::Frame(KDL::Rotation::Quaternion(
                     target->pose.orientation.x, target->pose.orientation.y,
                     target->pose.orientation.z, target->pose.orientation.w),
                 KDL::Vector(target->pose.position.x, target->pose.position.y,
                             target->pose.position.z));
  m_received_target_frame = true;
}
} // namespace kuka_clik_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(kuka_clik_controller::KukaClikController,
                       controller_interface::ControllerInterface)
