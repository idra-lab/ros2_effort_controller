#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <effort_controller_base/control_math.hpp>

namespace cartesian_impedance_controller {

CartesianImpedanceController::CartesianImpedanceController()
    : Base::EffortControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<double>("nullspace_stiffness", 0.0);
  auto_declare<bool>("compensate_dJdq", false);
  auto_declare<std::vector<double>>("nullspace_desired_configuration",
                                    std::vector<double>());

  constexpr double default_lin_stiff = 500.0;
  constexpr double default_rot_stiff = 50.0;
  auto_declare<double>("stiffness.trans_x", default_lin_stiff);
  auto_declare<double>("stiffness.trans_y", default_lin_stiff);
  auto_declare<double>("stiffness.trans_z", default_lin_stiff);
  auto_declare<double>("stiffness.rot_x", default_rot_stiff);
  auto_declare<double>("stiffness.rot_y", default_rot_stiff);
  auto_declare<double>("stiffness.rot_z", default_rot_stiff);
  auto_declare<double>("max_impedance_force", 70.0);
  auto_declare<double>("max_impedance_torque", 10.0);
  auto_declare<double>("command_timeout_s", 0.0); // zero holds the latest equilibrium indefinitely
  auto_declare<double>("task_inertia_cutoff", 1e-6);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  // Set stiffness
  ctrl::Vector6D tmp;
  tmp[0] = get_node()->get_parameter("stiffness.trans_x").as_double();
  tmp[1] = get_node()->get_parameter("stiffness.trans_y").as_double();
  tmp[2] = get_node()->get_parameter("stiffness.trans_z").as_double();
  tmp[3] = get_node()->get_parameter("stiffness.rot_x").as_double();
  tmp[4] = get_node()->get_parameter("stiffness.rot_y").as_double();
  tmp[5] = get_node()->get_parameter("stiffness.rot_z").as_double();

  if (!tmp.allFinite() || tmp.minCoeff() <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Stiffness values must be finite and positive");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  m_cartesian_stiffness = tmp.asDiagonal();

  // Set damping
  tmp[0] = 2 * sqrt(tmp[0]);
  tmp[1] = 2 * sqrt(tmp[1]);
  tmp[2] = 2 * sqrt(tmp[2]);
  tmp[3] = 2 * sqrt(tmp[3]);
  tmp[4] = 2 * sqrt(tmp[4]);
  tmp[5] = 2 * sqrt(tmp[5]);
  
  m_cartesian_damping = tmp.asDiagonal();

  m_max_impendance_force =
      get_node()->get_parameter("max_impedance_force").as_double();
  m_max_impedance_torque = get_node()->get_parameter("max_impedance_torque").as_double();
  m_command_timeout_s = get_node()->get_parameter("command_timeout_s").as_double();
  m_task_inertia_cutoff = get_node()->get_parameter("task_inertia_cutoff").as_double();
  if (!std::isfinite(m_max_impendance_force) || m_max_impendance_force <= 0.0 ||
      !std::isfinite(m_max_impedance_torque) || m_max_impedance_torque <= 0.0 ||
      !std::isfinite(m_command_timeout_s) || m_command_timeout_s < 0.0 ||
      !std::isfinite(m_task_inertia_cutoff) || m_task_inertia_cutoff <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid impedance limits, timeout or inertia cutoff");
    return CallbackReturn::ERROR;
  }
  // Set nullspace stiffness
  m_null_space_stiffness =
      get_node()->get_parameter("nullspace_stiffness").as_double();
  if (!std::isfinite(m_null_space_stiffness) || m_null_space_stiffness < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "nullspace_stiffness must be finite and nonnegative");
    return CallbackReturn::ERROR;
  }
  m_q_ns = ctrl::VectorND::Zero(Base::m_joint_number);
  if (m_null_space_stiffness > 0.0) {
    // Set nullspace configuration
    std::vector<double> nullspace_config =
        get_node()
            ->get_parameter("nullspace_desired_configuration")
            .as_double_array();
    if (nullspace_config.empty()) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Null space configuration is empty, zeroing null space stiffness");
      m_null_space_stiffness = 0.0;
    } else if (nullspace_config.size() != Base::m_joint_number ||
               !std::all_of(nullspace_config.begin(), nullspace_config.end(),
                            [](double x) { return std::isfinite(x); })) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Null space configuration size does not match joint number: "
                   "%zu != %zu",
                   nullspace_config.size(), Base::m_joint_number);
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
          CallbackReturn::ERROR;
    }
    m_q_ns = ctrl::VectorND::Zero(Base::m_joint_number);
    for (size_t i = 0; i < nullspace_config.size(); ++i) {
      m_q_ns(i) = nullspace_config[i];
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(),
    "Postural task stiffness: " << m_null_space_stiffness
    << " for configuration: "
    << m_q_ns.transpose());
  }
  m_compensate_dJdq = get_node()->get_parameter("compensate_dJdq").as_bool();
  RCLCPP_INFO(get_node()->get_logger(), "Compensate dJdq: %d",
              m_compensate_dJdq);
  // Set nullspace damping
  m_null_space_damping = 2 * sqrt(m_null_space_stiffness);

  // Set the identity matrix with dimension of the joint space
  m_identity = ctrl::MatrixND::Identity(m_joint_number, m_joint_number);
  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 1,
          std::bind(&CartesianImpedanceController::targetFrameCallback, this,
                    std::placeholders::_1));
  m_data_publisher = get_node()->create_publisher<debug_msg::msg::Debug>(
      get_node()->get_name() + std::string("/data"), 1);
  
  m_data_impedance_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      get_node()->get_name() + std::string("/data_impedance"), 1);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) return ret;

  // Update joint states
  Base::updateJointStates();

  // Compute the forward kinematics
  if (!Base::m_joint_positions.data.allFinite() || !Base::m_joint_velocities.data.allFinite() ||
      Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame) < 0) {
    Base::on_deactivate(previous_state);
    return CallbackReturn::ERROR;
  }

  // Set the target frame to the current frame
  m_target_frame = m_current_frame;
  m_target_frame_old = m_current_frame;

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");
  
  m_error_old = ctrl::Vector6D::Zero();
  m_error_dot_old = ctrl::Vector6D::Zero();
  m_target_velocity = ctrl::Vector6D::Zero();  
  m_last_time_target_frame_received = get_node()->now();

  // Ignore pre-activation messages without racing subscription buffer writes.
  m_activation_time = std::chrono::steady_clock::now();
  m_input_inhibited = true;
#if LOGGING
  m_logger = XBot::MatLogger2::MakeLogger("/tmp/cart_impedance_log");
  m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
#endif
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
CartesianImpedanceController::update(const rclcpp::Time &time,
                                     const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  if (!Base::m_joint_positions.data.allFinite() || !Base::m_joint_velocities.data.allFinite()) {
    Base::writeZeroEffortCommands();
    return controller_interface::return_type::ERROR;
  }
  ctrl::VectorND tau_tot;
  try {
    tau_tot = computeTorque();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Impedance computation failed: %s", error.what());
    Base::writeZeroEffortCommands();
    return controller_interface::return_type::ERROR;
  }
  if (!tau_tot.allFinite()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Non-finite impedance torque; stopping controller");
    Base::writeZeroEffortCommands();
    return controller_interface::return_type::ERROR;
  }

  // Saturation of the torque
  if (!Base::computeJointEffortCmds(tau_tot)) return controller_interface::return_type::ERROR;

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianImpedanceController::computeMotionError() {
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
  // Note that this is also the maximal offset that the
  // cartesian_compliance_controller can use to build up a restoring stiffness
  // wrench.
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

ctrl::VectorND CartesianImpedanceController::computeTorque() {
  namespace math = effort_controller_base::control_math;
  const auto invalid = [this]() -> ctrl::VectorND {
    return ctrl::VectorND::Constant(Base::m_joint_number, std::numeric_limits<double>::quiet_NaN());
  };
  const ctrl::VectorND &q = Base::m_joint_positions.data;
  const ctrl::VectorND &q_dot = Base::m_joint_velocities.data;
  if (Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame) < 0 ||
      Base::m_jnt_to_jac_solver->JntToJac(Base::m_joint_positions, Base::m_jacobian) < 0)
    return invalid();
  readInputs();
  const ctrl::MatrixND &jac = Base::m_jacobian.data;
  KDL::JntSpaceInertiaMatrix mass(Base::m_joint_number);
  if (m_dyn_solver->JntToMass(Base::m_joint_positions, mass) < 0) return invalid();
  ctrl::Matrix6D lambda, task_pinv;
  ctrl::MatrixND inverse_mass_jt;
  if (!math::taskInertia(mass.data, jac, m_task_inertia_cutoff,
                         lambda, inverse_mass_jt, task_pinv)) return invalid();

  const ctrl::Vector6D motion_error = computeMotionError();
  ctrl::Matrix6D stiffness = Base::displayInBaseLink(m_cartesian_stiffness, Base::m_end_effector_link);
  ctrl::Matrix6D damping;
  if (!math::modalDamping(lambda, stiffness, std::sqrt(0.5), damping)) return invalid();
  ctrl::VectorND tau = ctrl::VectorND::Zero(Base::m_joint_number);
  ctrl::VectorND null_torque = ctrl::VectorND::Zero(Base::m_joint_number);
  KDL::JntArray coriolis(Base::m_joint_number), gravity(Base::m_joint_number);
  coriolis.data.setZero();
  gravity.data.setZero();
  if (m_compensate_gravity) {
    if (Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, gravity) < 0) return invalid();
    tau += gravity.data;
  }
  if (m_compensate_coriolis) {
    if (Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
                                        Base::m_joint_velocities, coriolis) < 0) return invalid();
    tau += coriolis.data;
  }
  ctrl::Vector6D task_wrench = stiffness * motion_error - damping * (jac * q_dot);
  if (m_compensate_dJdq) {
    KDL::JntArrayVel q_in(Base::m_joint_positions, Base::m_joint_velocities);
    KDL::Twist derivative;
    if (Base::m_jnt_to_jac_dot_solver->JntToJacDot(q_in, derivative) < 0) return invalid();
    ctrl::Vector6D jdot_qdot;
    jdot_qdot << derivative.vel.x(), derivative.vel.y(), derivative.vel.z(),
                 derivative.rot.x(), derivative.rot.y(), derivative.rot.z();
    // x_ddot = J q_ddot + J_dot q_dot: cancellation requires a negative sign.
    task_wrench -= lambda * jdot_qdot;
  }
  if (m_null_space_stiffness > 0.0 && !m_input_inhibited) {
    const ctrl::MatrixND projector = m_identity - jac.transpose() * task_pinv * inverse_mass_jt.transpose();
    null_torque = projector * (m_null_space_stiffness * (m_q_ns-q) - m_null_space_damping*q_dot);
  }
  if (!task_wrench.allFinite() || !tau.allFinite() || !null_torque.allFinite()) return invalid();
  const double force_norm = task_wrench.head<3>().norm();
  const double torque_norm = task_wrench.tail<3>().norm();
  if (force_norm > m_max_impendance_force)
    task_wrench.head<3>() *= m_max_impendance_force / force_norm;
  if (torque_norm > m_max_impedance_torque)
    task_wrench.tail<3>() *= m_max_impedance_torque / torque_norm;
  tau += jac.transpose() * task_wrench + null_torque;
#if DEBUG
  debug_msg::msg::Debug debug;
  const ctrl::VectorND spring_torque = jac.transpose() * stiffness * motion_error;
  const ctrl::VectorND damping_torque = -jac.transpose() * damping * jac * q_dot;
  const size_t count = std::min(Base::m_joint_number, debug.stiffness_torque.size());
  for (size_t i = 0; i < count; ++i) {
    debug.stiffness_torque[i] = spring_torque[i];
    debug.damping_torque[i] = damping_torque[i];
    debug.coriolis_torque[i] = coriolis(i);
    debug.nullspace_torque[i] = null_torque[i];
  }
  for (size_t i = 0; i < 6; ++i) debug.impedance_force[i] = task_wrench[i];
  m_data_publisher->publish(debug);
#endif
#if LOGGING
  m_logger->add("task_wrench", task_wrench);
  m_logger->add("joint_torque", tau);
#endif
  return tau;
}

void CartesianImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto &p = msg->pose.position;
  const auto &q = msg->pose.orientation;
  const double norm = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
  if (msg->header.frame_id != Base::m_robot_base_link ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-6) return;
  auto normalized = *msg;
  normalized.pose.orientation.x /= norm;
  normalized.pose.orientation.y /= norm;
  normalized.pose.orientation.z /= norm;
  normalized.pose.orientation.w /= norm;
  m_pose_input.writeFromNonRT({normalized, std::chrono::steady_clock::now(), true});
}

void CartesianImpedanceController::readInputs() {
  const auto &pose = *m_pose_input.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  const bool valid = pose.valid && pose.received >= m_activation_time;
  if (valid) {
    const auto &p = pose.message.pose.position;
    const auto &q = pose.message.pose.orientation;
    m_target_frame = KDL::Frame(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w),
                               KDL::Vector(p.x, p.y, p.z));
  }
  bool command_ok = m_command_timeout_s == 0.0;
  if (!command_ok && valid) {
    const double received_age = std::chrono::duration<double>(now-pose.received).count();
    const double stamped_age = (get_node()->now()-rclcpp::Time(pose.message.header.stamp)).seconds();
    command_ok = received_age <= m_command_timeout_s &&
                 stamped_age >= -0.05 && stamped_age <= m_command_timeout_s;
  }
  m_input_inhibited = !command_ok;
  if (m_input_inhibited) {
    // Expired references remove the spring and postural task; damping and
    // configured model compensation remain active. No FT input is required.
    m_target_frame = m_current_frame;
  }
}

} // namespace cartesian_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    cartesian_impedance_controller::CartesianImpedanceController,
    controller_interface::ControllerInterface)
