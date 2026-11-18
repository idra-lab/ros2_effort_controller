#include <force_impedance_controller/force_impedance_controller.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <effort_controller_base/control_math.hpp>

namespace force_impedance_controller {

ForceImpedanceController::ForceImpedanceController()
    : Base::EffortControllerBase(), m_hand_frame_control(true) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceImpedanceController::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<std::string>("ft_sensor_topic", "");
  auto_declare<bool>("hand_frame_control", true);
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
  auto_declare<bool>("hybrid_normal_force", false);
  auto_declare<double>("command_timeout_s", 0.0); // zero retains the legacy pose-hold mode
  auto_declare<double>("sensor_timeout_s", 0.2);
  auto_declare<double>("max_contact_force", 4.0);
  auto_declare<double>("force_feedback_gain", 1.0);
  auto_declare<double>("sensor_tool_mass", 0.0); // zero for precompensated input
  auto_declare<std::vector<double>>("sensor_tool_com", {0.0, 0.0, 0.0});
  auto_declare<std::vector<double>>("sensor_bias", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  auto_declare<double>("sensor_force_sign", 1.0);
  auto_declare<std::vector<double>>("sensor_tare_orientation_xyzw", std::vector<double>());
  auto_declare<double>("task_inertia_cutoff", 1e-6);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceImpedanceController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!Base::robotChainContains(m_ft_sensor_ref_link)) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        m_ft_sensor_ref_link
                            << " is not part of the kinematic chain from "
                            << Base::m_robot_base_link << " to "
                            << Base::m_end_effector_link);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::ERROR;
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
  m_hybrid_normal_force = get_node()->get_parameter("hybrid_normal_force").as_bool();
  m_command_timeout_s = get_node()->get_parameter("command_timeout_s").as_double();
  m_sensor_timeout_s = get_node()->get_parameter("sensor_timeout_s").as_double();
  m_max_contact_force = get_node()->get_parameter("max_contact_force").as_double();
  m_force_feedback_gain = get_node()->get_parameter("force_feedback_gain").as_double();
  m_sensor_tool_mass = get_node()->get_parameter("sensor_tool_mass").as_double();
  m_sensor_force_sign = get_node()->get_parameter("sensor_force_sign").as_double();
  m_task_inertia_cutoff = get_node()->get_parameter("task_inertia_cutoff").as_double();
  const auto com = get_node()->get_parameter("sensor_tool_com").as_double_array();
  const auto bias = get_node()->get_parameter("sensor_bias").as_double_array();
  if (com.size() != 3 || bias.size() != 6 ||
      !std::all_of(com.begin(), com.end(), [](double x) { return std::isfinite(x); }) ||
      !std::all_of(bias.begin(), bias.end(), [](double x) { return std::isfinite(x); })) {
    RCLCPP_ERROR(get_node()->get_logger(), "sensor_tool_com needs 3 finite values; sensor_bias needs 6");
    return CallbackReturn::ERROR;
  }
  m_sensor_tool_com = Eigen::Map<const Eigen::Vector3d>(com.data());
  m_sensor_bias = Eigen::Map<const ctrl::Vector6D>(bias.data());
  if (!std::isfinite(m_max_impendance_force) || m_max_impendance_force <= 0.0 ||
      !std::isfinite(m_max_impedance_torque) || m_max_impedance_torque <= 0.0 ||
      !std::isfinite(m_command_timeout_s) || m_command_timeout_s < 0.0 ||
      !std::isfinite(m_sensor_timeout_s) || m_sensor_timeout_s <= 0.0 ||
      !std::isfinite(m_max_contact_force) || m_max_contact_force <= 0.0 ||
      !std::isfinite(m_force_feedback_gain) || m_force_feedback_gain < 0.0 ||
      !std::isfinite(m_sensor_tool_mass) || m_sensor_tool_mass < 0.0 ||
      (m_sensor_force_sign != 1.0 && m_sensor_force_sign != -1.0) ||
      !std::isfinite(m_task_inertia_cutoff) || m_task_inertia_cutoff <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid force limits or watchdog parameters");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  const auto tare = get_node()->get_parameter("sensor_tare_orientation_xyzw").as_double_array();
  m_sensor_tare_gravity.setZero();
  if (!tare.empty()) {
    if (tare.size() != 4 || !std::all_of(tare.begin(), tare.end(), [](double x) { return std::isfinite(x); })) {
      RCLCPP_ERROR(get_node()->get_logger(), "sensor_tare_orientation_xyzw must be empty or contain 4 finite values");
      return CallbackReturn::ERROR;
    }
    Eigen::Quaterniond orientation(tare[3], tare[0], tare[1], tare[2]);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      RCLCPP_ERROR(get_node()->get_logger(), "Invalid sensor tare orientation quaternion");
      return CallbackReturn::ERROR;
    }
    orientation.normalize();
    m_sensor_tare_gravity = orientation.toRotationMatrix().transpose() *
        Eigen::Vector3d(0, 0, -9.80665*m_sensor_tool_mass);
  }
  RCLCPP_INFO(get_node()->get_logger(), "FT load compensation: mass=%.6f kg, COM=[%.6f %.6f %.6f] m in %s; input %s",
      m_sensor_tool_mass, m_sensor_tool_com.x(), m_sensor_tool_com.y(), m_sensor_tool_com.z(),
      m_ft_sensor_ref_link.c_str(), tare.empty() ? "untared" : "tared at configured orientation");
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
  // Make sure sensor wrenches are interpreted correctly
  // setFtSensorReferenceFrame(Base::m_end_effector_link);

  m_target_wrench_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          get_node()->get_name() + std::string("/target_wrench"), 10,
          std::bind(&ForceImpedanceController::targetWrenchCallback, this,
                    std::placeholders::_1));

  const auto configured_ft_topic = get_node()->get_parameter("ft_sensor_topic").as_string();
  const auto ft_topic = configured_ft_topic.empty()
      ? get_node()->get_name() + std::string("/ft_sensor_wrench") : configured_ft_topic;
  m_ft_sensor_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
          ft_topic, rclcpp::SensorDataQoS(),
          std::bind(&ForceImpedanceController::ftSensorWrenchCallback, this,
                    std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "FT input directly from: %s", ft_topic.c_str());

  m_target_frame_subscriber =
      get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
          get_node()->get_name() + std::string("/target_frame"), 1,
          std::bind(&ForceImpedanceController::targetFrameCallback, this,
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
ForceImpedanceController::on_activate(
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

  m_target_wrench = ctrl::Vector6D::Zero();
  m_ft_sensor_wrench = ctrl::Vector6D::Zero();
  m_contact_fault = false;
  // Ignore pre-activation messages without racing subscription buffer writes.
  m_activation_time = std::chrono::steady_clock::now();
  m_input_inhibited = true;
#if LOGGING
  m_logger = XBot::MatLogger2::MakeLogger("/tmp/force_impedance_log");
  m_logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
#endif
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type
ForceImpedanceController::update(const rclcpp::Time &time,
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

ctrl::Vector6D ForceImpedanceController::computeMotionError() {
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

ctrl::VectorND ForceImpedanceController::computeTorque() {
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
  if (m_hybrid_normal_force && m_target_wrench.head<3>().norm() > 1e-6) {
    const Eigen::Vector3d normal = m_target_wrench.head<3>().normalized();
    ctrl::Matrix6D tangent = ctrl::Matrix6D::Identity();
    tangent.topLeftCorner<3, 3>() -= normal * normal.transpose();
    // Damping remains positive on the force axis, while the normal spring is removed.
    stiffness = tangent * stiffness * tangent;
  }

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
  ctrl::Vector6D task_wrench = stiffness * motion_error - damping * (jac * q_dot) +
      math::forceFeedback(m_target_wrench, m_ft_sensor_wrench,
                          m_force_feedback_gain, m_hybrid_normal_force);
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

namespace {
bool finiteWrench(const geometry_msgs::msg::WrenchStamped &msg) {
  const auto &f = msg.wrench.force;
  const auto &t = msg.wrench.torque;
  return std::isfinite(f.x) && std::isfinite(f.y) && std::isfinite(f.z) &&
         std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z);
}
ctrl::Vector6D wrenchVector(const geometry_msgs::msg::WrenchStamped &msg) {
  ctrl::Vector6D out;
  out << msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
         msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z;
  return out;
}
} // namespace

void ForceImpedanceController::targetWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (!finiteWrench(*msg) || (msg->header.frame_id != Base::m_robot_base_link &&
      !Base::robotChainContains(msg->header.frame_id))) return;
  m_wrench_input.writeFromNonRT({*msg, std::chrono::steady_clock::now(), true});
}

void ForceImpedanceController::ftSensorWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
  if (!finiteWrench(*msg) || (msg->header.frame_id != Base::m_robot_base_link &&
      !Base::robotChainContains(msg->header.frame_id))) return;
  if (msg->header.frame_id != m_ft_sensor_ref_link) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                        "FT input must use ft_sensor_ref_link axes and origin");
    return;
  }
  m_sensor_input.writeFromNonRT({*msg, std::chrono::steady_clock::now(), true});
}

void ForceImpedanceController::targetFrameCallback(
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

ctrl::Vector6D ForceImpedanceController::wrenchInBaseAtTcp(
    const ctrl::Vector6D &wrench, const std::string &frame) {
  // Protocol exception: base-frame messages already refer to the TCP origin.
  // This convention applies to target wrenches. FT input uses the sensor frame
  // and is compensated/shifted separately in readInputs().
  if (frame == Base::m_robot_base_link) return wrench;
  KDL::Frame source;
  if (Base::m_forward_kinematics_solver->JntToCart(Base::m_joint_positions, source, frame) < 0)
    throw std::runtime_error("Cannot transform wrench source frame");
  Eigen::Matrix3d rotation;
  Eigen::Vector3d offset;
  for (int i = 0; i < 3; ++i) {
    offset[i] = source.p[i] - m_current_frame.p[i];
    for (int j = 0; j < 3; ++j) rotation(i,j) = source.M(i,j);
  }
  return effort_controller_base::control_math::wrenchAtTcp(wrench, rotation, offset);
}

void ForceImpedanceController::readInputs() {
  // References avoid copying frame-id strings on each update. The RT buffer's
  // returned snapshot remains stable until the next readFromRT on that buffer.
  const auto &pose = *m_pose_input.readFromRT();
  const auto &wrench = *m_wrench_input.readFromRT();
  const auto &sensor = *m_sensor_input.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  const auto valid = [this](const auto &input) {
    return input.valid && input.received >= m_activation_time;
  };
  const auto fresh = [&valid, now](const auto &input, double timeout) {
    return valid(input) && std::chrono::duration<double>(now-input.received).count() <= timeout;
  };
  const auto stampedFresh = [this, &valid](const auto &input, double timeout) {
    if (!valid(input)) return false;
    const double age = (get_node()->now()-rclcpp::Time(input.message.header.stamp)).seconds();
    return age >= -0.05 && age <= timeout;
  };
  if (valid(pose)) {
    const auto &p = pose.message.pose.position;
    const auto &q = pose.message.pose.orientation;
    m_target_frame = KDL::Frame(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w),
                               KDL::Vector(p.x, p.y, p.z));
  }
  m_target_wrench.setZero();
  if (valid(wrench))
    m_target_wrench = wrenchInBaseAtTcp(wrenchVector(wrench.message), wrench.message.header.frame_id);
  m_ft_sensor_wrench.setZero();
  if (valid(sensor)) {
    KDL::Frame source;
    if (Base::m_forward_kinematics_solver->JntToCart(Base::m_joint_positions,
        source, m_ft_sensor_ref_link) < 0)
      throw std::runtime_error("Cannot evaluate FT sensor orientation");
    Eigen::Matrix3d rotation;
    Eigen::Vector3d offset;
    for (int i=0; i<3; ++i) {
      offset[i] = source.p[i]-m_current_frame.p[i];
      for (int j=0; j<3; ++j) rotation(i,j) = source.M(i,j);
    }
    // Remove the load BEFORE transforming the contact wrench or entering feedback.
    const ctrl::Vector6D corrected = effort_controller_base::control_math::compensateSensorWrench(
        wrenchVector(sensor.message), rotation, m_sensor_tool_mass, m_sensor_tool_com,
        m_sensor_bias, m_sensor_force_sign, m_sensor_tare_gravity);
    if (!corrected.allFinite()) throw std::runtime_error("Non-finite compensated FT wrench");
    m_ft_sensor_wrench = effort_controller_base::control_math::wrenchAtTcp(corrected, rotation, offset);
  }
  const bool commands_ok = m_command_timeout_s == 0.0 ||
      (fresh(pose, m_command_timeout_s) && fresh(wrench, m_command_timeout_s) &&
       stampedFresh(pose, m_command_timeout_s) && stampedFresh(wrench, m_command_timeout_s));
  // Disabling command expiry must not disable sensor freshness or overload checks.
  const bool sensor_ok = fresh(sensor, m_sensor_timeout_s) && stampedFresh(sensor, m_sensor_timeout_s);
  m_input_inhibited = effort_controller_base::control_math::inhibitInput(
      commands_ok, sensor_ok, m_target_wrench, m_ft_sensor_wrench, m_max_contact_force, m_contact_fault);
  if (m_input_inhibited) {
    m_target_wrench.setZero();
    m_target_frame = m_current_frame; // zero spring error; retain damping and gravity
  }
}

} // namespace force_impedance_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    force_impedance_controller::ForceImpedanceController,
    controller_interface::ControllerInterface)
