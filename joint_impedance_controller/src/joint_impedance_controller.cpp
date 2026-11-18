#include <joint_impedance_controller/joint_impedance_controller.h>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace joint_impedance_controller {

JointImpedanceController::JointImpedanceController() : Base::EffortControllerBase() {}

JointImpedanceController::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_init() {
  const auto result = Base::on_init();
  if (result != CallbackReturn::SUCCESS) return result;
  auto_declare<double>("command_timeout_s", 0.0);
  // Joint-specific parameters are declared on configure, once the actual
  // actuated chain size is known, including when joints were set after init.
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_configure(const rclcpp_lifecycle::State &previous_state) {
  const auto result = Base::on_configure(previous_state);
  if (result != CallbackReturn::SUCCESS) return result;
  m_command_timeout_s = get_node()->get_parameter("command_timeout_s").as_double();
  if (!std::isfinite(m_command_timeout_s) || m_command_timeout_s < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "command_timeout_s must be finite and nonnegative");
    return CallbackReturn::ERROR;
  }
  m_joint_stiffness.resize(Base::m_joint_number);
  m_joint_damping.resize(Base::m_joint_number);
  m_q_desired = ctrl::VectorND::Zero(Base::m_joint_number);
  m_ik_candidate.resize(Base::m_joint_number);
  for (size_t i = 0; i < Base::m_joint_number; ++i) {
    const auto suffix = std::string(".joint") + std::to_string(i + 1);
    const double stiffness = auto_declare<double>("stiffness" + suffix, 100.0);
    if (!std::isfinite(stiffness) || stiffness < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "stiffness%s must be finite and nonnegative", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    // Retain the legacy default, but allow physical damping to be tuned.
    // 2*sqrt(K) alone is not critical damping for an arbitrary robot inertia.
    const double damping = auto_declare<double>("damping" + suffix, 2.0 * std::sqrt(stiffness));
    if (!std::isfinite(damping) || damping < 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(), "damping%s must be finite and nonnegative", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    m_joint_stiffness[i] = stiffness;
    m_joint_damping[i] = damping;
  }
  m_target_frame_subscriber = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      get_node()->get_name() + std::string("/target_frame"), 1,
      std::bind(&JointImpedanceController::targetFrameCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(), "Joint impedance configured for %zu joints", Base::m_joint_number);
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_activate(const rclcpp_lifecycle::State &previous_state) {
  const auto result = Base::on_activate(previous_state);
  if (result != CallbackReturn::SUCCESS) return result;
  Base::updateJointStates();
  if (!Base::m_joint_positions.data.allFinite() || !Base::m_joint_velocities.data.allFinite()) {
    Base::on_deactivate(previous_state);
    return CallbackReturn::ERROR;
  }
  // Hold the measured joint configuration, including redundant posture, until
  // a new Cartesian command arrives. No IK is needed to hold this equilibrium.
  m_q_desired = Base::m_joint_positions.data;
  m_activation_time = std::chrono::steady_clock::now();
  m_last_processed = m_activation_time;
  m_reference_valid = true;
  return CallbackReturn::SUCCESS;
}

JointImpedanceController::LifecycleNodeInterface::CallbackReturn
JointImpedanceController::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  m_reference_valid = false;
  return Base::on_deactivate(previous_state);
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  Base::updateJointStates();
  if (!Base::m_joint_positions.data.allFinite() || !Base::m_joint_velocities.data.allFinite()) {
    Base::writeZeroEffortCommands();
    return controller_interface::return_type::ERROR;
  }
  ctrl::VectorND torque;
  try {
    torque = computeTorque();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Joint impedance failed: %s", error.what());
    Base::writeZeroEffortCommands();
    return controller_interface::return_type::ERROR;
  }
  // The base checks finite values, applies URDF effort limits and limits the
  // increment before writing. Invalid results clear efforts without terminating.
  if (!Base::computeJointEffortCmds(torque)) return controller_interface::return_type::ERROR;
  Base::writeJointEffortCmds();
  return controller_interface::return_type::OK;
}

void JointImpedanceController::readReference() {
  const auto &input = *m_pose_input.readFromRT();
  const auto now = std::chrono::steady_clock::now();
  const bool valid = input.valid && input.received >= m_activation_time;
  bool fresh = m_command_timeout_s == 0.0;
  if (!fresh && valid) {
    const double receive_age = std::chrono::duration<double>(now - input.received).count();
    const double stamp_age = (get_node()->now() - rclcpp::Time(input.message.header.stamp)).seconds();
    fresh = receive_age <= m_command_timeout_s && stamp_age >= -0.05 && stamp_age <= m_command_timeout_s;
  }
  if (valid && input.received > m_last_processed) {
    // Consume each snapshot once, including failed or expired commands.
    m_last_processed = input.received;
    if (fresh) {
      const auto &p = input.message.pose.position;
      const auto &q = input.message.pose.orientation;
      const KDL::Frame target(KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w),
                              KDL::Vector(p.x, p.y, p.z));
      const int status = Base::m_ik_solver->CartToJnt(Base::m_joint_positions, target, m_ik_candidate);
      m_reference_valid = status >= 0 && m_ik_candidate.data.allFinite();
      if (m_reference_valid) {
        m_q_desired = m_ik_candidate.data;
      } else {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 3000,
                            "IK failed; joint spring inhibited until a new valid pose command");
      }
    }
  }
  if (!fresh) m_reference_valid = false;
  if (!m_reference_valid) {
    // Do not continue pulling toward an obsolete equilibrium after failed IK.
    // Retain damping and configured model compensation; this is not a brake.
    m_q_desired = Base::m_joint_positions.data;
  }
}

ctrl::VectorND JointImpedanceController::computeTorque() {
  const auto invalid = [this]() -> ctrl::VectorND {
    return ctrl::VectorND::Constant(Base::m_joint_number, std::numeric_limits<double>::quiet_NaN());
  };
  if (!Base::m_joint_positions.data.allFinite() || !Base::m_joint_velocities.data.allFinite()) return invalid();
  readReference();
  ctrl::VectorND torque = m_joint_stiffness.cwiseProduct(m_q_desired - Base::m_joint_positions.data) -
                         m_joint_damping.cwiseProduct(Base::m_joint_velocities.data);
  KDL::JntArray compensation(Base::m_joint_number);
  if (Base::m_compensate_gravity) {
    if (Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, compensation) < 0 ||
        !compensation.data.allFinite()) return invalid();
    torque += compensation.data;
  }
  if (Base::m_compensate_coriolis) {
    if (Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions, Base::m_joint_velocities, compensation) < 0 ||
        !compensation.data.allFinite()) return invalid();
    torque += compensation.data;
  }
  return torque;
}

void JointImpedanceController::targetFrameCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr target) {
  const auto &p = target->pose.position;
  const auto &q = target->pose.orientation;
  const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (target->header.frame_id != Base::m_robot_base_link ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-6) return;
  auto normalized = *target;
  normalized.pose.orientation.x /= norm;
  normalized.pose.orientation.y /= norm;
  normalized.pose.orientation.z /= norm;
  normalized.pose.orientation.w /= norm;
  m_pose_input.writeFromNonRT({normalized, std::chrono::steady_clock::now(), true});
}

}  // namespace joint_impedance_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(joint_impedance_controller::JointImpedanceController,
                       controller_interface::ControllerInterface)
