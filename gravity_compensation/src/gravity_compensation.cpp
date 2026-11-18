#include <gravity_compensation/gravity_compensation.h>

namespace gravity_compensation {

GravityCompensation::GravityCompensation()
    : Base::EffortControllerBase() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GravityCompensation::on_init() {
  const auto ret = Base::on_init();
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }
  auto_declare<std::string>("ft_sensor_ref_link", "");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GravityCompensation::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_configure(previous_state);
  if (ret != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
                 CallbackReturn::SUCCESS) {
    return ret;
  }
    // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link =
      get_node()->get_parameter("ft_sensor_ref_link").as_string();
  RCLCPP_INFO(get_node()->get_logger(), "Finished GravityCompensation "
                                         "on_configure");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GravityCompensation::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  const auto ret = Base::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS) return ret;

  // Update joint states
  Base::updateJointStates();

  m_data_impedance_publisher = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      get_node()->get_name() + std::string("/data_impedance"), 1);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_activate");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
GravityCompensation::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  Base::on_deactivate(previous_state);

  RCLCPP_INFO(get_node()->get_logger(), "Finished Impedance on_deactivate");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensation::update(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
  // Update joint states
  Base::updateJointStates();

  // Compute the torque to applay at the joints
  ctrl::VectorND tau_tot = computeTorque();

  // Saturation of the torque
  if (!Base::computeJointEffortCmds(tau_tot)) return controller_interface::return_type::ERROR;

  // Write final commands to the hardware interface
  Base::writeJointEffortCmds();

  // Compute the forward kinematics
  auto m_current_frame = KDL::Frame();
  Base::m_fk_solver->JntToCart(Base::m_joint_positions, m_current_frame);
  static std_msgs::msg::Float64MultiArray current_orientation_message;
  double x, y, z, w;
  m_current_frame.M.GetQuaternion(x, y, z, w);
  current_orientation_message.data = {
      x, y, z, w
  };
  m_data_impedance_publisher->publish(current_orientation_message);
  return controller_interface::return_type::OK;
}


ctrl::VectorND GravityCompensation::computeTorque() {
  KDL::JntArray tau_gravity(Base::m_joint_number);
  KDL::JntArray tau_coriolis(Base::m_joint_number);
  ctrl::VectorND tau(Base::m_joint_number);

  // Set tau to zero 
  tau.setZero();
  
  if (m_compensate_gravity) {
    Base::m_dyn_solver->JntToGravity(Base::m_joint_positions, tau_gravity);
    tau = tau + tau_gravity.data;
  }
  if (m_compensate_coriolis) {
    Base::m_dyn_solver->JntToCoriolis(Base::m_joint_positions,
                                      Base::m_joint_velocities, tau_coriolis);
    tau = tau + tau_coriolis.data;
  }
  // TODO add nullspace projector
#if DEBUG
  for (int i = 0; i < 7; i++) {
    debug_msg.coriolis_torque[i] = tau_coriolis(i);
  }
  m_data_publisher->publish(debug_msg);
#endif

  return tau;
}
}  // namespace gravity_compensation

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(gravity_compensation::GravityCompensation,
                       controller_interface::ControllerInterface)
