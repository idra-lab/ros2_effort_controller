#ifndef CARTESIAN_IMPEDANCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <realtime_tools/realtime_buffer.hpp>
#include <chrono>

#define DEBUG 0
#if LOGGING
#include <matlogger2/matlogger2.h>
#endif

namespace cartesian_impedance_controller {

/**
 * Cartesian spring-damper impedance mapped to joint torques with J^T.
 * Accepts pose equilibria; does not subscribe to wrench commands or FT sensors.
 * The force feedback controller is a separate force_impedance_controller plugin.
 */
class CartesianImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  CartesianImpedanceController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ctrl::VectorND computeTorque();

  using Base = effort_controller_base::EffortControllerBase;

  ctrl::Matrix6D m_cartesian_stiffness;
  ctrl::Matrix6D m_cartesian_damping;
  double m_null_space_stiffness;
  double m_null_space_damping;
  double m_max_impendance_force;

 private:
  template <class Message> struct TimedInput {
    Message message = Message();
    std::chrono::steady_clock::time_point received{};
    bool valid = false;
  };
  realtime_tools::RealtimeBuffer<TimedInput<geometry_msgs::msg::PoseStamped>> m_pose_input;
  double m_command_timeout_s = 0.0;
  double m_task_inertia_cutoff = 1e-6;
  bool m_input_inhibited = true;
  std::chrono::steady_clock::time_point m_activation_time{};
  double m_max_impedance_torque = 10.0;
  void readInputs();

  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);
  ctrl::Vector6D computeMotionError();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_data_impedance_publisher;
#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif
  KDL::Frame m_target_frame;
  KDL::Frame m_target_frame_old;
  ctrl::Vector6D m_error_old;
  ctrl::Vector6D m_error_dot_old;
  ctrl::Vector6D m_error_dot_dot_old;

  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_ns; // Null space configuration
  ctrl::Vector6D m_target_velocity;

  double m_vel_old = 0.0;
  double current_acc_j0 = 0.0;
  bool m_compensate_dJdq = false;
  rclcpp::Time m_last_time_target_frame_received;
};

}  // namespace cartesian_impedance_controller

#endif
