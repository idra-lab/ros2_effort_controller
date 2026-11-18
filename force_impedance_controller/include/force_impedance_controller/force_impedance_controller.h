#ifndef FORCE_IMPEDANCE_CONTROLLER_H_INCLUDED
#define FORCE_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <realtime_tools/realtime_buffer.hpp>
#include <chrono>

#define DEBUG 0
#if LOGGING
#include <matlogger2/matlogger2.h>
#endif

namespace force_impedance_controller {

/**
 * Cartesian impedance with FT feedback, load compensation and optional hybrid
 * normal-force control. Sensor calibration and feedback run in this C++ plugin.
 */
class ForceImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  ForceImpedanceController();

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
  ctrl::Vector6D m_target_wrench;

 protected:
  // Extension point for torque-based task laws. Input parsing, compensated FT,
  // watchdogs and final effort limits stay in this shared implementation.
  virtual ctrl::Vector6D taskWrench(const ctrl::Vector6D &error,
      const ctrl::Vector6D &velocity, const ctrl::Matrix6D &inertia,
      ctrl::Matrix6D &stiffness, ctrl::Matrix6D &damping);
  virtual double defaultForceFeedbackGain() const { return 1.0; }
  virtual bool requireForceSensor() const { return false; }
  bool inputInhibited() const { return m_input_inhibited; }
  const KDL::Frame &currentFrame() const { return m_current_frame; }
  const ctrl::Vector6D &contactWrench() const { return m_ft_sensor_wrench; }
  double m_control_period_s = 0.0;

 private:
  template <class Message> struct TimedInput {
    Message message = Message();
    std::chrono::steady_clock::time_point received{};
    bool valid = false;
  };
  realtime_tools::RealtimeBuffer<TimedInput<geometry_msgs::msg::PoseStamped>> m_pose_input;
  realtime_tools::RealtimeBuffer<TimedInput<geometry_msgs::msg::WrenchStamped>> m_wrench_input;
  realtime_tools::RealtimeBuffer<TimedInput<geometry_msgs::msg::WrenchStamped>> m_sensor_input;
  bool m_hybrid_normal_force = false;
  bool m_contact_fault = false;
  double m_command_timeout_s = 0.0;
  double m_sensor_timeout_s = 0.2;
  double m_max_contact_force = 4.0;
  double m_force_feedback_gain = 1.0;
  double m_sensor_tool_mass = 0.0;
  Eigen::Vector3d m_sensor_tool_com = Eigen::Vector3d::Zero();
  ctrl::Vector6D m_sensor_bias = ctrl::Vector6D::Zero();
  double m_sensor_force_sign = 1.0;
  Eigen::Vector3d m_sensor_tare_gravity = Eigen::Vector3d::Zero();
  double m_task_inertia_cutoff = 1e-6;
  bool m_input_inhibited = true;
  std::chrono::steady_clock::time_point m_activation_time{};
  double m_max_impedance_torque = 10.0;
  void readInputs();
  ctrl::Vector6D wrenchInBaseAtTcp(const ctrl::Vector6D &wrench, const std::string &frame);

  void targetWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void targetFrameCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr target);
  ctrl::Vector6D computeMotionError();

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      m_ft_sensor_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_data_impedance_publisher;
#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
#endif
  KDL::Frame m_target_frame;
  KDL::Frame m_target_frame_old;
  ctrl::Vector6D m_ft_sensor_wrench;
  ctrl::Vector6D m_error_old;
  ctrl::Vector6D m_error_dot_old;
  ctrl::Vector6D m_error_dot_dot_old;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  ctrl::VectorND m_q_ns; // Null space configuration
  ctrl::Vector6D m_target_velocity;

  double m_vel_old = 0.0;
  double current_acc_j0 = 0.0;
  bool m_compensate_dJdq = false;
  /**
   * Allow users to choose whether to specify their target wrenches in the
   * end-effector frame (= True) or the base frame (= False). The first one
   * is easier for explicit task programming, while the second one is more
   * intuitive for tele-manipulation.
   */
  bool m_hand_frame_control;

  rclcpp::Time m_last_time_target_frame_received;
};

}  // namespace force_impedance_controller

#endif
