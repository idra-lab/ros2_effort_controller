#ifndef JOINT_IMPEDANCE_CONTROLLER_H_INCLUDED
#define JOINT_IMPEDANCE_CONTROLLER_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <chrono>

namespace joint_impedance_controller {

/**
 * Joint-space spring-damper impedance, with Cartesian pose equilibria mapped
 * to joint references by IK. No FT measurement or wrench command is consumed.
 */
class JointImpedanceController
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  JointImpedanceController();
  using Base = effort_controller_base::EffortControllerBase;

  LifecycleNodeInterface::CallbackReturn on_init() override;
  LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;
  LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::return_type update(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;
  ctrl::VectorND computeTorque();

  ctrl::VectorND m_joint_stiffness;
  ctrl::VectorND m_joint_damping;
  ctrl::VectorND m_q_desired;

 private:
  struct PoseInput {
    geometry_msgs::msg::PoseStamped message;
    std::chrono::steady_clock::time_point received{};
    bool valid = false;
  };
  realtime_tools::RealtimeBuffer<PoseInput> m_pose_input;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_target_frame_subscriber;
  std::chrono::steady_clock::time_point m_activation_time{};
  std::chrono::steady_clock::time_point m_last_processed{};
  double m_command_timeout_s = 0.0;
  bool m_reference_valid = false;
  KDL::JntArray m_ik_candidate;

  void targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
  void readReference();
};

}  // namespace joint_impedance_controller
#endif
