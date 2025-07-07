#ifndef KUKA_CLIK_CONTROLLER_H_INCLUDED
#define KUKA_CLIK_CONTROLLER_H_INCLUDED

#include <controller_base/controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#if LOGGING
#include <lbr_fri_idl/msg/lbr_state.hpp>
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif
#define DEBUG 0

namespace kuka_clik_controller {

class KukaClikController
    : public virtual controller_base::ControllerBase {
public:
  KukaClikController();

  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  ctrl::VectorND computeTorque();
  void computeTargetConfiguration();
  void filterMaximumForce();

  using Base = controller_base::ControllerBase;

private:
  ctrl::Vector6D compensateGravity();

  void
  targetFrameCallback(const geometry_msgs::msg::PoseStamped::SharedPtr target);
  ctrl::Vector6D computeMotionError();
  void filterTargetFrame();

#if LOGGING
  void stateCallback(const lbr_fri_idl::msg::LBRState::SharedPtr state);
  lbr_fri_idl::msg::LBRState m_state;
#endif
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      m_target_frame_subscriber;
  rclcpp::Publisher<debug_msg::msg::Debug>::SharedPtr m_data_publisher;
  KDL::Frame m_target_frame, m_filtered_frame;
  ctrl::VectorND m_target_joint_position;
#if LOGGING
  XBot::MatLogger2::Ptr m_logger;
  XBot::MatAppender::Ptr m_logger_appender;
  rclcpp::Subscription<lbr_fri_idl::msg::LBRState>::SharedPtr
      m_state_subscriber;
#endif
  KDL::JntArray m_null_space;
  KDL::Frame m_current_frame;

  ctrl::MatrixND m_identity;
  double m_max_linear_velocity;
  double m_max_angular_velocity;
  bool m_received_target_frame = false;

  double m_last_time;
  // CLIK parameters
  const double EPS = 2e-4;
  const int IT_MAX = 50;
  const double DT = 0.05;

  bool m_hand_frame_control;

};

} // namespace kuka_clik_controller

#endif