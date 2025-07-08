#ifndef GRAVITY_COMPENSATION_H_INCLUDED
#define GRAVITY_COMPENSATION_H_INCLUDED

#include <effort_controller_base/effort_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "controller_interface/controller_interface.hpp"
#include "debug_msg/msg/debug.hpp"
#include "effort_controller_base/Utility.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#define DEBUG 0
namespace gravity_compensation {

class GravityCompensation
    : public virtual effort_controller_base::EffortControllerBase {
 public:
  GravityCompensation();

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


 private:
  ctrl::Vector6D compensateGravity();

  ctrl::Vector6D m_ft_sensor_wrench;
  std::string m_ft_sensor_ref_link;
  KDL::Frame m_ft_sensor_transform;

  ctrl::MatrixND m_identity;
};

}  // namespace gravity_compensation {

#endif