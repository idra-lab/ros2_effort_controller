#pragma once
#include <force_impedance_controller/force_impedance_controller.h>
#include <adaptive_compliance_controller/adaptive_math.hpp>
#include <adaptive_compliance_controller/tissue_map.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace adaptive_compliance_controller {
class AdaptiveComplianceController : public force_impedance_controller::ForceImpedanceController {
 public:
  using ForceBase=force_impedance_controller::ForceImpedanceController;
  using Base=effort_controller_base::EffortControllerBase;
  LifecycleNodeInterface::CallbackReturn on_init() override;
  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  double tankEnergy() const {return core_.energy();}
  const Eigen::Vector3d &adaptiveStiffness() const {return last_.stiffness;}
  bool qpFeasible() const {return last_.feasible;}
 protected:
  double defaultForceFeedbackGain() const override {return 0.;}
  bool requireForceSensor() const override {return true;}
  ctrl::Vector6D taskWrench(const ctrl::Vector6D &error,const ctrl::Vector6D &velocity,
      const ctrl::Matrix6D &inertia,ctrl::Matrix6D &stiffness,ctrl::Matrix6D &damping) override;
 private:
  math::PassiveQP core_;
  math::Config cfg_;
  math::Result last_;
  TissueMap map_;
  ctrl::Vector6D damping_=ctrl::Vector6D::Zero();
  Eigen::Vector3d force_limit_{15.,15.,15.};
  double beta_=1.35,depth_=.003,max_depth_=.008,contact_threshold_=.5,max_period_=.05;
  double previous_surface_=0.;
  bool previous_valid_=false,variable_force_=false;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> state_publisher_;
  void publishState(const Tissue &t,double penetration,double reference,bool contact,bool inhibited);
};
}  // namespace adaptive_compliance_controller
