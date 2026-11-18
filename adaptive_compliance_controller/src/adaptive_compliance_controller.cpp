// Torque-output adaptation of the adaptive compliance controller by Luca Beber.
// Original: lucabeber/cartesian_controllers, online_estimation, 036c1b6.
// See NOTICE, LICENSE and README for paper equations and porting differences.
#include <adaptive_compliance_controller/adaptive_compliance_controller.h>
#include <limits>

namespace adaptive_compliance_controller {
AdaptiveComplianceController::LifecycleNodeInterface::CallbackReturn
AdaptiveComplianceController::on_init() {
  const auto result=ForceBase::on_init();
  if(result!=CallbackReturn::SUCCESS)return result;
  auto_declare<std::string>("adaptive.mode","constant_force");
  auto_declare<std::vector<double>>("adaptive.stiffness_min",{300.,300.,100.});
  auto_declare<std::vector<double>>("adaptive.stiffness_max",{1000.,1000.,1000.});
  auto_declare<std::vector<double>>("adaptive.damping",{30.,30.,20.,5.,5.,5.});
  auto_declare<std::vector<double>>("adaptive.force_limit",{15.,15.,15.});
  auto_declare<std::vector<double>>("adaptive.q",{3200.,3200.,3200.});
  auto_declare<std::vector<double>>("adaptive.r",{1e-5,1e-5,1e-5});
  auto_declare<double>("adaptive.tank_min",.4);
  auto_declare<double>("adaptive.tank_initial",.5);
  auto_declare<double>("adaptive.tank_max",1.);
  auto_declare<double>("adaptive.power_limit_w",.1);
  auto_declare<double>("adaptive.contact_threshold_n",.5);
  auto_declare<double>("adaptive.desired_penetration_m",.003);
  auto_declare<double>("adaptive.max_penetration_m",.008);
  auto_declare<double>("adaptive.max_period_s",.05);
  auto_declare<std::string>("tissue.map_directory","");
  auto_declare<double>("tissue.surface_z_m",0.);
  auto_declare<double>("tissue.kappa",10000.);
  auto_declare<double>("tissue.viscosity",100.);
  auto_declare<double>("tissue.beta",1.35);
  return CallbackReturn::SUCCESS;
}

AdaptiveComplianceController::LifecycleNodeInterface::CallbackReturn
AdaptiveComplianceController::on_configure(const rclcpp_lifecycle::State &state) {
  const auto result=ForceBase::on_configure(state);
  if(result!=CallbackReturn::SUCCESS)return result;
  try {
    const auto array=[this](const std::string &name,size_t n) {
      const auto values=get_node()->get_parameter(name).as_double_array();
      if(values.size()!=n || !std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);}))
        throw std::runtime_error(name+" requires "+std::to_string(n)+" finite values");
      return values;
    };
    const auto vector3=[&array](const std::string &name) -> Eigen::Vector3d {
      const auto values=array(name,3);return Eigen::Map<const Eigen::Vector3d>(values.data());
    };
    cfg_.k_min=vector3("adaptive.stiffness_min");cfg_.k_max=vector3("adaptive.stiffness_max");
    cfg_.q=vector3("adaptive.q");cfg_.r=vector3("adaptive.r");
    force_limit_=vector3("adaptive.force_limit");
    const auto damp=array("adaptive.damping",6);
    damping_=Eigen::Map<const ctrl::Vector6D>(damp.data());cfg_.damping=damping_.head<3>();
    const auto scalar=[this](const std::string &name) {
      const double value=get_node()->get_parameter(name).as_double();
      if(!std::isfinite(value))throw std::runtime_error(name+" must be finite");
      return value;
    };
    cfg_.tank_min=scalar("adaptive.tank_min");cfg_.tank_initial=scalar("adaptive.tank_initial");
    cfg_.tank_max=scalar("adaptive.tank_max");cfg_.power_limit=scalar("adaptive.power_limit_w");
    depth_=scalar("adaptive.desired_penetration_m");max_depth_=scalar("adaptive.max_penetration_m");
    contact_threshold_=scalar("adaptive.contact_threshold_n");max_period_=scalar("adaptive.max_period_s");
    beta_=scalar("tissue.beta");
    const auto mode=get_node()->get_parameter("adaptive.mode").as_string();
    if(mode!="constant_force" && mode!="variable_force")throw std::runtime_error("Unknown adaptive.mode");
    variable_force_=mode=="variable_force";
    if(!core_.configure(cfg_) || damping_.minCoeff()<0. || force_limit_.minCoeff()<=0. ||
       depth_<0. || max_depth_<=0. || depth_>max_depth_ || contact_threshold_<=0. ||
       max_period_<=0. || beta_<1.)throw std::runtime_error("Invalid adaptive gains, tank or tissue bounds");
    if(get_node()->get_parameter("hybrid_normal_force").as_bool() ||
       get_node()->get_parameter("force_feedback_gain").as_double()!=0. ||
       get_node()->get_parameter("nullspace_stiffness").as_double()!=0. ||
       get_node()->get_parameter("compensate_dJdq").as_bool())
      throw std::runtime_error("Adaptive law requires hybrid=false, force_feedback_gain=0, nullspace_stiffness=0, compensate_dJdq=false");
    const auto directory=get_node()->get_parameter("tissue.map_directory").as_string();
    if(directory.empty())map_.flat({scalar("tissue.surface_z_m"),scalar("tissue.kappa"),scalar("tissue.viscosity")});
    else map_.load(directory);
    auto publisher=get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
        get_node()->get_name()+std::string("/adaptive_state"),1);
    state_publisher_=std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);
    state_publisher_->msg_.data.resize(15,0.);
  } catch(const std::exception &error) {
    RCLCPP_ERROR(get_node()->get_logger(),"Adaptive configure failed: %s",error.what());return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

AdaptiveComplianceController::LifecycleNodeInterface::CallbackReturn
AdaptiveComplianceController::on_activate(const rclcpp_lifecycle::State &state) {
  const auto result=ForceBase::on_activate(state);
  if(result!=CallbackReturn::SUCCESS)return result;
  core_.reset();last_=math::Result();last_.tank=core_.energy();previous_valid_=false;
  return CallbackReturn::SUCCESS;
}

ctrl::Vector6D AdaptiveComplianceController::taskWrench(const ctrl::Vector6D &error,
    const ctrl::Vector6D &velocity,const ctrl::Matrix6D &inertia,
    ctrl::Matrix6D &stiffness,ctrl::Matrix6D &damping) {
  damping=damping_.asDiagonal();
  stiffness.setZero();
  const auto dissipative=[&]() -> ctrl::Vector6D {
    ctrl::Vector6D out=-damping_.cwiseProduct(velocity);
    for(int i=0;i<3;++i)out[i]=std::clamp(out[i],-force_limit_[i],force_limit_[i]);
    return out;
  };
  // Always require fresh compensated FT, including in VS-VF with a zero wrench
  // reference. Invalid clocks/periods never enter energy integration.
  if(!std::isfinite(m_control_period_s) || m_control_period_s<=0. || m_control_period_s>max_period_)
    return ctrl::Vector6D::Constant(std::numeric_limits<double>::quiet_NaN());
  Tissue tissue;const auto &p=currentFrame().p;
  if(inputInhibited() || !map_.sample(p.x(),p.y(),tissue)) {
    previous_valid_=false;last_=math::Result();last_.tank=core_.energy();
    publishState(tissue,0.,0.,false,true);return dissipative();
  }
  if(m_target_wrench.tail<3>().norm()>1e-8 || m_target_wrench[2]>1e-8)
    return ctrl::Vector6D::Constant(std::numeric_limits<double>::quiet_NaN());
  const double surface_velocity=previous_valid_ ? (tissue.surface-previous_surface_)/m_control_period_s : 0.;
  previous_surface_=tissue.surface;previous_valid_=true;
  const double penetration=tissue.surface-p.z();
  const double rate=surface_velocity-velocity[2];
  const bool contact=contactWrench()[2]>=contact_threshold_ && penetration>=0.;
  Eigen::Vector3d desired=Eigen::Vector3d::Zero(),lower=-force_limit_;
  if(contact) {
    desired=m_target_wrench.head<3>();
    const double depth_force=math::huntCrossley(tissue.kappa,tissue.viscosity,beta_,
        variable_force_ ? depth_ : max_depth_,rate);
    if(!std::isfinite(depth_force))return ctrl::Vector6D::Constant(std::numeric_limits<double>::quiet_NaN());
    if(variable_force_)desired[2]=-std::min(depth_force,force_limit_[2]);
    else {lower[2]=std::max(lower[2],-depth_force);desired[2]=std::clamp(desired[2],lower[2],0.);}
  }
  last_=core_.step(error.head<3>(),velocity.head<3>(),desired,lower,force_limit_,m_control_period_s);
  publishState(tissue,penetration,desired[2],contact,!last_.feasible);
  if(!last_.feasible)return dissipative();
  // Fixed base-axis stiffness/damping keep the QP power convention explicit.
  stiffness.topLeftCorner<3,3>()=last_.stiffness.asDiagonal();
  stiffness.bottomRightCorner<3,3>()=m_cartesian_stiffness.bottomRightCorner<3,3>();
  ctrl::Vector6D out;
  out.head<3>()=last_.wrench;
  out.tail<3>()=stiffness.bottomRightCorner<3,3>()*error.tail<3>()-
                  damping_.tail<3>().cwiseProduct(velocity.tail<3>());
  return out;
}

void AdaptiveComplianceController::publishState(const Tissue &t,double penetration,
    double reference,bool contact,bool inhibited) {
  if(!state_publisher_ || !state_publisher_->trylock())return;
  auto &v=state_publisher_->msg_.data;
  v[0]=last_.tank;
  for(int i=0;i<3;++i){v[1+i]=last_.stiffness[i];v[5+i]=last_.wrench[i];}
  v[4]=reference;v[8]=t.kappa;v[9]=t.viscosity;v[10]=penetration;
  v[11]=contact?1.:0.;v[12]=last_.feasible?1.:0.;v[13]=inhibited?1.:0.;v[14]=last_.variable_power;
  state_publisher_->unlockAndPublish();
}
}  // namespace adaptive_compliance_controller
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(adaptive_compliance_controller::AdaptiveComplianceController,
                       controller_interface::ControllerInterface)
