#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <array>
#include <iostream>
#include <stdexcept>

using Controller = cartesian_impedance_controller::CartesianImpedanceController;
using Callback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
int count = 0;
void check(bool condition, const char *name) {
  if (!condition) throw std::runtime_error(name);
  ++count;
  std::cout << "PASS " << name << std::endl;
}

std::string description(bool y_axis=false) {
  std::string xml = "<robot name='test'><link name='base'/>";
  for (int i=1; i<=7; ++i) {
    const std::string id=std::to_string(i);
    xml += "<link name='link"+id+"'><inertial><origin xyz='0 0 0.05'/>"
           "<mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' "
           "ixy='0' ixz='0' iyz='0'/></inertial></link><joint name='joint"+id+
           "' type='"+(i==1 ? "continuous" : "revolute")+"'><parent link='"+
           (i==1 ? "base" : "link"+std::to_string(i-1))+"'/><child link='link"+id+
           "'/><origin xyz='0 0 0.1'/><axis xyz='"+(y_axis ? "0 1 0" : "0 0 1")+"'/>"
           "<limit lower='-3' upper='3' effort='3' velocity='1'/></joint>";
  }
  return xml+"</robot>";
}

std::unique_ptr<Controller> controller(const std::string &name, bool reverse=false,
                                      double null_stiffness=1.0,
                                      const std::vector<rclcpp::Parameter> &extra={}, bool y_axis=false) {
  auto result=std::make_unique<Controller>();
  std::vector<std::string> joints;
  for(int i=1;i<=7;++i) joints.push_back("joint"+std::to_string(i));
  if(reverse) std::reverse(joints.begin(), joints.end());
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  options.parameter_overrides({rclcpp::Parameter("robot_description",description(y_axis)),
      rclcpp::Parameter("robot_base_link","base"), rclcpp::Parameter("end_effector_link","link7"),
      rclcpp::Parameter("joints",joints),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"}),
      rclcpp::Parameter("delta_tau_max",0.2), rclcpp::Parameter("nullspace_stiffness",null_stiffness)});
  auto overrides=options.parameter_overrides();
  overrides.insert(overrides.end(),extra.begin(),extra.end());
  options.parameter_overrides(overrides);
  check(result->init(name,"",options)==controller_interface::return_type::OK,
        "init uses robot_description parameter without waiting for topic");
  return result;
}

struct Hardware {
  std::array<double,7> q{}, dq{}, effort{};
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  Hardware() {
    commands.reserve(7); states.reserve(14);
    for(int i=0;i<7;++i) {
      const auto joint="joint"+std::to_string(i+1);
      commands.emplace_back(joint,"effort",&effort[i]);
      states.emplace_back(joint,"position",&q[i]);
      states.emplace_back(joint,"velocity",&dq[i]);
    }
  }
  void assign(Controller &controller) {
    std::vector<hardware_interface::LoanedCommandInterface> cmd;
    std::vector<hardware_interface::LoanedStateInterface> state;
    for(auto &c:commands) cmd.emplace_back(c);
    for(auto &s:states) state.emplace_back(s);
    controller.assign_interfaces(std::move(cmd),std::move(state));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc,argv);
  try {
    const rclcpp_lifecycle::State previous;
    auto missing=controller("missing_interfaces");
    check(missing->on_configure(previous)==Callback::SUCCESS && missing->m_null_space_stiffness==0,
          "empty nullspace configuration disables task without out-of-bounds access");
    check(missing->on_activate(previous)==Callback::ERROR,
          "missing hardware interfaces returns activation error without FK access");
    auto wrong=controller("wrong_order",true);
    check(wrong->on_configure(previous)==Callback::ERROR,"wrong joint order is rejected");
    auto invalid=controller("invalid_nullspace",false,-1);
    check(invalid->on_configure(previous)==Callback::ERROR,"negative nullspace stiffness rejected");
    auto active=controller("active_controller");
    check(active->on_configure(previous)==Callback::SUCCESS,"valid seven-joint model configured");
    Hardware hw;
    hw.assign(*active);
    check(active->on_activate(previous)==Callback::SUCCESS,"valid hardware interfaces activate");
    check(active->update(active->get_node()->now(),rclcpp::Duration::from_seconds(.002))==
              controller_interface::return_type::OK,
          "production update handles rank-one Jacobian with seven joints");
    for(double effort:hw.effort) check(std::isfinite(effort),"finite hardware command");
    // Exercise deactivation after a nonzero effort, without the normal rate limiter.
    hw.effort.fill(1.0);
    check(active->on_deactivate(previous)==Callback::SUCCESS &&
          std::all_of(hw.effort.begin(),hw.effort.end(),[](double x){return x==0;}),
          "deactivation writes zero effort to every hardware interface");
    hw.assign(*active);
    check(active->on_activate(previous)==Callback::SUCCESS,"reactivation reacquires interfaces");
    hw.q[0]=std::numeric_limits<double>::quiet_NaN();
    hw.effort.fill(1.0);
    check(active->update(active->get_node()->now(),rclcpp::Duration::from_seconds(.002))==
              controller_interface::return_type::ERROR &&
          std::all_of(hw.effort.begin(),hw.effort.end(),[](double x){return x==0;}),
          "nonfinite measured joint state returns error and clears hardware commands");
    active->on_deactivate(previous);
    auto vanilla=controller("vanilla_controller",false,0.,{
        rclcpp::Parameter("command_timeout_s",0.1)});
    check(vanilla->on_configure(previous)==Callback::SUCCESS,
          "vanilla configures without any FT frame, topic or calibration");
    check(!vanilla->get_node()->has_parameter("ft_sensor_ref_link") &&
          !vanilla->get_node()->has_parameter("hybrid_normal_force") &&
          !vanilla->get_node()->has_parameter("sensor_tool_mass"),
          "vanilla does not declare force-controller parameters");
    Hardware pose_hw;
    pose_hw.assign(*vanilla);
    check(vanilla->on_activate(previous)==Callback::SUCCESS,"pose-only controller activates without FT");
    auto input=std::make_shared<rclcpp::Node>("pose_test_input");
    auto pose_pub=input->create_publisher<geometry_msgs::msg::PoseStamped>("/vanilla_controller/target_frame",10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(input);
    executor.add_node(vanilla->get_node()->get_node_base_interface());
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(pose_pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline) {
      executor.spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    check(pose_pub->get_subscription_count()>0,"vanilla pose topic matches");
    check(input->count_subscribers("/vanilla_controller/target_wrench")==0 &&
          input->count_subscribers("/vanilla_controller/ft_sensor_wrench")==0,
          "vanilla has no target-wrench or FT subscriptions");
    const auto step=[&]() {
      if(vanilla->update(input->now(),rclcpp::Duration::from_seconds(.002))!=controller_interface::return_type::OK)
        throw std::runtime_error("vanilla update failed");
    };
    const auto publish_pose=[&](bool stale=false,bool invalid=false) {
      for(int i=0;i<15;++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id=invalid ? "wrong_frame" : "base";
        pose.header.stamp=stale ? input->now()-rclcpp::Duration::from_seconds(1.) : input->now();
        pose.pose.position.z=.7;
        pose.pose.orientation.z=std::sin(.004/2.);
        pose.pose.orientation.w=std::cos(.004/2.);
        pose_pub->publish(pose);
        executor.spin_some();
        step();
        rclcpp::sleep_for(std::chrono::milliseconds(2));
      }
    };
    step();
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x)<1e-9;}),
          "vanilla starts at measured pose with zero spring effort");
    publish_pose();
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x-.2)<1e-9;}),
          "vanilla retains rotational stiffness without a force target or FT sample");
    publish_pose(true);
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x)<1e-9;}),
          "recently received but old-stamped pose is inhibited");
    publish_pose(false,true);
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x)<1e-9;}),
          "wrong-frame pose cannot restore an expired command");
    publish_pose();
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x-.2)<1e-9;}),
          "fresh pose resumes vanilla impedance");
    rclcpp::sleep_for(std::chrono::milliseconds(120));
    step();
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x)<1e-9;}),
          "pose receive timeout removes spring effort without a wrench watchdog");
    publish_pose();
    vanilla->on_deactivate(previous);
    pose_hw.assign(*vanilla);
    check(vanilla->on_activate(previous)==Callback::SUCCESS,"vanilla reactivates");
    step();
    check(std::all_of(pose_hw.effort.begin(),pose_hw.effort.end(),[](double x){return std::abs(x)<1e-9;}),
          "reactivation ignores the pre-activation pose reference");
    vanilla->on_deactivate(previous);
    std::cout << count << " lifecycle checks passed" << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch(const std::exception &error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
