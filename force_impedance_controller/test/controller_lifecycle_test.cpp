#include <force_impedance_controller/force_impedance_controller.h>
#include <array>
#include <iostream>
#include <stdexcept>

using Controller = force_impedance_controller::ForceImpedanceController;
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
      rclcpp::Parameter("ft_sensor_ref_link","link7"), rclcpp::Parameter("joints",joints),
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
    auto load=controller("load_controller",false,0.,{
        rclcpp::Parameter("ft_sensor_topic","/test_ft_driver/wrench"),
        rclcpp::Parameter("sensor_tool_mass",1.),
        rclcpp::Parameter("sensor_tool_com",std::vector<double>{.02,-.01,.08}),
        rclcpp::Parameter("sensor_bias",std::vector<double>{.1,-.2,.3,.01,-.02,.03}),
        rclcpp::Parameter("sensor_force_sign",-1.),
        rclcpp::Parameter("sensor_tare_orientation_xyzw",std::vector<double>{1.,0.,0.,0.})},true);
    check(load->on_configure(previous)==Callback::SUCCESS,"mass, COM, sign, bias and tare loaded from parameters");
    Hardware loaded_hw;
    loaded_hw.assign(*load);
    check(load->on_activate(previous)==Callback::SUCCESS,"load compensation controller activated");
    auto input=std::make_shared<rclcpp::Node>("ft_test_input");
    auto sensor_pub=input->create_publisher<geometry_msgs::msg::WrenchStamped>("/test_ft_driver/wrench",rclcpp::SensorDataQoS());
    auto target_pub=input->create_publisher<geometry_msgs::msg::WrenchStamped>("/load_controller/target_wrench",10);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(input);
    executor.add_node(load->get_node()->get_node_base_interface());
    const auto connection_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while((sensor_pub->get_subscription_count()==0 || target_pub->get_subscription_count()==0) &&
          std::chrono::steady_clock::now()<connection_deadline) {
      executor.spin_some();
      rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    check(sensor_pub->get_subscription_count()>0 && target_pub->get_subscription_count()>0,"direct driver FT topic and target publishers matched without Python relay");
    const auto pump=[&](double contact_force,int cycles) {
      for(int step=0;step<cycles;++step) {
        geometry_msgs::msg::WrenchStamped raw,target;
        raw.header.frame_id="link7";
        target.header.frame_id="base";
        raw.header.stamp=target.header.stamp=input->now();
        // Device sign is reversed, externally tared with gravity +z. At
        // current identity orientation delta weight is -2mg along sensor z.
        raw.wrench.force.x=.1; raw.wrench.force.y=-.2;
        raw.wrench.force.z=contact_force+2*9.80665+.3;
        raw.wrench.torque.x=-2*9.80665*.01+.01;
        raw.wrench.torque.y=-(2*9.80665*.02-.2)-.02;
        raw.wrench.torque.z=.03;
        target.wrench.force.z=2.; target.wrench.torque.y=.2;
        sensor_pub->publish(raw); target_pub->publish(target);
        executor.spin_some();
        if(load->update(input->now(),rclcpp::Duration::from_seconds(.002))!=controller_interface::return_type::OK)
          throw std::runtime_error("load compensation update failed");
        rclcpp::sleep_for(std::chrono::milliseconds(2));
      }
    };
    pump(2.,30);
    check(std::abs(load->m_target_wrench[2]-2.)<1e-12,
          "19.6 N tare/load contribution removed before 4 N contact threshold");
    check(std::all_of(loaded_hw.effort.begin(),loaded_hw.effort.end(),[](double x){return std::abs(x-.2)<1e-9;}),
          "COM moment removed before feedback: only requested 0.2 Nm reaches joints");
    pump(5.,30);
    check(load->m_target_wrench.isZero(),"5 N true contact overload still inhibits force control");
    pump(2.,10);
    check(load->m_target_wrench.isZero(),"compensated contact overload stays latched");
    load->on_deactivate(previous);
    std::cout << count << " lifecycle checks passed" << std::endl;
    rclcpp::shutdown();
    return 0;
  } catch(const std::exception &error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}
