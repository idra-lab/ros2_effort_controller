#include <joint_impedance_controller/joint_impedance_controller.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

using Controller = joint_impedance_controller::JointImpedanceController;
using Callback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
int checks = 0;
void check(bool ok, const char *name) {
  if (!ok) throw std::runtime_error(name);
  ++checks;
  std::cout << "PASS " << name << std::endl;
}

std::string description(size_t n) {
  std::string xml = "<robot name='test'><link name='base'/>";
  for (size_t i = 1; i <= n; ++i) {
    const std::string id = std::to_string(i);
    xml += "<link name='link"+id+"'><inertial><origin xyz='0 0 0.05'/>"
           "<mass value='1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' "
           "ixy='0' ixz='0' iyz='0'/></inertial></link><joint name='joint"+id+
           "' type='"+(i==1 ? "continuous" : "revolute")+"'><parent link='"+
           (i==1 ? "base" : "link"+std::to_string(i-1))+"'/><child link='link"+id+
           "'/><origin xyz='0 0 0.1'/><axis xyz='0 0 1'/>"
           "<limit lower='-3' upper='3' effort='3' velocity='1'/></joint>";
  }
  return xml+"</robot>";
}

// Select the installed ros2_control initialization API (Jazzy or Humble).
template<class T>
auto initializeController(T &c, const std::string &name, const std::string &urdf,
                          const rclcpp::NodeOptions &options, int)
    -> decltype(c.init(name, urdf, 1000, "", options)) {
  return c.init(name, urdf, 1000, "", options);
}
template<class T>
auto initializeController(T &c, const std::string &name, const std::string &,
                          const rclcpp::NodeOptions &options, long)
    -> decltype(c.init(name, "", options)) {
  return c.init(name, "", options);
}

std::unique_ptr<Controller> controller(const std::string &name, size_t n=7,
    const std::vector<rclcpp::Parameter> &extra={}) {
  auto result = std::make_unique<Controller>();
  std::vector<std::string> joints;
  for (size_t i=1;i<=n;++i) joints.push_back("joint"+std::to_string(i));
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  std::vector<rclcpp::Parameter> params{
      rclcpp::Parameter("robot_description",description(n)),
      rclcpp::Parameter("robot_base_link","base"),
      rclcpp::Parameter("end_effector_link","link"+std::to_string(n)),
      rclcpp::Parameter("joints",joints),
      rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"}),
      rclcpp::Parameter("delta_tau_max",.2)};
  params.insert(params.end(),extra.begin(),extra.end());
  options.parameter_overrides(params);
  check(initializeController(*result,name,description(n),options,0)==controller_interface::return_type::OK,"controller initialized");
  return result;
}

struct Hardware {
  std::vector<double> q,dq,effort;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  explicit Hardware(size_t n=7):q(n),dq(n),effort(n) {
    commands.reserve(n);states.reserve(2*n);
    for(size_t i=0;i<n;++i) {
      const auto joint="joint"+std::to_string(i+1);
      commands.emplace_back(joint,"effort",&effort[i]);
      states.emplace_back(joint,"position",&q[i]);
      states.emplace_back(joint,"velocity",&dq[i]);
    }
  }
  void assign(Controller &c) {
    std::vector<hardware_interface::LoanedCommandInterface> cmd;
    std::vector<hardware_interface::LoanedStateInterface> state;
    for(auto &v:commands)cmd.emplace_back(v);
    for(auto &v:states)state.emplace_back(v);
    c.assign_interfaces(std::move(cmd),std::move(state));
  }
  bool zero() const {
    return std::all_of(effort.begin(),effort.end(),[](double v){return std::abs(v)<1e-9;});
  }
};

void step(Controller &c) {
  if(c.update(c.get_node()->now(),rclcpp::Duration::from_seconds(.002))!=controller_interface::return_type::OK)
    throw std::runtime_error("production update failed");
}

struct PoseDriver {
  std::shared_ptr<rclcpp::Node> input;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
  rclcpp::executors::SingleThreadedExecutor executor;
  Controller &c;
  size_t n;
  PoseDriver(Controller &controller,size_t count=7):input(std::make_shared<rclcpp::Node>(
      std::string(controller.get_node()->get_name())+"_publisher")),c(controller),n(count) {
    pub=input->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/"+std::string(c.get_node()->get_name())+"/target_frame",10);
    executor.add_node(input);
    executor.add_node(c.get_node()->get_node_base_interface());
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(pub->get_subscription_count()==0 && std::chrono::steady_clock::now()<deadline) {
      executor.spin_some();rclcpp::sleep_for(std::chrono::milliseconds(5));
    }
    check(pub->get_subscription_count()>0,"pose publisher connected");
  }
  geometry_msgs::msg::PoseStamped pose(double yaw=.14) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id="base";msg.header.stamp=input->now();
    msg.pose.position.z=.1*n;
    msg.pose.orientation.z=std::sin(yaw/2);
    msg.pose.orientation.w=std::cos(yaw/2);
    return msg;
  }
  void send(geometry_msgs::msg::PoseStamped msg,bool update=true,int cycles=12) {
    for(int i=0;i<cycles;++i) {
      pub->publish(msg);executor.spin_some();
      if(update)step(c);
      rclcpp::sleep_for(std::chrono::milliseconds(2));
    }
  }
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    const rclcpp_lifecycle::State state;
    for(size_t n:{size_t(6),size_t(7),size_t(8)}) {
      auto c=controller("size_"+std::to_string(n),n);
      check(c->on_configure(state)==Callback::SUCCESS,"arbitrary chain size configures without FT");
      check(c->m_joint_stiffness.size()==static_cast<int>(n) &&
            c->get_node()->has_parameter("stiffness.joint"+std::to_string(n)),
            "all actual joint gains are declared");
      check(c->m_joint_damping.isApprox(ctrl::VectorND::Constant(n,20.)),"legacy damping default retained");
      Hardware hw(n);hw.assign(*c);
      check(c->on_activate(state)==Callback::SUCCESS,"arbitrary-size hardware activates");
      step(*c);check(hw.zero(),"activation holds actual joint configuration without IK");
      c->on_deactivate(state);
    }
    const std::vector<rclcpp::Parameter> invalid{
      rclcpp::Parameter("stiffness.joint1",-1.),
      rclcpp::Parameter("stiffness.joint1",std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("damping.joint1",-1.),
      rclcpp::Parameter("damping.joint1",std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("command_timeout_s",-1.),
      rclcpp::Parameter("command_timeout_s",std::numeric_limits<double>::quiet_NaN())};
    for(size_t i=0;i<invalid.size();++i) {
      auto c=controller("invalid_"+std::to_string(i),7,{invalid[i]});
      check(c->on_configure(state)==Callback::ERROR,"invalid gain or timeout rejected on configure");
    }
    auto free=controller("free_axis",7,{rclcpp::Parameter("stiffness.joint1",0.),rclcpp::Parameter("damping.joint2",3.)});
    check(free->on_configure(state)==Callback::SUCCESS && free->m_joint_damping[0]==0. && free->m_joint_damping[1]==3.,
          "zero stiffness and explicit finite damping are supported");
    check(free->on_activate(state)==Callback::ERROR,"missing hardware returns activation error");
    check(!free->get_node()->has_parameter("ft_sensor_ref_link") && !free->get_node()->has_parameter("nullspace_stiffness"),
          "unused FT and postural parameters are absent");

    auto hold=controller("redundant_hold",2);
    check(hold->on_configure(state)==Callback::SUCCESS,"redundant hold configured");
    Hardware held(2);held.q[0]=.2;held.q[1]=-.2;held.assign(*hold);
    check(hold->on_activate(state)==Callback::SUCCESS,"redundant configuration activated");
    held.q[0]=.1;held.q[1]=-.1;step(*hold);
    check(std::abs(hold->m_q_desired[0]-.2)<1e-12 && held.effort[0]>0 && held.effort[1]<0,
          "unchanged Cartesian pose cannot erase joint spring in redundant motion");
    std::fill(held.effort.begin(),held.effort.end(),1.);
    check(hold->on_deactivate(state)==Callback::SUCCESS && held.zero(),"deactivation clears every effort directly");

    auto c=controller("joint_pose");
    check(c->on_configure(state)==Callback::SUCCESS,"pose controller configured");
    Hardware hw;hw.assign(*c);
    PoseDriver driver(*c);
    check(driver.input->count_subscribers("/joint_pose/target_wrench")==0,"no silently ignored wrench subscription");
    driver.send(driver.pose(),false);
    check(c->on_activate(state)==Callback::SUCCESS,"pose controller activated");
    step(*c);check(hw.zero(),"pre-activation messages ignored");
    driver.send(driver.pose());
    check(std::abs(c->m_q_desired.sum()-.14)<1e-5,"reachable Cartesian pose solved by checked IK");
    check(std::all_of(hw.effort.begin(),hw.effort.end(),[](double v){return std::abs(v-2.)<1e-4;}),
          "joint spring torques match solved reference");
    const auto accepted=c->m_q_desired;
    auto bad=driver.pose();bad.header.frame_id="wrong";driver.send(bad);
    bad=driver.pose();bad.pose.position.x=std::numeric_limits<double>::quiet_NaN();driver.send(bad);
    bad=driver.pose();bad.pose.orientation.w=bad.pose.orientation.z=0.;driver.send(bad);
    bad=driver.pose();bad.pose.orientation.w=std::numeric_limits<double>::infinity();driver.send(bad);
    check(c->m_q_desired.isApprox(accepted),"wrong-frame and nonfinite/zero-quaternion messages are rejected");
    auto normalized=driver.pose();normalized.pose.orientation.z*=4;normalized.pose.orientation.w*=4;
    driver.send(normalized);
    check(c->m_q_desired.isApprox(accepted,1e-5),"valid nonunit quaternion is normalized");
    hw.q[0]=.01;step(*c);
    check(c->m_q_desired.isApprox(accepted,1e-5),"no per-update IK drift of an accepted joint equilibrium");
    hw.q[0]=0.;
    auto unreachable=driver.pose();unreachable.pose.position.x=100.;driver.send(unreachable);
    check(hw.zero() && c->m_q_desired.isZero(),"failed IK inhibits old joint reference rather than reusing it");
    hw.q[0]=.01;step(*c);
    check(hw.zero(),"failed IK keeps spring inhibited as measured posture changes");
    hw.q[0]=0.;driver.send(driver.pose());
    check(std::abs(c->m_q_desired.sum()-.14)<1e-5,"a new reachable command recovers from IK failure");
    c->on_deactivate(state);hw.assign(*c);
    check(c->on_activate(state)==Callback::SUCCESS,"reactivation reacquires interfaces");
    step(*c);check(hw.zero(),"reactivation clears old equilibrium and ignores buffered pose");
    auto large=driver.pose(.7);driver.send(large,true,1);
    check(std::all_of(hw.effort.begin(),hw.effort.end(),[](double v){return std::abs(v-.2)<1e-6;}),
          "first commanded torque increment respects delta_tau_max");
    driver.send(large,true,30);
    check(std::all_of(hw.effort.begin(),hw.effort.end(),[](double v){return std::abs(v-3.)<1e-4;}),
          "URDF effort limits apply to all joints including continuous joint");
    hw.dq[0]=std::numeric_limits<double>::quiet_NaN();std::fill(hw.effort.begin(),hw.effort.end(),1.);
    check(c->update(c->get_node()->now(),rclcpp::Duration::from_seconds(.002))==controller_interface::return_type::ERROR && hw.zero(),
          "nonfinite measured velocity returns error and clears efforts");
    c->on_deactivate(state);

    auto timed=controller("joint_timeout",7,{rclcpp::Parameter("command_timeout_s",.2)});
    check(timed->on_configure(state)==Callback::SUCCESS,"pose watchdog configured");
    Hardware thw;thw.assign(*timed);
    PoseDriver td(*timed);
    check(timed->on_activate(state)==Callback::SUCCESS,"watchdog controller activated");
    td.send(td.pose());check(thw.effort[0]>0,"fresh pose enables spring");
    auto stale=td.pose();stale.header.stamp=td.input->now()-rclcpp::Duration::from_seconds(1.);td.send(stale);
    check(thw.zero(),"recent receipt cannot hide old ROS timestamp");
    td.send(td.pose());
    rclcpp::sleep_for(std::chrono::milliseconds(220));
    for(int i=0;i<15;++i)step(*timed);
    check(thw.zero(),"receive timeout inhibits joint spring");
    auto future=td.pose();future.header.stamp=td.input->now()+rclcpp::Duration::from_seconds(.1);td.send(future);
    check(thw.zero(),"future-stamped command is inhibited");
    rclcpp::sleep_for(std::chrono::milliseconds(70));step(*timed);
    check(thw.zero(),"future command cannot later activate without a new valid message");
    td.send(td.pose());check(thw.effort[0]>0,"new fresh command resumes spring after timeout");
    auto expired=td.pose();expired.header.stamp=td.input->now()-rclcpp::Duration::from_seconds(1.);td.send(expired);
    thw.dq[0]=.001;step(*timed);
    check(thw.effort[0]<0. && timed->m_q_desired.isZero(),"watchdog inhibits spring while retaining joint damping");
    timed->on_deactivate(state);

    auto nan=controller("invalid_activation");
    check(nan->on_configure(state)==Callback::SUCCESS,"invalid-state activation case configured");
    Hardware nhw;nhw.q[0]=std::numeric_limits<double>::quiet_NaN();nhw.assign(*nan);
    check(nan->on_activate(state)==Callback::ERROR && nhw.zero(),"nonfinite activation state rejects and releases interfaces");
    std::cout<<checks<<" joint controller checks passed"<<std::endl;
    rclcpp::shutdown();return 0;
  } catch(const std::exception &error) {
    std::cerr<<"FAIL "<<error.what()<<std::endl;
    rclcpp::shutdown();return 1;
  }
}
