#include <adaptive_compliance_controller/adaptive_compliance_controller.h>
#include <pluginlib/class_loader.hpp>
#include <array>
#include <iostream>
#include <stdexcept>
using C=adaptive_compliance_controller::AdaptiveComplianceController;
using Callback=rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
int checks=0;
void check(bool value,const char *name){if(!value)throw std::runtime_error(name);++checks;std::cout<<"PASS "<<name<<std::endl;}
std::string model(){
  std::string xml="<robot name='synthetic'><link name='base'/>";
  const std::array<std::string,3> axes{"1 0 0","0 1 0","0 0 1"};
  for(int i=1;i<=3;++i){const auto id=std::to_string(i);
    xml+="<link name='link"+id+"'><inertial><mass value='1'/><inertia ixx='.01' iyy='.01' izz='.01' ixy='0' ixz='0' iyz='0'/></inertial></link>"
         "<joint name='joint"+id+"' type='prismatic'><parent link='"+(i==1?"base":"link"+std::to_string(i-1))+"'/><child link='link"+id+
         "'/><axis xyz='"+axes[i-1]+"'/><limit lower='-2' upper='2' effort='50' velocity='1'/></joint>";
  }return xml+"</robot>";
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

std::unique_ptr<C> controller(const std::string &name,const std::vector<rclcpp::Parameter>&extra={}){
  auto c=std::make_unique<C>();rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  std::vector<rclcpp::Parameter> p{
    rclcpp::Parameter("robot_description",model()),rclcpp::Parameter("robot_base_link","base"),
    rclcpp::Parameter("end_effector_link","link3"),rclcpp::Parameter("ft_sensor_ref_link","link3"),
    rclcpp::Parameter("joints",std::vector<std::string>{"joint1","joint2","joint3"}),
    rclcpp::Parameter("state_interfaces",std::vector<std::string>{"position","velocity"}),
    rclcpp::Parameter("delta_tau_max",.2),rclcpp::Parameter("sensor_tool_mass",.2),
    rclcpp::Parameter("adaptive.stiffness_min",std::vector<double>{100.,100.,100.}),
    rclcpp::Parameter("adaptive.damping",std::vector<double>{0.,0.,0.,0.,0.,0.})};
  p.insert(p.end(),extra.begin(),extra.end());options.parameter_overrides(p);
  check(initializeController(*c,name,model(),options,0)==controller_interface::return_type::OK,"adaptive plugin initialized");return c;
}
struct Hardware{
  std::array<double,3>q{0.,0.,-.01},v{},effort{};
  std::vector<hardware_interface::CommandInterface>commands;
  std::vector<hardware_interface::StateInterface>states;
  Hardware(){commands.reserve(3);states.reserve(6);for(int i=0;i<3;++i){const auto name="joint"+std::to_string(i+1);
    commands.emplace_back(name,"effort",&effort[i]);states.emplace_back(name,"position",&q[i]);states.emplace_back(name,"velocity",&v[i]);}}
  void assign(C &c){std::vector<hardware_interface::LoanedCommandInterface>a;std::vector<hardware_interface::LoanedStateInterface>b;
    for(auto &e:commands)a.emplace_back(e);
    for(auto &e:states)b.emplace_back(e);
    c.assign_interfaces(std::move(a),std::move(b));}
  bool zero()const{return std::all_of(effort.begin(),effort.end(),[](double x){return std::abs(x)<1e-10;});}
};
struct Driver{
  C&c;std::shared_ptr<rclcpp::Node>node;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force,sensor;
  rclcpp::executors::SingleThreadedExecutor exec;
  explicit Driver(C&control):c(control),node(std::make_shared<rclcpp::Node>(std::string(c.get_node()->get_name())+"_inputs")){
    const auto prefix="/"+std::string(c.get_node()->get_name());
    pose=node->create_publisher<geometry_msgs::msg::PoseStamped>(prefix+"/target_frame",10);
    force=node->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix+"/target_wrench",10);
    sensor=node->create_publisher<geometry_msgs::msg::WrenchStamped>(prefix+"/ft_sensor_wrench",rclcpp::SensorDataQoS());
    exec.add_node(node);exec.add_node(c.get_node()->get_node_base_interface());
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while((!pose->get_subscription_count()||!force->get_subscription_count()||!sensor->get_subscription_count())&&std::chrono::steady_clock::now()<end){exec.spin_some();rclcpp::sleep_for(std::chrono::milliseconds(5));}
    check(pose->get_subscription_count()&&force->get_subscription_count()&&sensor->get_subscription_count(),"pose/wrench/direct sensor topics connected");
  }
  void pump(double contact=2.,double target=-2.,bool update=true,int count=20){for(int i=0;i<count;++i){
    geometry_msgs::msg::PoseStamped p;geometry_msgs::msg::WrenchStamped w,s;
    p.header.frame_id=w.header.frame_id="base";s.header.frame_id="link3";
    p.header.stamp=w.header.stamp=s.header.stamp=node->now();
    p.pose.position.z=-.02;p.pose.orientation.w=1.;w.wrench.force.z=target;
    s.wrench.force.z=contact-.2*9.80665;
    pose->publish(p);force->publish(w);sensor->publish(s);exec.spin_some();
    if(update && c.update(node->now(),rclcpp::Duration::from_seconds(.002))!=controller_interface::return_type::OK)throw std::runtime_error("adaptive update failed");
    rclcpp::sleep_for(std::chrono::milliseconds(2));
  }}
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{
  const rclcpp_lifecycle::State state;
  pluginlib::ClassLoader<controller_interface::ControllerInterface>loader("controller_interface","controller_interface::ControllerInterface");
  auto instance=loader.createSharedInstance("adaptive_compliance_controller/AdaptiveComplianceController");
  check(std::dynamic_pointer_cast<C>(instance)!=nullptr,"new torque plugin discovered and loaded through pluginlib");
  std::vector<rclcpp::Parameter>invalid{
    rclcpp::Parameter("adaptive.tank_initial",.1),rclcpp::Parameter("adaptive.mode","wrong"),
    rclcpp::Parameter("adaptive.stiffness_min",std::vector<double>{-1.,100.,100.}),
    rclcpp::Parameter("adaptive.desired_penetration_m",1.),rclcpp::Parameter("force_feedback_gain",1.),
    rclcpp::Parameter("hybrid_normal_force",true),rclcpp::Parameter("compensate_dJdq",true),
    rclcpp::Parameter("tissue.map_directory","/nonexistent/adaptive_map")};
  for(size_t i=0;i<invalid.size();++i){auto c=controller("invalid_adaptive_"+std::to_string(i),{invalid[i]});
    check(c->on_configure(state)==Callback::ERROR,"invalid/mixed adaptive configuration rejected");}
  auto c=controller("adaptive_cf");check(c->on_configure(state)==Callback::SUCCESS,"constant-force configuration accepted");
  check(c->get_node()->get_parameter("force_feedback_gain").as_double()==0.,"no unbudgeted proportional force feedback enabled");
  Hardware hw;hw.assign(*c);Driver driver(*c);driver.pump(2.,-2.,false);
  check(c->on_activate(state)==Callback::SUCCESS,"torque hardware activated");
  check(c->update(c->get_node()->now(),rclcpp::Duration::from_seconds(.002))==controller_interface::return_type::OK&&hw.zero()&&!c->qpFeasible(),"pre-activation inputs ignored and no FT means damping-only");
  driver.pump();
  check(c->qpFeasible()&&std::abs(c->adaptiveStiffness()[2]-200.)<.1,"VS-CF adapts stiffness to requested contact force after sensor load compensation");
  check(std::abs(hw.effort[2]+2.)<.002&&std::abs(hw.effort[0])<1e-9,"Cartesian wrench maps directly to joint effort through Jacobian transpose");
  check(std::abs(c->tankEnergy()-.5)<1e-12,"static contact does not consume tank");
  hw.v[2]=-.01;driver.pump(2.,-2.,true,50);hw.v[2]=0.;
  check(c->tankEnergy()<.5&&c->tankEnergy()>=.4,"measured velocity drives tank power accounting");
  driver.pump(5.);check(!c->qpFeasible(),"compensated contact overload inhibits adaptation");
  driver.pump(2.);check(!c->qpFeasible(),"contact overload remains latched");
  c->on_deactivate(state);hw.assign(*c);check(c->on_activate(state)==Callback::SUCCESS,"reactivation clears fault");
  driver.pump();check(c->qpFeasible()&&std::abs(c->tankEnergy()-.5)<1e-8,"reactivation resets tank and resumes only with fresh input");
  check(c->update(c->get_node()->now(),rclcpp::Duration::from_seconds(0.))==controller_interface::return_type::ERROR&&hw.zero(),"invalid period fails without integrating energy and clears joint effort");
  c->on_deactivate(state);
  auto vf=controller("adaptive_vf",{rclcpp::Parameter("adaptive.mode","variable_force")});
  check(vf->on_configure(state)==Callback::SUCCESS,"variable-force configuration accepted");Hardware vhw;vhw.assign(*vf);Driver vd(*vf);
  check(vf->on_activate(state)==Callback::SUCCESS,"variable-force controller activated");vd.pump(2.,0.);
  check(vf->qpFeasible()&&vf->adaptiveStiffness()[2]>300.,"VS-VF derives force from HC parameters and desired depth");
  rclcpp::sleep_for(std::chrono::milliseconds(220));
  check(vf->update(vf->get_node()->now(),rclcpp::Duration::from_seconds(.002))==controller_interface::return_type::OK&&!vf->qpFeasible(),"zero target wrench does not bypass FT freshness in VS-VF");
  vf->on_deactivate(state);
  auto limited=controller("adaptive_depth_limit",{rclcpp::Parameter("tissue.kappa",1000.)});
  check(limited->on_configure(state)==Callback::SUCCESS,"depth-limited model configured");Hardware lhw;lhw.assign(*limited);Driver ld(*limited);
  check(limited->on_activate(state)==Callback::SUCCESS,"depth-limited controller activated");ld.pump();
  const double ceiling=adaptive_compliance_controller::math::huntCrossley(1000.,100.,1.35,.008,0.);
  check(limited->qpFeasible()&&std::abs(lhw.effort[2])<=ceiling+1e-6,"HC maximum penetration caps VS-CF predicted force");
  limited->on_deactivate(state);
  std::cout<<checks<<" adaptive lifecycle checks passed"<<std::endl;rclcpp::shutdown();return 0;
}catch(const std::exception&e){std::cerr<<"FAIL "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}}
