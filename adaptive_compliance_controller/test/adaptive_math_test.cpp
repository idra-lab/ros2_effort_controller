#include <adaptive_compliance_controller/adaptive_math.hpp>
#include <adaptive_compliance_controller/tissue_map.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <chrono>
using namespace adaptive_compliance_controller;
using namespace adaptive_compliance_controller::math;
int checks=0;
void check(bool value,const char *name){if(!value)throw std::runtime_error(name);++checks;std::cout<<"PASS "<<name<<'\n';}
int main(){try {
  check(std::abs(huntCrossley(1000.,100.,1.,.01,.02)-10.02)<1e-12,"HC elastic and viscous force");
  check(huntCrossley(1000.,100.,1.35,-.01,.1)==0.,"no contact has no tissue force");
  check(huntCrossley(1000.,100.,1.,.01,-20.)==0.,"unloading cannot create tensile tissue force");
  check(!std::isfinite(huntCrossley(-1.,0.,1.,.01,0.)),"invalid tissue coefficients rejected");
  V3 sol;const V3 zero=V3::Zero(),ten=V3::Constant(10.);
  const std::array<V3,2> none{zero,zero};
  check(solveQP(V3::Ones(),V3(-2,-3,-4),zero,ten,none,Eigen::Vector2d(1,1),sol) && sol.isApprox(V3(2,3,4)),"QP interior analytical optimum");
  check(solveQP(V3::Ones(),V3(-2,-3,-4),zero,V3(1,2,3),none,Eigen::Vector2d(1,1),sol) && sol.isApprox(V3(1,2,3)),"QP box analytical optimum");
  check(solveQP(V3::Ones(),V3(-2,-3,-4),zero,ten,{V3(1,1,0),zero},Eigen::Vector2d(3,1),sol) && sol.isApprox(V3(1,2,4)),"QP one coupled halfspace analytical optimum");
  check(solveQP(V3::Ones(),V3(-2,-3,-4),zero,ten,{V3(1,1,0),V3(0,1,1)},Eigen::Vector2d(3,4),sol) && sol.isApprox(V3(5./3,4./3,8./3)),"QP two coupled halfspaces analytical optimum");
  check(!solveQP(V3::Ones(),zero,V3::Ones(),zero,none,Eigen::Vector2d(1,1),sol),"infeasible box rejected");
  check(!solveQP(V3::Zero(),zero,zero,ten,none,Eigen::Vector2d(1,1),sol),"nonpositive Hessian rejected");
  check(!solveQP(V3::Ones(),zero,zero,ten,none,Eigen::Vector2d(-1,1),sol),"infeasible zero-normal halfspace rejected");
  Config cfg;cfg.k_min=V3::Constant(100.);cfg.k_max=V3::Constant(1000.);cfg.damping.setZero();
  PassiveQP core;check(core.configure(cfg),"valid configuration accepted");
  const V3 e=V3::Constant(.01),f(2,3,4);
  auto r=core.step(e,zero,f,V3::Constant(-15),V3::Constant(15),.002);
  const V3 expected=(cfg.r.cwiseProduct(cfg.k_min)+cfg.q.cwiseProduct(e.cwiseProduct(f))).cwiseQuotient(cfg.r+cfg.q.cwiseProduct(e.cwiseProduct(e)));
  check(r.feasible && r.stiffness.isApprox(expected),"stiffness matches scalar closed-form force tracking optimum");
  check(r.tank==cfg.tank_initial,"stationary robot does not consume tank energy");
  core.reset();r=core.step(e,zero,V3::Constant(10),V3::Constant(-2),V3::Constant(2),.002);
  check(r.feasible && (r.wrench.array()<=2.+1e-9).all(),"predicted force box constrains stiffness");
  r=core.step(V3::Constant(1.),zero,f,V3::Constant(-2),V3::Constant(2),.002);
  check(!r.feasible,"force box infeasible even at minimum stiffness is reported");
  cfg.power_limit=.1;cfg.tank_initial=.5;cfg.tank_min=.4;core.configure(cfg);
  bool energy_ok=true,power_ok=true,balance_ok=true;
  for(int i=0;i<1000;++i){
    const double previous=core.energy();
    r=core.step(e,V3::Constant(.1),V3::Constant(10),V3::Constant(-15),V3::Constant(15),.01);
    energy_ok &= r.feasible && r.tank>=cfg.tank_min-1e-9 && r.tank<=cfg.tank_max;
    power_ok &= r.variable_power-r.damping_power<=cfg.power_limit+1e-8;
    balance_ok &= std::abs(r.tank-(previous+.01*(r.damping_power-r.variable_power)))<1e-8;
  }
  check(energy_ok,"1000-step tank never drops below minimum");
  check(power_ok,"1000-step extraction respects energy valve");
  check(balance_ok,"1000-step discrete energy balance is consistent");
  check(std::abs(core.energy()-cfg.tank_min)<1e-7,"finite energy depletes to threshold without artificial refill");
  r=core.step(e,V3::Constant(-.1),V3::Constant(10),V3::Constant(-15),V3::Constant(15),.01);
  check(r.feasible && r.tank>cfg.tank_min,"opposing variable spring power recharges tank");
  core.reset();check(core.energy()==cfg.tank_initial,"reactivation reset restores configured initial energy");
  const double before=core.energy();r=core.step(e,zero,f,-ten,ten,0.);
  check(!r.feasible && core.energy()==before,"zero period cannot integrate energy");
  V3 nan=e;nan[0]=std::numeric_limits<double>::quiet_NaN();
  check(!core.step(nan,zero,f,-ten,ten,.01).feasible,"nonfinite state cannot enter QP");
  Config invalid=cfg;invalid.tank_initial=.1;check(!core.configure(invalid),"initial tank below threshold rejected");
  invalid=cfg;invalid.r[0]=0.;check(!core.configure(invalid),"singular objective rejected");
  invalid=cfg;invalid.power_limit=-1.;check(!core.configure(invalid),"negative extraction limit rejected");
  TissueMap map;map.flat({.2,1000.,10.});Tissue t;
  check(map.sample(100.,-100.,t)&&t.surface==.2,"configured flat tissue does not depend on XY");
  const auto path=std::filesystem::temp_directory_path()/("adaptive_map_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(path);
  const auto put=[&](const char *name,const char *value){std::ofstream(path/name)<<value;};
  put("x.txt","0 1");put("y.txt","0 1");put("z.txt","0 .1 .2 .3");
  put("stiffness.txt","1000 2000 3000 4000");put("damping.txt","10 20 30 40");map.load(path.string());
  check(map.sample(.5,.5,t)&&std::abs(t.surface-.15)<1e-12&&t.kappa==2500.&&t.viscosity==25.,"map bilinear interpolation and x-major layout");
  check(map.sample(1.,1.,t)&&t.kappa==4000.,"map upper boundary valid");
  check(!map.sample(-.1,0.,t),"out-of-domain map lookup rejected");
  put("stiffness.txt","1 2");bool rejected=false;try{map.load(path.string());}catch(...){rejected=true;}
  check(rejected,"malformed grid shape rejected");
  put("x.txt","0 0");rejected=false;try{map.load(path.string());}catch(...){rejected=true;}
  check(rejected,"nonincreasing map coordinates rejected");
  std::filesystem::remove_all(path);
  std::cout<<checks<<" adaptive math checks passed\n";return 0;
}catch(const std::exception &e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
