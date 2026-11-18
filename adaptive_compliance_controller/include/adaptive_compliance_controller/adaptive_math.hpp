// Torque adaptation of Beber et al., ICRA 2024, DOI 10.1109/ICRA57147.2024.10610167.
// See NOTICE and LICENSE for source attribution. No upstream binary is used.
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace adaptive_compliance_controller::math {
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;

inline double huntCrossley(double kappa, double viscosity, double beta,
                          double penetration, double penetration_rate) {
  if (!std::isfinite(kappa) || kappa <= 0. || !std::isfinite(viscosity) || viscosity < 0. ||
      !std::isfinite(beta) || beta < 1. || !std::isfinite(penetration) ||
      !std::isfinite(penetration_rate)) return std::numeric_limits<double>::quiet_NaN();
  if (penetration <= 0.) return 0.;
  // Unilateral contact: a rapid unloading phase cannot create tensile force.
  return std::max(0., (kappa + viscosity * penetration_rate) * std::pow(penetration, beta));
}

// Strictly convex diagonal 3-variable QP with box bounds and two halfspaces.
// Enumerate at most 93 active sets (0..3 of 8 constraints), using fixed storage.
// This replaces the upstream precompiled qpOASES binary for this exact QP size.
inline bool solveQP(const V3 &h, const V3 &g, const V3 &lower, const V3 &upper,
                    const std::array<V3, 2> &planes, const Eigen::Vector2d &rhs, V3 &solution) {
  if (!h.allFinite() || h.minCoeff() <= 0. || !g.allFinite() ||
      !lower.allFinite() || !upper.allFinite() || (lower.array() > upper.array()).any() ||
      !planes[0].allFinite() || !planes[1].allFinite() || !rhs.allFinite()) return false;
  Eigen::Matrix<double,8,3> a;
  Eigen::Matrix<double,8,1> b;
  a.setZero();
  for (int i=0;i<3;++i) {a(i,i)=1.;b[i]=upper[i];a(i+3,i)=-1.;b[i+3]=-lower[i];}
  a.row(6)=planes[0].transpose();a.row(7)=planes[1].transpose();
  b[6]=rhs[0];b[7]=rhs[1];
  for (int i=0;i<8;++i) {
    const double scale=a.row(i).norm();
    if (scale > 1e-14) {a.row(i)/=scale;b[i]/=scale;}
    else if (b[i] < -1e-10) return false;
  }
  const V3 inverse_h=h.cwiseInverse(), free=-inverse_h.cwiseProduct(g);
  bool found=false;
  double best=std::numeric_limits<double>::infinity();
  for (unsigned mask=0;mask<256;++mask) {
    unsigned bits=mask;int count=0;
    while(bits){bits&=bits-1;++count;}
    if(count>3)continue;
    M3 active=M3::Zero();V3 bound=V3::Zero();int n=0;
    for (int i=0;i<8;++i) if(mask & (1u<<i)) {active.row(n)=a.row(i);bound[n++]=b[i];}
    V3 candidate=free;
    if (n) {
      M3 gram=active*inverse_h.asDiagonal()*active.transpose();
      for(int i=n;i<3;++i)gram(i,i)=1.;
      Eigen::LDLT<M3> ldlt(gram);
      if(ldlt.info()!=Eigen::Success || ldlt.vectorD().minCoeff()<1e-12)continue;
      const V3 multiplier=ldlt.solve(active*free-bound);
      if(!multiplier.allFinite() || multiplier.minCoeff() < -1e-8)continue;
      candidate-=inverse_h.cwiseProduct(active.transpose()*multiplier);
    }
    if(!candidate.allFinite() || (a*candidate-b).maxCoeff()>1e-7)continue;
    const double cost=.5*candidate.dot(h.cwiseProduct(candidate))+g.dot(candidate);
    if(cost<best) {best=cost;solution=candidate;found=true;}
  }
  return found;
}

struct Config {
  V3 k_min{300.,300.,100.}, k_max{1000.,1000.,1000.};
  V3 damping{30.,30.,20.}, q{3200.,3200.,3200.}, r{1e-5,1e-5,1e-5};
  double tank_min=.4, tank_initial=.5, tank_max=1., power_limit=.1;
  bool valid() const {
    return k_min.allFinite() && k_max.allFinite() && damping.allFinite() && q.allFinite() && r.allFinite() &&
      k_min.minCoeff()>0. && (k_max-k_min).minCoeff()>=0. && damping.minCoeff()>=0. &&
      q.minCoeff()>0. && r.minCoeff()>0. && std::isfinite(tank_min) && tank_min>0. &&
      std::isfinite(tank_initial) && tank_initial>=tank_min && std::isfinite(tank_max) &&
      tank_max>=tank_initial && std::isfinite(power_limit) && power_limit>=0.;
  }
};

struct Result {
  V3 stiffness=V3::Zero(), wrench=V3::Zero();
  double tank=0., variable_power=0., damping_power=0.;
  bool feasible=false;
};

class PassiveQP {
 public:
  explicit PassiveQP(const Config &config=Config()):cfg_(config),tank_(config.tank_initial) {}
  bool configure(const Config &config) {if(!config.valid())return false;cfg_=config;reset();return true;}
  void reset() {tank_=cfg_.tank_initial;}
  double energy() const {return tank_;}
  Result step(const V3 &error, const V3 &velocity, const V3 &desired_force,
              const V3 &force_min, const V3 &force_max, double dt) {
    Result result;result.tank=tank_;
    if(!cfg_.valid() || !error.allFinite() || !velocity.allFinite() || !desired_force.allFinite() ||
       !force_min.allFinite() || !force_max.allFinite() || (force_min.array()>force_max.array()).any() ||
       !std::isfinite(dt) || dt<=0.)return result;
    const V3 drag=cfg_.damping.cwiseProduct(velocity);
    V3 lower=cfg_.k_min,upper=cfg_.k_max;
    for(int i=0;i<3;++i) {
      if(std::abs(error[i])<1e-12) {
        if(-drag[i]<force_min[i]-1e-9 || -drag[i]>force_max[i]+1e-9)return result;
      } else {
        double a=(force_min[i]+drag[i])/error[i],b=(force_max[i]+drag[i])/error[i];
        if(a>b)std::swap(a,b);
        lower[i]=std::max(lower[i],a);upper[i]=std::min(upper[i],b);
      }
    }
    const V3 h=cfg_.r+cfg_.q.cwiseProduct(error.cwiseProduct(error));
    const V3 g=-cfg_.r.cwiseProduct(cfg_.k_min)-cfg_.q.cwiseProduct(error.cwiseProduct(desired_force+drag));
    const V3 power_row=error.cwiseProduct(velocity);
    const double sigma=tank_<cfg_.tank_max ? 1. : 0.;
    const double dissipated=sigma*velocity.dot(drag);
    const double baseline=power_row.dot(cfg_.k_min);
    const Eigen::Vector2d budgets(dissipated+baseline+(tank_-cfg_.tank_min)/dt,
                                  dissipated+baseline+cfg_.power_limit);
    V3 stiffness;
    if(!solveQP(h,g,lower,upper,{power_row,power_row},budgets,stiffness))return result;
    const double variable=power_row.dot(stiffness-cfg_.k_min);
    const double next=tank_+dt*(dissipated-variable);
    if(!std::isfinite(next) || next<cfg_.tank_min-1e-9)return result;
    // Discard excess charge at the upper limit; never refill an empty tank.
    tank_=std::min(cfg_.tank_max,std::max(cfg_.tank_min,next));
    result.stiffness=stiffness;result.wrench=stiffness.cwiseProduct(error)-drag;
    result.tank=tank_;result.variable_power=variable;result.damping_power=dissipated;result.feasible=true;
    return result;
  }
 private:
  Config cfg_;
  double tank_;
};
}  // namespace adaptive_compliance_controller::math
