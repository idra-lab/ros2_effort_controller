#include <effort_controller_base/control_math.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace math = effort_controller_base::control_math;
using math::Matrix6;
using math::Vector6;
int count = 0;
void check(bool condition, const char *name) {
  if (!condition) throw std::runtime_error(name);
  ++count;
  std::cout << "PASS " << name << '\n';
}

int main() {
 try {
  Matrix6 lambda, pinv, damping;
  Eigen::MatrixXd inverse_mass_jt;
  Eigen::MatrixXd mass = 4.0 * Eigen::MatrixXd::Identity(7, 7);
  Eigen::MatrixXd jac = Eigen::MatrixXd::Zero(6, 7);
  jac.leftCols(6).setIdentity();
  check(math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv) &&
        lambda.isApprox(4.0*Matrix6::Identity()), "nominal redundant task inertia");
  check(math::modalDamping(lambda, 25.0*Matrix6::Identity(), 0.7, damping) &&
        damping.isApprox(14.0*Matrix6::Identity()), "closed-form modal damping");
  Eigen::MatrixXd projector = Eigen::MatrixXd::Identity(7,7) - jac.transpose()*pinv*inverse_mass_jt.transpose();
  check((jac*mass.inverse()*projector).norm() < 1e-10 &&
        (projector*projector-projector).norm() < 1e-10 && std::abs(projector(6,6)-1.0)<1e-10,
        "redundant nullspace projector preserves task acceleration");
  jac(5,5) = 0.0;
  check(math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv) && lambda.allFinite() &&
        math::modalDamping(lambda, 25.0*Matrix6::Identity(), 0.7, damping) && damping.allFinite(),
        "rank-deficient Jacobian gives finite inertia and damping");
  projector = Eigen::MatrixXd::Identity(7,7) - jac.transpose()*pinv*inverse_mass_jt.transpose();
  check((jac*mass.inverse()*projector).norm() < 1e-10, "singular nullspace projector");
  jac(5,5) = 1e-12;
  check(math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv) && lambda.norm() < 1.01e6,
        "near-singular inertia is bounded by eigenvalue floor");
  jac.setZero();
  check(math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv) &&
        lambda.isApprox(1e6*Matrix6::Identity()) && pinv.isZero(), "zero Jacobian is handled");
  mass(0,0) = -1;
  check(!math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv), "nonpositive mass is rejected");
  mass.setIdentity(); mass(0,1)=0.2;
  check(!math::taskInertia(mass, jac, 1e-6, lambda, inverse_mass_jt, pinv), "asymmetric mass is rejected");
  mass.setIdentity();
  check(!math::taskInertia(mass, jac, 0, lambda, inverse_mass_jt, pinv), "invalid regularization is rejected");
  check(!math::modalDamping(-Matrix6::Identity(), Matrix6::Identity(), 1, damping), "nonpositive task inertia rejected");
  check(!math::modalDamping(Matrix6::Identity(), -Matrix6::Identity(), 1, damping), "negative stiffness rejected");
  check(math::modalDamping(Matrix6::Identity(), Matrix6::Zero(), 1, damping) && damping.isZero(),
        "semidefinite stiffness supported");

  // Dense coupled inertia/geometry exercises the non-diagonal projector and modes.
  Eigen::MatrixXd seed = Eigen::MatrixXd::Random(7,7);
  mass = seed.transpose()*seed + Eigen::MatrixXd::Identity(7,7);
  jac = Eigen::MatrixXd::Random(6,7);
  check(math::taskInertia(mass,jac,1e-10,lambda,inverse_mass_jt,pinv) &&
        (jac*mass.inverse()*jac.transpose()*lambda-Matrix6::Identity()).norm() < 1e-9,
        "coupled redundant task inertia matches full-rank inverse");
  projector = Eigen::MatrixXd::Identity(7,7)-jac.transpose()*pinv*inverse_mass_jt.transpose();
  check((jac*mass.inverse()*projector).norm() < 1e-9,
        "coupled dynamically consistent projector rejects task acceleration");
  Matrix6 gain_seed=Matrix6::Random();
  Matrix6 stiffness=gain_seed.transpose()*gain_seed+Matrix6::Identity();
  check(math::modalDamping(lambda,stiffness,0.7,damping) && damping.isApprox(damping.transpose(),1e-10) &&
        Eigen::SelfAdjointEigenSolver<Matrix6>(damping).eigenvalues().minCoeff() > 0,
        "coupled damping is symmetric positive definite");
  Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6> modes(stiffness,lambda);
  Matrix6 modal = modes.eigenvectors().transpose()*damping*modes.eigenvectors();
  check(modal.isApprox((1.4*modes.eigenvalues().cwiseSqrt()).asDiagonal().toDenseMatrix(),1e-9),
        "coupled modal damping has the requested damping ratio");
  mass=Eigen::MatrixXd::Identity(6,6); jac=Eigen::MatrixXd::Identity(6,6);
  check(math::taskInertia(mass,jac,1e-6,lambda,inverse_mass_jt,pinv) && lambda.isIdentity(),
        "six-joint model supported");
  Vector6 target = Vector6::Zero(), measured = Vector6::Zero();
  target[2] = 0.05;
  check(std::abs(math::forceFeedback(target, measured, 1, true)[2] - 0.1) < 1e-12,
        "force ramp below 0.1 N is continuous");
  target[2] = -2; measured[2]=2; measured[0]=0.5;
  check(math::forceFeedback(target, measured, 1, true).isApprox(target), "2 N equilibrium and tangential feedback rejection");
  target.setZero(); target[4]=0.02;
  check(math::wrenchActive(target), "torque-only command requires sensor feedback");
  check(math::forceFeedback(Vector6::Zero(), measured, 1, false).isZero(), "pose-only mode does not apply force feedback");

  Vector6 source = Vector6::Zero(); source[1]=2;
  Eigen::Vector3d offset(0.1,0,0);
  const auto transformed = math::wrenchAtTcp(source, Eigen::Matrix3d::Identity(), offset);
  check(std::abs(transformed[5]-0.2) < 1e-12, "sensor-to-TCP moment arm sign");
  const Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()).toRotationMatrix();
  source << 1,2,3,0.1,0.2,0.3;
  Vector6 twist; twist << .3,-.2,.1,.4,.5,-.1;
  Vector6 source_twist;
  source_twist.head<3>() = rotation.transpose()*(twist.head<3>() + twist.tail<3>().cross(offset));
  source_twist.tail<3>() = rotation.transpose()*twist.tail<3>();
  check(std::abs(source.dot(source_twist)-math::wrenchAtTcp(source,rotation,offset).dot(twist)) < 1e-12,
        "wrench transform preserves virtual power");

  const double tool_mass = .135617;
  const Eigen::Vector3d com(.02,-.01,.08);
  Vector6 bias; bias << .1,-.2,.3,.01,-.02,.03;
  Vector6 contact; contact << .15,-.3,2.,.01,.02,-.03;
  for (int tared=0; tared<2; ++tared) {
    Eigen::Vector3d tare_gravity = Eigen::Vector3d::Zero();
    if(tared) tare_gravity.z() = tool_mass*9.80665;
    for (int orientation=0; orientation<4; ++orientation) {
      const Eigen::Matrix3d sensor_rotation =
          Eigen::AngleAxisd(orientation*0.7,Eigen::Vector3d(1,2,3).normalized()).toRotationMatrix();
      const Eigen::Vector3d load = sensor_rotation.transpose()*Eigen::Vector3d(0,0,-tool_mass*9.80665)-tare_gravity;
      Vector6 weight; weight << load,com.cross(load);
      for (double sign : {-1., 1.}) {
        const Vector6 raw = bias+sign*(contact+weight);
        check(math::compensateSensorWrench(raw,sensor_rotation,tool_mass,com,bias,sign,tare_gravity).isApprox(contact,1e-12),
              "raw/tared FT retains 2 N contact with COM, bias and sign correction");
        check(math::compensateSensorWrench(bias+sign*weight,sensor_rotation,tool_mass,com,bias,sign,tare_gravity).norm()<1e-12,
              "unloaded tool has zero contact force and moment at any tilt");
      }
    }
  }
  check(math::compensateSensorWrench(contact,Eigen::Matrix3d::Identity(),0.,com,Vector6::Zero(),1.,Eigen::Vector3d::Zero()).isApprox(contact),
        "zero-mass contact-only input remains unchanged");
  bool fault=false;
  check(math::inhibitInput(true, false, target, measured, 4, fault) && !fault,
        "stale sensor inhibits torque-only target without command expiry");
  measured[2]=4.01;
  check(math::inhibitInput(true, true, Vector6::Zero(), measured, 4, fault) && fault,
        "overload protection works with command expiry disabled");
  measured.setZero();
  check(math::inhibitInput(true, true, Vector6::Zero(), measured, 4, fault), "overload fault stays latched");
  fault=false;
  check(math::inhibitInput(false, true, target, measured, 4, fault), "stale command inhibits control");
  check(!math::inhibitInput(true, false, Vector6::Zero(), measured, 4, fault), "legacy pose hold does not require FT heartbeat");

  Eigen::VectorXd command=Eigen::VectorXd::Zero(7), limits=Eigen::VectorXd::Constant(7,3);
  Eigen::VectorXd desired=Eigen::VectorXd::Constant(7,100);
  check(math::limitEffort(desired,limits,0.2,command) && command.isApprox(Eigen::VectorXd::Constant(7,0.2)),
        "large torque step is rate limited without terminating process");
  for(int i=0;i<100;++i) math::limitEffort(desired,limits,0.2,command);
  check(command.isApprox(limits), "URDF effort saturation including continuous joints");
  const Eigen::VectorXd old=command;
  desired[4]=std::numeric_limits<double>::infinity();
  check(!math::limitEffort(desired,limits,0.2,command) && command.isApprox(old), "infinite target rejected atomically");
  check(!math::limitEffort(Eigen::VectorXd::Zero(6),limits,0.2,command), "wrong joint count rejected");
  check(!math::limitEffort(old,limits,-1,command), "invalid torque increment rejected");
  limits[0]=std::numeric_limits<double>::quiet_NaN();
  check(!math::limitEffort(old,limits,1,command), "nonfinite effort limit rejected");
  std::cout << count << " checks passed\n";
  return 0;
 } catch(const std::exception &error) {
  std::cerr << "FAIL " << error.what() << '\n';
  return 1;
 }
}
