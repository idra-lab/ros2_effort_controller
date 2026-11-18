#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace effort_controller_base::control_math {
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;

// Floor eigenvalues of J M^-1 J^T instead of inverting a singular matrix.
// The cutoff is in the unscaled [linear, angular] task coordinates.
inline bool taskInertia(const Eigen::MatrixXd &mass, const Eigen::MatrixXd &jac,
                        double cutoff, Matrix6 &lambda,
                        Eigen::MatrixXd &inverse_mass_jt, Matrix6 &task_pinv) {
  if (mass.rows() == 0 || mass.rows() != mass.cols() || jac.rows() != 6 ||
      jac.cols() != mass.rows() || !mass.allFinite() || !jac.allFinite() ||
      !std::isfinite(cutoff) || cutoff <= 0.0 ||
      !mass.isApprox(mass.transpose(), 1e-10)) return false;
  Eigen::LLT<Eigen::MatrixXd> factor(mass);
  if (factor.info() != Eigen::Success) return false;
  inverse_mass_jt = factor.solve(jac.transpose());
  const Matrix6 inverse_lambda = jac * inverse_mass_jt;
  Eigen::SelfAdjointEigenSolver<Matrix6> eig(
      (inverse_lambda + inverse_lambda.transpose()) * 0.5);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
      eig.eigenvalues().minCoeff() < -cutoff) return false;
  Vector6 regularized, pseudo;
  for (int i = 0; i < 6; ++i) {
    const double value = eig.eigenvalues()[i];
    regularized[i] = 1.0 / std::max(value, cutoff);
    pseudo[i] = value > cutoff ? 1.0 / value : 0.0;
  }
  lambda = eig.eigenvectors() * regularized.asDiagonal() * eig.eigenvectors().transpose();
  task_pinv = eig.eigenvectors() * pseudo.asDiagonal() * eig.eigenvectors().transpose();
  return lambda.allFinite() && inverse_mass_jt.allFinite() && task_pinv.allFinite();
}

// Generalized modal damping: V^T Lambda V = I, hence V^-T = Lambda V.
inline bool modalDamping(const Matrix6 &lambda, const Matrix6 &stiffness,
                         double ratio, Matrix6 &damping) {
  if (!lambda.allFinite() || !stiffness.allFinite() ||
      !lambda.isApprox(lambda.transpose(), 1e-10) ||
      !stiffness.isApprox(stiffness.transpose(), 1e-10) ||
      !std::isfinite(ratio) || ratio < 0.0) return false;
  Eigen::LLT<Matrix6> factor(lambda);
  if (factor.info() != Eigen::Success) return false;
  Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6> eig(stiffness, lambda);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() ||
      eig.eigenvalues().minCoeff() < -1e-10) return false;
  const Matrix6 modes = lambda * eig.eigenvectors();
  const Vector6 values = 2.0 * ratio * eig.eigenvalues().cwiseMax(0.0).cwiseSqrt();
  damping = modes * values.asDiagonal() * modes.transpose();
  return damping.allFinite();
}

inline bool wrenchActive(const Vector6 &target) {
  return target.head<3>().norm() > 1e-6 || target.tail<3>().norm() > 1e-6;
}

// Sensor sign convention: environment-on-tool; desired is tool-on-environment.
inline Vector6 forceFeedback(const Vector6 &target, const Vector6 &measured,
                             double gain, bool hybrid) {
  if (!wrenchActive(target)) return Vector6::Zero();
  if (hybrid && target.head<3>().norm() > 1e-6) {
    const Eigen::Vector3d normal = target.head<3>().normalized();
    Vector6 result = target;
    result.head<3>() += gain * normal * normal.dot((target + measured).head<3>());
    return result;
  }
  return target + gain * (target + measured);
}

// offset = source origin - TCP origin, expressed in base axes.
inline Vector6 wrenchAtTcp(const Vector6 &source, const Eigen::Matrix3d &rotation,
                           const Eigen::Vector3d &offset) {
  Vector6 result;
  result.head<3>() = rotation * source.head<3>();
  result.tail<3>() = rotation * source.tail<3>() + offset.cross(result.head<3>());
  return result;
}

// Raw input, bias, COM and tare gravity all use the FT sensor's axes/origin.
// rotation maps sensor axes into base axes. A zero tare vector means untared
// input; otherwise it is gravity in sensor axes at the external zeroing pose.
inline Vector6 compensateSensorWrench(const Vector6 &raw, const Eigen::Matrix3d &rotation,
                                     double mass_kg, const Eigen::Vector3d &com_sensor_m,
                                     const Vector6 &bias, double sign,
                                     const Eigen::Vector3d &tare_gravity_sensor) {
  const Eigen::Vector3d gravity = rotation.transpose() * Eigen::Vector3d(0, 0, -9.80665*mass_kg);
  const Eigen::Vector3d load = gravity-tare_gravity_sensor;
  Vector6 corrected = sign*(raw-bias);
  corrected.head<3>() -= load;
  corrected.tail<3>() -= com_sensor_m.cross(load);
  return corrected;
}

inline bool inhibitInput(bool commands_ok, bool sensor_ok, const Vector6 &target,
                         const Vector6 &measured, double force_limit, bool &latched) {
  if (sensor_ok && measured.head<3>().norm() > force_limit) latched = true;
  return !commands_ok || (wrenchActive(target) && !sensor_ok) || latched;
}

// Invalid inputs leave the previous command untouched; caller must handle false.
inline bool limitEffort(const Eigen::VectorXd &target, const Eigen::VectorXd &limits,
                        double increment, Eigen::VectorXd &command) {
  if (target.size() != command.size() || limits.size() != command.size() ||
      !target.allFinite() || !command.allFinite() || !limits.allFinite() ||
      limits.size() == 0 || limits.minCoeff() <= 0.0 ||
      !std::isfinite(increment) || increment <= 0.0) return false;
  for (Eigen::Index i = 0; i < command.size(); ++i) {
    const double desired = std::clamp(target[i], -limits[i], limits[i]);
    command[i] = std::clamp(command[i] + std::clamp(desired-command[i], -increment, increment),
                            -limits[i], limits[i]);
  }
  return true;
}
}  // namespace effort_controller_base::control_math
