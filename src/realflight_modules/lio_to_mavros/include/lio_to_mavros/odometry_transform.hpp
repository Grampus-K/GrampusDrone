#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lio_to_mavros {
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector3 = Eigen::Vector3d;
using Rotation = Eigen::Matrix3d;

inline Rotation hat(const Vector3 &v) {
  Rotation m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

inline bool validRotation(const Rotation &r) {
  return r.allFinite() && (r.transpose()*r - Rotation::Identity()).norm() < 1e-6 &&
         std::abs(r.determinant() - 1.0) < 1e-6;
}

inline Eigen::Quaterniond checkedQuaternion(Eigen::Quaterniond q) {
  if (!q.coeffs().allFinite() || std::abs(q.norm()-1.0) > 0.01)
    throw std::invalid_argument("quaternion norm outside [0.99, 1.01]");
  return q.normalized();
}

// T_X_Y maps Y coordinates into X coordinates.
struct Transform {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Rotation rotation{Rotation::Identity()};
  Vector3 translation{Vector3::Zero()};
  Transform inverse() const {
    Transform t;
    t.rotation = rotation.transpose();
    t.translation = -t.rotation * translation;
    return t;
  }
  Transform operator*(const Transform &other) const {
    Transform t;
    t.rotation = rotation * other.rotation;
    t.translation = translation + rotation * other.translation;
    return t;
  }
};

inline Transform imuFromBase(const Transform &base_from_lidar,
                             const Transform &imu_from_lidar) {
  return imu_from_lidar * base_from_lidar.inverse();
}

inline bool sanitizeCovariance(Matrix6 &c) {
  if (!c.allFinite()) return false;
  const double scale = std::max(1.0, c.cwiseAbs().maxCoeff());
  if ((c-c.transpose()).cwiseAbs().maxCoeff() > 1e-7*scale) return false;
  c = (0.5*(c+c.transpose())).eval();
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(c);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      solver.eigenvalues().minCoeff() < -1e-9*scale) return false;
  if (solver.eigenvalues().minCoeff() < 0) {
    c = solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal() *
        solver.eigenvectors().transpose();
    c = (0.5*(c+c.transpose())).eval();
  }
  return c.allFinite();
}

struct Sample {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Transform pose;
  Vector3 linear{Vector3::Zero()}, angular{Vector3::Zero()};
  Matrix6 pose_covariance{Matrix6::Identity()}, twist_covariance{Matrix6::Identity()};
};

// Input pose error is [dp_W, dtheta_W] (left/fixed-axis rotation error).
inline Matrix6 poseJacobian(const Rotation &r_wi, const Transform &i_b,
                            const Transform &a_w) {
  Matrix6 j = Matrix6::Zero();
  j.block<3,3>(0,0) = a_w.rotation;
  j.block<3,3>(0,3) = -a_w.rotation * hat(r_wi*i_b.translation);
  j.block<3,3>(3,3) = a_w.rotation;
  return j;
}

inline Matrix6 twistJacobian(const Transform &i_b) {
  Matrix6 j = Matrix6::Zero();
  const Rotation r_bi = i_b.rotation.transpose();
  j.block<3,3>(0,0) = r_bi;
  j.block<3,3>(0,3) = -r_bi * hat(i_b.translation);
  j.block<3,3>(3,3) = r_bi;
  return j;
}

inline Sample transformSample(Sample in, const Transform &i_b, const Transform &a_w) {
  if (!validRotation(in.pose.rotation) || !validRotation(i_b.rotation) ||
      !validRotation(a_w.rotation) || !in.pose.translation.allFinite() ||
      !i_b.translation.allFinite() || !a_w.translation.allFinite() ||
      !in.linear.allFinite() || !in.angular.allFinite() ||
      !sanitizeCovariance(in.pose_covariance) || !sanitizeCovariance(in.twist_covariance))
    throw std::invalid_argument("invalid odometry, transform or covariance");
  Sample out;
  out.pose = a_w * in.pose * i_b;
  out.linear = i_b.rotation.transpose() * (in.linear + in.angular.cross(i_b.translation));
  out.angular = i_b.rotation.transpose() * in.angular;
  const Matrix6 jp = poseJacobian(in.pose.rotation, i_b, a_w), jt = twistJacobian(i_b);
  out.pose_covariance = jp * in.pose_covariance * jp.transpose();
  out.twist_covariance = jt * in.twist_covariance * jt.transpose();
  if (!out.pose.translation.allFinite() || !out.linear.allFinite() || !out.angular.allFinite() ||
      !sanitizeCovariance(out.pose_covariance) || !sanitizeCovariance(out.twist_covariance))
    throw std::invalid_argument("non-finite transformed odometry or covariance");
  return out;
}

// Preserve gravity: rotate only yaw, and optionally place the initial B at zero.
inline Transform referenceTransform(const Transform &world_from_base, double yaw,
                                    bool zero_position, bool zero_yaw) {
  Transform a_w;
  if (zero_yaw) {
    if (std::hypot(world_from_base.rotation(0,0), world_from_base.rotation(1,0)) < 1e-6)
      throw std::invalid_argument("initial heading is undefined (vertical body x axis)");
    yaw -= std::atan2(world_from_base.rotation(1,0), world_from_base.rotation(0,0));
  }
  a_w.rotation = Eigen::AngleAxisd(yaw, Vector3::UnitZ()).toRotationMatrix();
  if (zero_position) a_w.translation = -a_w.rotation * world_from_base.translation;
  return a_w;
}
}  // namespace lio_to_mavros
