#ifndef FAST_LIO_ODOMETRY_COVARIANCE_HPP
#define FAST_LIO_ODOMETRY_COVARIANCE_HPP

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>

namespace fast_lio_full
{
// Error-state layout in use-ikfom.hpp, guarded against changes by the adapter.
constexpr int kStateDof = 23;
constexpr int kPosition = 0, kRotation = 3, kVelocity = 12, kGyroBias = 15;
using StateCovariance = Eigen::Matrix<double, kStateDof, kStateDof>;
using Covariance6 = Eigen::Matrix<double, 6, 6>;

struct Options
{
    double covariance_scale = 1.0;
    double position_stddev_floor = 0.05;       // m
    double orientation_stddev_floor = 0.03;   // rad
    double linear_stddev_floor = 0.10;         // m/s
    double angular_stddev_floor = 0.03;        // rad/s
    double gyro_noise_stddev = 0.02;           // rad/s, per sample, NOT EKF Q
    double max_gyro_gap = 0.02;                // s, on EACH side of interpolation

    bool valid() const
    {
        const double positive[] = {position_stddev_floor, orientation_stddev_floor,
            linear_stddev_floor, angular_stddev_floor, gyro_noise_stddev, max_gyro_gap};
        for (double value : positive)
            if (!std::isfinite(value) || value <= 0.0 || !std::isfinite(value * value))
                return false;
        return std::isfinite(covariance_scale) && covariance_scale >= 1.0;
    }
};

inline Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d result;
    result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return result;
}

// Reject invalid covariance; repair only floating-point asymmetry/negative modes.
// Work on copies, never modify the filter's own P.
template<int N>
bool sanitize_covariance(Eigen::Matrix<double, N, N> &covariance)
{
    if (!covariance.allFinite()) return false;
    const double scale = std::max(1.0, covariance.cwiseAbs().maxCoeff());
    if ((covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-7 * scale)
        return false;
    covariance = (0.5 * (covariance + covariance.transpose())).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
        solver.eigenvalues().minCoeff() < -1e-9 * scale)
        return false;
    if (solver.eigenvalues().minCoeff() < 0.0)
    {
        covariance = solver.eigenvectors() * solver.eigenvalues().cwiseMax(0.0).asDiagonal()
            * solver.eigenvectors().transpose();
        covariance = (0.5 * (covariance + covariance.transpose())).eval();
    }
    return covariance.allFinite();
}

struct GyroSample
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp = 0.0;
    Eigen::Vector3d value = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

// Samples use the SAME corrected IMU clock as the estimator. No extrapolation.
inline bool interpolate_gyro(const GyroSample &before, const GyroSample *after,
                             double target, double max_gap, GyroSample &result)
{
    if (!std::isfinite(target) || target <= 0.0 || !std::isfinite(max_gap) || max_gap <= 0.0 ||
        !std::isfinite(before.stamp) || before.stamp <= 0.0 ||
        !before.value.allFinite() || !before.covariance.allFinite()) return false;
    const double age = target - before.stamp;
    if (age < 0.0 || age > max_gap) return false;
    if (age == 0.0)
    {
        result = before;
        return true;
    }
    if (!after || !std::isfinite(after->stamp) || after->stamp < target ||
        after->stamp - target > max_gap || after->stamp <= before.stamp ||
        !after->value.allFinite() || !after->covariance.allFinite()) return false;
    const double weight = age / (after->stamp - before.stamp);
    result.stamp = target;
    result.value = (1.0 - weight) * before.value + weight * after->value;
    // Linear weights give a conservative bound even for correlated samples;
    // squared weights would assume independent noise and understate it.
    result.covariance = (1.0 - weight) * before.covariance + weight * after->covariance;
    return sanitize_covariance(result.covariance);
}

inline void apply_floors(Covariance6 &covariance, double linear_stddev, double angular_stddev)
{
    for (int i = 0; i < 6; ++i)
    {
        const double stddev = i < 3 ? linear_stddev : angular_stddev;
        // Add PSD diagonal uncertainty; keep existing off-diagonal correlations.
        covariance(i, i) += std::max(0.0, stddev * stddev - covariance(i, i));
    }
}

inline bool transform_covariances(StateCovariance state_covariance,
                                  const Eigen::Matrix3d &body_to_world,
                                  const Eigen::Vector3d &velocity_body,
                                  Eigen::Matrix3d gyro_covariance, const Options &options,
                                  Covariance6 &pose_covariance, Covariance6 &twist_covariance)
{
    if (!options.valid() || !body_to_world.allFinite() || !velocity_body.allFinite() ||
        (body_to_world.transpose() * body_to_world - Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
        std::abs(body_to_world.determinant() - 1.0) > 1e-6 ||
        !sanitize_covariance(state_covariance) || !sanitize_covariance(gyro_covariance)) return false;

    Eigen::Matrix<double, 6, kStateDof> pose_jacobian =
        Eigen::Matrix<double, 6, kStateDof>::Zero();
    pose_jacobian.block<3, 3>(0, kPosition).setIdentity();
    // MTK uses R_true = R * Exp(delta_theta_body). ROS fixed-axis pose
    // small-angle error is delta_theta_world = R * delta_theta_body.
    pose_jacobian.block<3, 3>(3, kRotation) = body_to_world;
    pose_covariance = options.covariance_scale *
        (pose_jacobian * state_covariance * pose_jacobian.transpose());

    Eigen::Matrix<double, 6, kStateDof> twist_jacobian =
        Eigen::Matrix<double, 6, kStateDof>::Zero();
    // v_body = R^T v_world, so delta_v_body = [v_body]x delta_theta + R^T delta_v_world.
    twist_jacobian.block<3, 3>(0, kRotation) = hat(velocity_body);
    twist_jacobian.block<3, 3>(0, kVelocity) = body_to_world.transpose();
    // omega_body = gyro_measurement - bg, evaluated with the UPDATED bias.
    twist_jacobian.block<3, 3>(3, kGyroBias) = -Eigen::Matrix3d::Identity();
    twist_covariance = twist_jacobian * state_covariance * twist_jacobian.transpose();
    twist_covariance.block<3, 3>(3, 3) += gyro_covariance;
    // The gyro was also used by the EKF. Its cross-covariance with the posterior
    // is unavailable: Cov(a+b) <= 2(Cov(a)+Cov(b)), without an independence claim.
    twist_covariance *= 2.0 * options.covariance_scale;
    apply_floors(pose_covariance, options.position_stddev_floor, options.orientation_stddev_floor);
    apply_floors(twist_covariance, options.linear_stddev_floor, options.angular_stddev_floor);
    return sanitize_covariance(pose_covariance) && sanitize_covariance(twist_covariance);
}
} // namespace fast_lio_full
#endif
