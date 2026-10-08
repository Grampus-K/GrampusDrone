#include "odometry_covariance.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace fast_lio_full;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

Eigen::Matrix3d exp_rotation(const Eigen::Vector3d &angle)
{
    return angle.norm() == 0.0 ? Eigen::Matrix3d::Identity() :
        Eigen::AngleAxisd(angle.norm(), angle.normalized()).toRotationMatrix();
}

int main()
{
    try
    {
        Options options;
        options.position_stddev_floor = options.orientation_stddev_floor = 1e-6;
        options.linear_stddev_floor = options.angular_stddev_floor = 1e-6;
        const Eigen::Matrix3d rotation = exp_rotation(Eigen::Vector3d(0.4, -0.3, 1.2));
        const Eigen::Vector3d velocity_world(2.0, -1.0, 0.5);
        const Eigen::Vector3d velocity_body = rotation.transpose() * velocity_world;
        const double step = 1e-6;
        Eigen::Matrix<double, 6, kStateDof> numerical_pose, numerical_twist;
        numerical_pose.setZero();
        numerical_twist.setZero();
        for (int axis = 0; axis < 3; ++axis)
        {
            const Eigen::Vector3d delta = step * Eigen::Vector3d::Unit(axis);
            const Eigen::Matrix3d plus = rotation * exp_rotation(delta);
            const Eigen::Matrix3d minus = rotation * exp_rotation(-delta);
            const Eigen::AngleAxisd plus_world(plus * rotation.transpose());
            const Eigen::AngleAxisd minus_world(minus * rotation.transpose());
            numerical_pose.block<3, 1>(3, kRotation + axis) =
                (plus_world.angle() * plus_world.axis() - minus_world.angle() * minus_world.axis()) / (2 * step);
            numerical_pose(axis, kPosition + axis) = 1.0;
            numerical_twist.block<3, 1>(0, kRotation + axis) =
                (plus.transpose() * velocity_world - minus.transpose() * velocity_world) / (2 * step);
            numerical_twist.block<3, 1>(0, kVelocity + axis) =
                (rotation.transpose() * (velocity_world + delta) -
                 rotation.transpose() * (velocity_world - delta)) / (2 * step);
            numerical_twist(3 + axis, kGyroBias + axis) = -1.0;
        }
        // Dense PSD P exercises state ordering AND all cross-covariance blocks.
        const StateCovariance dense = StateCovariance::Random();
        const StateCovariance state_covariance = dense * dense.transpose() * 0.01;
        const Eigen::Matrix3d gyro_covariance = Eigen::Vector3d(0.001, 0.002, 0.003).asDiagonal();
        Covariance6 pose, twist;
        require(transform_covariances(state_covariance, rotation, velocity_body,
            gyro_covariance, options, pose, twist), "valid covariance rejected");
        const Covariance6 expected_pose = numerical_pose * state_covariance * numerical_pose.transpose();
        Covariance6 expected_twist = numerical_twist * state_covariance * numerical_twist.transpose();
        expected_twist.block<3, 3>(3, 3) += gyro_covariance;
        expected_twist *= 2.0;
        require((pose - expected_pose).norm() < 1e-7, "pose frame/index Jacobian mismatch");
        require((twist - expected_twist).norm() < 1e-7, "body velocity/bias Jacobian mismatch");
        options.covariance_scale = 3.0;
        Covariance6 scaled_pose, scaled_twist;
        require(transform_covariances(state_covariance, rotation, velocity_body,
            gyro_covariance, options, scaled_pose, scaled_twist), "scaled covariance rejected");
        require((scaled_pose - 3.0 * pose).norm() < 1e-10 &&
                (scaled_twist - 3.0 * twist).norm() < 1e-10, "output scaling mismatch");

        options = Options();
        require(transform_covariances(StateCovariance::Zero(), rotation, Eigen::Vector3d::Zero(),
            Eigen::Matrix3d::Zero(), options, pose, twist), "floors rejected");
        require(std::abs(pose(0, 0) - 0.05 * 0.05) < 1e-12 &&
                std::abs(twist(0, 0) - 0.10 * 0.10) < 1e-12, "stddev not squared");
        Covariance6 invalid = Covariance6::Identity();
        invalid(1, 1) = -0.1;
        require(!sanitize_covariance(invalid), "negative eigenvalue accepted");
        invalid.setIdentity();
        invalid(0, 1) = 0.1;
        require(!sanitize_covariance(invalid), "asymmetric covariance accepted");
        invalid.setIdentity();
        invalid(0, 0) = std::numeric_limits<double>::quiet_NaN();
        require(!sanitize_covariance(invalid), "NaN covariance accepted");
        invalid.setIdentity();
        invalid(0, 0) = -1e-12;
        require(sanitize_covariance(invalid) && invalid(0, 0) >= 0.0, "roundoff repair failed");
        options.gyro_noise_stddev = -1.0;
        require(!options.valid(), "negative standard deviation accepted");
        options = Options();
        options.covariance_scale = 0.5;
        require(!options.valid(), "uncertainty reduction accepted");

        GyroSample before, after, result;
        before.stamp = 100.0;
        after.stamp = 100.01;
        before.value << 1.0, 2.0, 3.0;
        after.value << 3.0, 4.0, 5.0;
        before.covariance = Eigen::Matrix3d::Identity();
        after.covariance = 3.0 * Eigen::Matrix3d::Identity();
        require(interpolate_gyro(before, &after, 100.005, 0.02, result), "bracketed gyro rejected");
        require((result.value - Eigen::Vector3d(2.0, 3.0, 4.0)).norm() < 1e-10 &&
            (result.covariance - 2.0 * Eigen::Matrix3d::Identity()).norm() < 1e-10,
            "gyro interpolation/correlated-noise bound mismatch");
        require(interpolate_gyro(before, nullptr, 100.0, 0.02, result), "exact gyro rejected");
        require(!interpolate_gyro(before, nullptr, 100.005, 0.02, result), "gyro extrapolation accepted");
        require(!interpolate_gyro(before, &after, 100.005, 0.001, result), "stale gyro accepted");
        require(!interpolate_gyro(before, &after, 99.99, 0.02, result), "clock reversal accepted");
        require(!interpolate_gyro(before, &after, 100.02, 0.02, result), "unbracketed gyro accepted");
        std::cout << "PASS: finite-difference frame Jacobians, cross-covariance, floors, invalid data and gyro timing\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
