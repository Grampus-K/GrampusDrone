#ifndef FAST_LIO_ODOMETRY_FULL_HPP
#define FAST_LIO_ODOMETRY_FULL_HPP

#include "odometry_covariance.hpp"
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>

namespace fast_lio_full
{
inline bool read_gyro(const sensor_msgs::Imu &message, const Options &options, GyroSample &sample)
{
    if (!options.valid() || message.angular_velocity_covariance[0] == -1.0) return false;
    sample.stamp = message.header.stamp.toSec();
    sample.value << message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            sample.covariance(row, col) = message.angular_velocity_covariance[row * 3 + col];
    // ROS all-zero means unknown. Explicitly configured per-sample floor is
    // also used when the driver supplies a smaller diagonal variance.
    if (!sample.value.allFinite() || !sanitize_covariance(sample.covariance)) return false;
    const double variance_floor = options.gyro_noise_stddev * options.gyro_noise_stddev;
    for (int i = 0; i < 3; ++i)
        sample.covariance(i, i) += std::max(0.0, variance_floor - sample.covariance(i, i));
    return std::isfinite(sample.stamp) && sample.stamp > 0.0;
}

// State is templated to leave the mathematical helper independent of ROS/IKFoM.
template<typename State>
bool make_odometry(const State &state, const StateCovariance &covariance,
                   const GyroSample &gyro, const ros::Time &stamp,
                   const Options &options, nav_msgs::Odometry &message)
{
    static_assert(State::DOF == kStateDof && decltype(state.pos)::IDX == kPosition &&
        decltype(state.rot)::IDX == kRotation && decltype(state.vel)::IDX == kVelocity &&
        decltype(state.bg)::IDX == kGyroBias, "Update odometry Jacobians for new IKFoM layout");
    const Eigen::Matrix3d rotation = state.rot.toRotationMatrix();
    const Eigen::Vector3d velocity_body = rotation.transpose() * state.vel;
    const Eigen::Vector3d angular_body = gyro.value - state.bg;
    if (!state.pos.allFinite() || !angular_body.allFinite() || !std::isfinite(gyro.stamp) ||
        stamp.isZero() || std::abs(gyro.stamp - stamp.toSec()) > 1e-6) return false;
    Covariance6 pose_covariance, twist_covariance;
    if (!transform_covariances(covariance, rotation, velocity_body, gyro.covariance,
                               options, pose_covariance, twist_covariance)) return false;
    const Eigen::Quaterniond quaternion(rotation);
    if (!quaternion.coeffs().allFinite()) return false;

    message = nav_msgs::Odometry();
    message.header.stamp = stamp;
    message.header.frame_id = "world";
    message.child_frame_id = "body";  // FAST-LIO internal IMU, NOT vehicle base_link
    message.pose.pose.position.x = state.pos.x();
    message.pose.pose.position.y = state.pos.y();
    message.pose.pose.position.z = state.pos.z();
    message.pose.pose.orientation.x = quaternion.x();
    message.pose.pose.orientation.y = quaternion.y();
    message.pose.pose.orientation.z = quaternion.z();
    message.pose.pose.orientation.w = quaternion.w();
    message.twist.twist.linear.x = velocity_body.x();
    message.twist.twist.linear.y = velocity_body.y();
    message.twist.twist.linear.z = velocity_body.z();
    message.twist.twist.angular.x = angular_body.x();
    message.twist.twist.angular.y = angular_body.y();
    message.twist.twist.angular.z = angular_body.z();
    for (int row = 0; row < 6; ++row)
        for (int col = 0; col < 6; ++col)
        {
            message.pose.covariance[row * 6 + col] = pose_covariance(row, col);
            message.twist.covariance[row * 6 + col] = twist_covariance(row, col);
        }
    return true;
}
} // namespace fast_lio_full
#endif
