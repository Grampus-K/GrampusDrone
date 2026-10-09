#pragma once
#include "odometry_transform.hpp"
#include <cstdint>
#include <string>

namespace lio_to_mavros {
struct GuardOptions {
  double stable_seconds = 5.0, timeout = 0.5, max_age = 0.5, future_tolerance = 0.05;
  double clock_tolerance = 0.5, stationary_speed = 0.35, stationary_angular = 0.2;
  double stationary_drift = 0.5, stationary_angle = 0.1;
  double max_position = 1000, max_speed = 20, max_angular = 12;
  double max_step = 1, max_angle_step = 0.35, reference_yaw = 0;
  bool zero_position = true, zero_yaw = false;
  void validate() const {
    const double positive[] = {stable_seconds, timeout, max_age, future_tolerance,
      clock_tolerance, stationary_speed, stationary_angular, stationary_drift,
      stationary_angle, max_position, max_speed, max_angular, max_step, max_angle_step};
    for (double v : positive)
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("guard limits must be finite and positive");
    if (!std::isfinite(reference_yaw)) throw std::invalid_argument("invalid reference yaw");
  }
};

inline double rotationDistance(const Rotation &a, const Rotation &b) {
  return Eigen::AngleAxisd(a.transpose()*b).angle();
}

// Times passed here are seconds: ROS time for age, steady_clock for arrival.
// Faults latch until process restart; no automatic reference reset.
class OdometryGuard {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OdometryGuard(GuardOptions options) : options_(options) { options_.validate(); }
  bool ready() const { return ready_ && reason_.empty(); }
  bool failed() const { return !reason_.empty(); }
  const std::string &reason() const { return reason_; }
  void fault(const std::string &reason) { if (!failed()) reason_ = reason; ready_ = false; }
  void tick(double now, double steady) {
    if (!std::isfinite(now) || !std::isfinite(steady)) { fault("invalid clock"); return; }
    if (clock_seen_ && (steady < clock_steady_ ||
        std::abs((now-clock_ros_) - (steady-clock_steady_)) > options_.clock_tolerance))
      fault("system clock jumped");
    clock_seen_ = true; clock_ros_ = now; clock_steady_ = steady;
    if (seen_ && steady-arrival_ > options_.timeout) {
      if (ready_) fault("odometry timed out; restart on ground");
      else anchor_seen_ = false;
    }
    if (ready_ && now - stamp_*1e-9 > options_.max_age) fault("latest measurement expired");
  }
  bool accept(const Sample &input, std::uint64_t stamp, std::uint32_t sequence,
              double now, double steady, const Transform &i_b, Sample &output) {
    tick(now, steady);
    if (failed()) return false;
    try {
      Sample base = transformSample(input, i_b, Transform());
      if (stamp == 0 || now - stamp*1e-9 > options_.max_age ||
          stamp*1e-9 - now > options_.future_tolerance)
        throw std::invalid_argument("measurement timestamp is zero, stale or future");
      if (base.pose.translation.norm() > options_.max_position ||
          base.linear.norm() > options_.max_speed || base.angular.norm() > options_.max_angular)
        throw std::invalid_argument("position, linear or angular speed outside limits");
      if (seen_) {
        if (stamp < stamp_) throw std::invalid_argument("measurement time moved backwards");
        if (stamp == stamp_) {
          if ((base.pose.translation-previous_.pose.translation).norm() > 1e-9 ||
              rotationDistance(base.pose.rotation,previous_.pose.rotation) > 1e-9 ||
              (base.linear-previous_.linear).norm() > 1e-9 ||
              (base.angular-previous_.angular).norm() > 1e-9 ||
              (base.pose_covariance-previous_.pose_covariance).norm() > 1e-9 ||
              (base.twist_covariance-previous_.twist_covariance).norm() > 1e-9)
            throw std::invalid_argument("different measurements share a timestamp");
          return false;  // Do not refresh arrival time, reference or health.
        }
        if (sequence < sequence_ && !(sequence_ > 0xfffffff0u && sequence < 16u))
          throw std::invalid_argument("publisher sequence restarted");
        const double dt = (stamp-stamp_)*1e-9;
        if ((base.pose.translation-previous_.pose.translation).norm() > options_.max_step+options_.max_speed*dt ||
            rotationDistance(base.pose.rotation,previous_.pose.rotation) > options_.max_angle_step+options_.max_angular*dt)
          throw std::invalid_argument("odometry pose jumped");
      }
      previous_ = base; stamp_ = stamp; sequence_ = sequence; arrival_ = steady; seen_ = true;
      if (!ready_) {
        if (!anchor_seen_ || base.linear.norm() > options_.stationary_speed ||
            base.angular.norm() > options_.stationary_angular ||
            (base.pose.translation-anchor_.translation).norm() > options_.stationary_drift ||
            rotationDistance(base.pose.rotation,anchor_.rotation) > options_.stationary_angle) {
          anchor_ = base.pose; anchor_seen_ = true; stable_since_ = steady;
          return false;
        }
        if (steady-stable_since_ < options_.stable_seconds) return false;
        reference_ = referenceTransform(base.pose, options_.reference_yaw,
                                        options_.zero_position, options_.zero_yaw);
        ready_ = true;
      }
      output = transformSample(input, i_b, reference_);
      return true;
    } catch (const std::invalid_argument &error) { fault(error.what()); return false; }
  }
 private:
  GuardOptions options_;
  bool ready_{false}, seen_{false}, anchor_seen_{false}, clock_seen_{false};
  std::string reason_;
  Transform anchor_, reference_;
  Sample previous_;
  std::uint64_t stamp_{0};
  std::uint32_t sequence_{0};
  double arrival_{0}, stable_since_{0}, clock_ros_{0}, clock_steady_{0};
};
}  // namespace lio_to_mavros
