#include <chrono>
#include <cmath>
#include <cstdlib>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

double distance(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
  const double x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
  return std::sqrt(x * x + y * y + z * z);
}

class OdometryGate {
 public:
  OdometryGate() : private_nh_("~") {
    private_nh_.param("stable_seconds", stable_seconds_, 5.0);
    private_nh_.param("odom_timeout", odom_timeout_, 0.5);
    private_nh_.param("max_stamp_error", max_stamp_error_, 2.0);
    private_nh_.param("stationary_speed", stationary_speed_, 0.35);
    private_nh_.param("stationary_drift", stationary_drift_, 0.5);
    private_nh_.param("max_speed", max_speed_, 20.0);
    private_nh_.param("max_step", max_step_, 1.0);
    private_nh_.param("max_position", max_position_, 1000.0);
    private_nh_.param("auto_restart_before_first_publish", auto_restart_, false);
    private_nh_.param("max_restart_attempts", max_restarts_, 3);
    private_nh_.param("restart_cooldown", restart_cooldown_, 15.0);
    private_nh_.param("clock_step_tolerance", clock_step_tolerance_, 0.5);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    health_pub_ = private_nh_.advertise<std_msgs::Bool>("healthy", 1, true);
    odom_sub_ = nh_.subscribe("/Odometry", 5, &OdometryGate::onOdometry,
                              this, ros::TransportHints().tcpNoDelay());
    state_sub_ = nh_.subscribe("/mavros/state", 5, &OdometryGate::onState,
                               this);
    reportHealth(false);
  }

  void run() {
    ros::WallRate rate(30);
    while (ros::ok()) {
      ros::spinOnce();
      const ros::WallTime now = ros::WallTime::now();
      checkSystemClock(ros::Time::now());
      if (mode_ == Mode::READY &&
          (now - last_odom_at_).toSec() > odom_timeout_) {
        fault("odometry timed out", false);
      }
      if (mode_ == Mode::FAULT) maybeRestart(now);
      if (mode_ == Mode::READY) publishPose(now);
      rate.sleep();
    }
  }

 private:
  enum class Mode { WAITING, READY, FAULT };

  void onState(const mavros_msgs::State::ConstPtr &state) {
    connected_ = state->connected;
    armed_ = state->armed;
    if (armed_) ever_armed_ = true;
    last_state_at_ = ros::WallTime::now();
  }

  void checkSystemClock(const ros::Time &ros_now) {
    const auto steady_now = std::chrono::steady_clock::now();
    if (have_clock_sample_) {
      const double ros_elapsed = (ros_now - previous_ros_time_).toSec();
      const double steady_elapsed =
          std::chrono::duration<double>(steady_now - previous_steady_time_).count();
      if (std::abs(ros_elapsed - steady_elapsed) > clock_step_tolerance_) {
        fault("system clock jumped while the stack was running", false);
      }
    }
    previous_ros_time_ = ros_now;
    previous_steady_time_ = steady_now;
    have_clock_sample_ = true;
  }

  void onOdometry(const nav_msgs::Odometry::ConstPtr &msg) {
    if (mode_ == Mode::FAULT) return;

    const ros::WallTime now = ros::WallTime::now();
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;
    const auto &v = msg->twist.twist.linear;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w) || !std::isfinite(v.x) || !std::isfinite(v.y) ||
        !std::isfinite(v.z)) {
      fault("non-finite odometry", true);
      return;
    }

    const double position = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    const double qnorm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (position > max_position_ || speed > max_speed_ ||
        qnorm < 0.5 || qnorm > 1.5) {
      fault("position, velocity or orientation outside limits", true);
      return;
    }

    if (msg->header.stamp.isZero() ||
        std::abs((ros::Time::now() - msg->header.stamp).toSec()) >
            max_stamp_error_) {
      fault("odometry timestamp disagrees with system clock", false);
      return;
    }

    if (have_odom_) {
      const double dt = (msg->header.stamp - latest_.header.stamp).toSec();
      if (dt <= 0) {
        fault("odometry timestamp moved backwards", false);
        return;
      }
      if (distance(p, latest_.pose.pose.position) >
          max_step_ + max_speed_ * dt) {
        fault("odometry position jumped", true);
        return;
      }
    }

    latest_ = *msg;
    have_odom_ = true;
    last_odom_at_ = now;
    if (mode_ != Mode::WAITING) return;

    if (!have_anchor_ || speed > stationary_speed_ ||
        distance(p, anchor_.pose.pose.position) > stationary_drift_) {
      anchor_ = *msg;
      have_anchor_ = true;
      stable_since_ = now;
      ROS_WARN_THROTTLE(3.0, "Waiting for stationary FAST-LIO odometry");
      return;
    }

    if ((now - stable_since_).toSec() >= stable_seconds_) {
      reference_ = msg->pose.pose;
      mode_ = Mode::READY;
      reportHealth(true);
      ROS_INFO("FAST-LIO odometry gate opened after %.1f stationary seconds",
               stable_seconds_);
    }
  }

  void fault(const std::string &reason, bool restartable) {
    if (mode_ == Mode::FAULT) return;
    mode_ = Mode::FAULT;
    restartable_ = restartable;
    fault_reason_ = reason;
    fault_at_ = ros::WallTime::now();
    reportHealth(false);
    ROS_ERROR("FAST-LIO odometry gate CLOSED: %s", reason.c_str());
  }

  void maybeRestart(const ros::WallTime &now) {
    if (!restartable_ || !auto_restart_ || ever_published_ || ever_armed_ ||
        !connected_ || armed_ || last_state_at_.isZero() ||
        (now - last_state_at_).toSec() > 1.0 ||
        restart_count_ >= max_restarts_ ||
        (now - fault_at_).toSec() < 1.0 ||
        (!last_restart_at_.isZero() &&
         (now - last_restart_at_).toSec() < restart_cooldown_)) {
      return;
    }

    ++restart_count_;
    last_restart_at_ = now;
    ROS_WARN("Restarting /laserMapping before first vision publication "
             "(attempt %d/%d): %s",
             restart_count_, max_restarts_, fault_reason_.c_str());
    if (std::system("rosnode kill /laserMapping >/dev/null 2>&1") != 0) {
      ROS_ERROR("Could not stop /laserMapping; vision output remains blocked");
      return;
    }

    // The onboard launch respawns this node. Discard queued old measurements.
    odom_sub_.shutdown();
    ros::WallDuration(2.5).sleep();
    odom_sub_ = nh_.subscribe("/Odometry", 5, &OdometryGate::onOdometry,
                              this, ros::TransportHints().tcpNoDelay());
    have_odom_ = false;
    have_anchor_ = false;
    mode_ = Mode::WAITING;
    reportHealth(false);
  }

  void publishPose(const ros::WallTime &now) {
    if (!have_odom_ || (now - last_odom_at_).toSec() > odom_timeout_) return;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "base_link";
    pose.pose.position.x = latest_.pose.pose.position.x - reference_.position.x;
    pose.pose.position.y = latest_.pose.pose.position.y - reference_.position.y;
    pose.pose.position.z = latest_.pose.pose.position.z - reference_.position.z;
    tf2::Quaternion q_ref, q_now;
    tf2::fromMsg(reference_.orientation, q_ref);
    tf2::fromMsg(latest_.pose.pose.orientation, q_now);
    tf2::Quaternion relative = q_now * q_ref.inverse();
    relative.normalize();
    pose.pose.orientation = tf2::toMsg(relative);
    pose_pub_.publish(pose);
    ever_published_ = true;
  }

  void reportHealth(bool healthy) {
    std_msgs::Bool msg;
    msg.data = healthy;
    health_pub_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher pose_pub_, health_pub_;
  ros::Subscriber odom_sub_, state_sub_;
  nav_msgs::Odometry latest_, anchor_;
  geometry_msgs::Pose reference_;
  ros::WallTime last_odom_at_, last_state_at_, stable_since_, fault_at_;
  ros::WallTime last_restart_at_;
  Mode mode_{Mode::WAITING};
  bool have_odom_{false}, have_anchor_{false};
  bool connected_{false}, armed_{false}, ever_armed_{false};
  bool ever_published_{false}, restartable_{false}, auto_restart_{false};
  int restart_count_{0}, max_restarts_{3};
  double stable_seconds_, odom_timeout_, max_stamp_error_;
  double stationary_speed_, stationary_drift_, max_speed_, max_step_;
  double max_position_, restart_cooldown_;
  double clock_step_tolerance_{0.5};
  bool have_clock_sample_{false};
  ros::Time previous_ros_time_;
  std::chrono::steady_clock::time_point previous_steady_time_;
  std::string fault_reason_;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_to_mavros");
  OdometryGate gate;
  gate.run();
  return 0;
}
