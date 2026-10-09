#pragma once
#include "odometry_guard.hpp"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <vector>

namespace lio_to_mavros {
inline double steadySeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline GuardOptions readOptions(const ros::NodeHandle &nh) {
  GuardOptions o;
  nh.param("full/stable_seconds", o.stable_seconds, o.stable_seconds);
  nh.param("full/timeout", o.timeout, o.timeout);
  nh.param("full/max_age", o.max_age, o.max_age);
  nh.param("full/future_tolerance", o.future_tolerance, o.future_tolerance);
  nh.param("full/clock_tolerance", o.clock_tolerance, o.clock_tolerance);
  nh.param("full/stationary_speed", o.stationary_speed, o.stationary_speed);
  nh.param("full/stationary_angular", o.stationary_angular, o.stationary_angular);
  nh.param("full/stationary_drift", o.stationary_drift, o.stationary_drift);
  nh.param("full/stationary_angle", o.stationary_angle, o.stationary_angle);
  nh.param("full/max_position", o.max_position, o.max_position);
  nh.param("full/max_speed", o.max_speed, o.max_speed);
  nh.param("full/max_angular", o.max_angular, o.max_angular);
  nh.param("full/max_step", o.max_step, o.max_step);
  nh.param("full/max_angle_step", o.max_angle_step, o.max_angle_step);
  nh.param("full/reference_yaw", o.reference_yaw, o.reference_yaw);
  nh.param("full/zero_position", o.zero_position, o.zero_position);
  nh.param("full/zero_yaw", o.zero_yaw, o.zero_yaw);
  o.validate();
  return o;
}

inline Vector3 readVector(const ros::NodeHandle &nh, const std::string &key) {
  std::vector<double> v;
  if (!nh.getParam(key,v) || v.size()!=3) throw std::invalid_argument(key+": expected three numbers");
  const Vector3 result(v[0],v[1],v[2]);
  if (!result.allFinite()) throw std::invalid_argument(key+": non-finite vector");
  return result;
}

inline Rotation readRotation(const ros::NodeHandle &nh, const std::string &key) {
  std::vector<double> v;
  if (!nh.getParam(key,v) || v.size()!=9) throw std::invalid_argument(key+": expected nine row-major numbers");
  Rotation r;
  for (int i=0;i<9;++i) r(i/3,i%3)=v[i];
  if (!validRotation(r)) throw std::invalid_argument(key+": invalid rotation");
  return r;
}

inline Sample fromMessage(const nav_msgs::Odometry &m) {
  Sample s;
  const auto &p=m.pose.pose.position; const auto &q=m.pose.pose.orientation;
  const auto &v=m.twist.twist.linear; const auto &w=m.twist.twist.angular;
  s.pose.translation=Vector3(p.x,p.y,p.z);
  s.pose.rotation=checkedQuaternion(Eigen::Quaterniond(q.w,q.x,q.y,q.z)).toRotationMatrix();
  s.linear=Vector3(v.x,v.y,v.z); s.angular=Vector3(w.x,w.y,w.z);
  for(int i=0;i<36;++i) {
    s.pose_covariance(i/6,i%6)=m.pose.covariance[i];
    s.twist_covariance(i/6,i%6)=m.twist.covariance[i];
  }
  // The full FAST-LIO source supplies positive floors. All-zero means unknown.
  if ((s.pose_covariance.diagonal().array()<=0).any() ||
      (s.twist_covariance.diagonal().array()<=0).any())
    throw std::invalid_argument("missing/unknown covariance diagonal");
  return s;
}

inline nav_msgs::Odometry toMessage(const Sample &s, const std_msgs::Header &header,
                                    const std::string &parent, const std::string &child) {
  nav_msgs::Odometry m; m.header=header; m.header.frame_id=parent; m.child_frame_id=child;
  auto &p=m.pose.pose.position; auto &q=m.pose.pose.orientation;
  auto &v=m.twist.twist.linear; auto &w=m.twist.twist.angular;
  p.x=s.pose.translation.x(); p.y=s.pose.translation.y(); p.z=s.pose.translation.z();
  const Eigen::Quaterniond orientation(s.pose.rotation);
  q.x=orientation.x(); q.y=orientation.y(); q.z=orientation.z(); q.w=orientation.w();
  v.x=s.linear.x(); v.y=s.linear.y(); v.z=s.linear.z();
  w.x=s.angular.x(); w.y=s.angular.y(); w.z=s.angular.z();
  for(int i=0;i<36;++i) {
    m.pose.covariance[i]=s.pose_covariance(i/6,i%6);
    m.twist.covariance[i]=s.twist_covariance(i/6,i%6);
  }
  return m;
}

class FullOdometryBridge {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FullOdometryBridge(const ros::NodeHandle &nh, const ros::NodeHandle &priv, bool preview)
      : nh_(nh), priv_(priv), preview_(preview), guard_(readOptions(priv)),
        listener_(buffer_,nh_,false) {
    priv_.param<std::string>("full/input_topic",input_,"/fast_lio/odometry_full");
    priv_.param<std::string>("full/mapping_namespace",mapping_ns_,"/mapping");
    priv_.param<std::string>("full/output_frame",parent_,"odom");
    priv_.param<std::string>("full/output_child_frame",child_,"base_link");
    std::string topic;
    if (preview_) {
      priv_.param<std::string>("full/preview_topic",topic,"/lio_to_mavros/odometry_preview");
      parent_="lio_preview_odom"; child_="lio_preview_base_link";
    } else priv_.param<std::string>("full/output_topic",topic,"/mavros/odometry/out");
    if (parent_.empty() || child_.empty() || parent_==child_ || parent_[0]=='/' || child_[0]=='/')
      throw std::invalid_argument("invalid output frames");
    const auto resolved=nh_.resolveName(topic);
    if (resolved==nh_.resolveName(input_) || resolved==nh_.resolveName("/Odometry") ||
        resolved==nh_.resolveName("/mavros/vision_pose/pose") ||
        (preview_ && (resolved.compare(0,8,"/mavros/")==0 ||
                     resolved==nh_.resolveName("/mavros/odometry/out"))))
      throw std::invalid_argument("unsafe output/preview topic or remap");
    b_l_.translation=readVector(priv_,"full/lidar_in_base/translation");
    b_l_.rotation=readRotation(priv_,"full/lidar_in_base/rotation");
    health_=priv_.advertise<std_msgs::Bool>(preview_?"preview/healthy":"healthy",1,true);
    status_=priv_.advertise<std_msgs::String>(preview_?"preview/status":"status",1,true);
    output_=nh_.advertise<nav_msgs::Odometry>(topic,5);
    priv_.setParam("full/input_topic",nh_.resolveName(input_));
    if (!preview_) priv_.setParam("full/output_topic",resolved);
    state_sub_=nh_.subscribe("/mavros/state",5,&FullOdometryBridge::onState,this);
    input_sub_=nh_.subscribe(input_,5,&FullOdometryBridge::onOdometry,this,ros::TransportHints().tcpNoDelay());
    report("WAITING: fixed FAST-LIO extrinsics and stationary measurements");
  }

  void tick() {
    const double steady=steadySeconds();
    guard_.tick(ros::Time::now().toSec(),steady);
    if (steady-last_config_check_>=1.0) { checkExtrinsics(); last_config_check_=steady; }
    if (!preview_ && guard_.ready() && !connected(steady)) guard_.fault("MAVROS disconnected or state expired");
    if (guard_.failed()) report("FAULT: "+guard_.reason());
  }

 private:
  void report(const std::string &text) {
    if (text==last_status_) return;
    last_status_=text;
    std_msgs::Bool h; h.data=guard_.ready(); health_.publish(h);
    std_msgs::String s; s.data=text; status_.publish(s);
    ROS_INFO_STREAM((preview_?"Odometry preview: ":"Odometry output: ")<<text);
  }
  bool connected(double now) const { return state_seen_ && state_.connected && now-state_at_<1.0; }
  void onState(const mavros_msgs::State::ConstPtr &m) {
    state_=*m; state_at_=steadySeconds(); state_seen_=true;
    if (!preview_ && !guard_.ready() && state_.armed) guard_.fault("cannot initialize reference while armed");
    if (!preview_ && guard_.ready() && !state_.connected) guard_.fault("MAVROS disconnected");
  }
  void checkExtrinsics() {
    if (guard_.failed()) return;
    try {
      bool enabled=true;
      if (!nh_.getParam(mapping_ns_+"/extrinsic_est_en",enabled)) {
        if (extrinsics_seen_) throw std::invalid_argument("FAST-LIO parameters disappeared");
        return;
      }
      if (enabled) throw std::invalid_argument("FAST-LIO online extrinsic estimation must be disabled");
      Transform i_l; i_l.rotation=readRotation(nh_,mapping_ns_+"/extrinsic_R");
      i_l.translation=readVector(nh_,mapping_ns_+"/extrinsic_T");
      const Transform next=imuFromBase(b_l_,i_l);
      if (extrinsics_seen_ && ((next.translation-i_b_.translation).norm()>1e-12 ||
                              (next.rotation-i_b_.rotation).norm()>1e-12))
        throw std::invalid_argument("FAST-LIO extrinsics changed; restart entire stack");
      i_b_=next; extrinsics_seen_=true;
    } catch (const std::invalid_argument &e) { guard_.fault(e.what()); }
  }
  bool checkMavrosTF() {
    std::string parent_target,child_target;
    nh_.param<std::string>("/mavros/odometry/fcu/odom_parent_id_des",parent_target,"odom");
    nh_.param<std::string>("/mavros/odometry/fcu/odom_child_id_des",child_target,"base_link");
    Rotation enu_ned; enu_ned<<0,1,0,1,0,0,0,0,-1;
    Rotation flu_frd=Rotation::Identity(); flu_frd(1,1)=-1; flu_frd(2,2)=-1;
    try {
      const auto p=buffer_.lookupTransform(parent_target+"_ned",parent_,ros::Time(0));
      const auto c=buffer_.lookupTransform(child_target+"_frd",child_,ros::Time(0));
      const geometry_msgs::Transform transforms[]={p.transform,c.transform};
      const Rotation expected[]={enu_ned,flu_frd};
      for(int i=0;i<2;++i) {
        const auto &q=transforms[i].rotation; const auto &t=transforms[i].translation;
        const Rotation r=checkedQuaternion(Eigen::Quaterniond(q.w,q.x,q.y,q.z)).toRotationMatrix();
        if (!Vector3(t.x,t.y,t.z).allFinite() || (r-expected[i]).norm()>1e-6 ||
            Vector3(t.x,t.y,t.z).norm()>1e-6)
          throw std::invalid_argument("MAVROS TF must be pure ENU/NED and FLU/FRD axis conversions");
      }
      return true;
    } catch(const tf2::TransformException &e) {
      if (guard_.ready()) guard_.fault("MAVROS TF lost");
      else report("WAITING: MAVROS TF axis conversions");
    } catch(const std::invalid_argument &e) { guard_.fault(e.what()); }
    return false;
  }
  void onOdometry(const nav_msgs::Odometry::ConstPtr &m) {
    tick();
    if (guard_.failed() || !extrinsics_seen_) return;
    const double steady=steadySeconds();
    if (!preview_) {
      if (!connected(steady)) { report("WAITING: fresh connected MAVROS state"); return; }
      if (!guard_.ready() && state_.armed) { guard_.fault("cannot initialize while armed"); tick(); return; }
      if (!checkMavrosTF()) { tick(); return; }
    }
    try {
      if (m->header.frame_id!="world" || m->child_frame_id!="body")
        throw std::invalid_argument("expected FAST-LIO world/body frames");
      Sample result;
      if (guard_.accept(fromMessage(*m),m->header.stamp.toNSec(),m->header.seq,
                        ros::Time::now().toSec(),steady,i_b_,result)) {
        output_.publish(toMessage(result,m->header,parent_,child_));
        report("READY: forwarding new scan measurements");
      } else if (!guard_.failed() && !guard_.ready()) report("WAITING: stationary full odometry");
    } catch(const std::invalid_argument &e) { guard_.fault(e.what()); }
    if (guard_.failed()) report("FAULT: "+guard_.reason());
  }
  ros::NodeHandle nh_,priv_;
  bool preview_,extrinsics_seen_{false},state_seen_{false};
  OdometryGuard guard_;
  Transform b_l_,i_b_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  ros::Publisher output_,health_,status_;
  ros::Subscriber input_sub_,state_sub_;
  mavros_msgs::State state_;
  double state_at_{0},last_config_check_{0};
  std::string input_,parent_,child_,mapping_ns_,last_status_;
};
}  // namespace lio_to_mavros
