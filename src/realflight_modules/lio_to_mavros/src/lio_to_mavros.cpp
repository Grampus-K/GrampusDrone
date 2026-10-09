#include <memory>
#include "lio_to_mavros/legacy_pose_gate.hpp"
#include "lio_to_mavros/full_odometry_bridge.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_to_mavros");
  ros::NodeHandle nh, priv("~");
  try {
    std::string mode; bool preview;
    priv.param<std::string>("output_mode",mode,"vision_pose");
    priv.param("odometry_preview",preview,false);
    if (mode!="vision_pose" && mode!="odometry")
      throw std::invalid_argument("output_mode must be vision_pose or odometry");
    if (mode=="odometry" && preview)
      throw std::invalid_argument("preview is only available with vision_pose");
    if (nh.resolveName("/mavros/vision_pose/pose")==nh.resolveName("/mavros/odometry/out"))
      throw std::invalid_argument("formal output topics must not alias through remapping");
    priv.setParam("output_mode",mode);
    std::unique_ptr<lio_to_mavros::LegacyPoseGate> legacy;
    std::unique_ptr<lio_to_mavros::FullOdometryBridge> full;
    if (mode=="vision_pose") legacy.reset(new lio_to_mavros::LegacyPoseGate());
    if (mode=="odometry" || preview)
      full.reset(new lio_to_mavros::FullOdometryBridge(nh,priv,preview));
    ros::WallRate rate(30);
    while(ros::ok()) {
      // Check timeouts before processing queued measurements.
      if (full) full->tick();
      ros::spinOnce();
      if (legacy) legacy->tick();
      rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_FATAL("lio_to_mavros configuration error: %s",e.what());
    return 1;
  }
  return 0;
}
