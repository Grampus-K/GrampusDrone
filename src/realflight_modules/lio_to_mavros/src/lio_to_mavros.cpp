#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

nav_msgs::Odometry lio_odom;
geometry_msgs::PoseStamped ref_pose;
bool calibrated = false;
ros::Time calibration_start_time;

void lio_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    lio_odom = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lio_to_mavros");
    ros::NodeHandle nh;
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    ros::Subscriber lio_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, lio_cb,
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    geometry_msgs::PoseStamped pose;
    setlocale(LC_ALL, "");
    ros::Rate rate(30.0);
    
    // 校准参数
    double calibration_duration = 2.0; // 校准持续时间（秒）
    calibration_start_time = ros::Time::now();
    ROS_INFO("启动校准，等待%.1f秒...", calibration_duration);

    while (ros::ok()) {
        if (!calibrated) {  
            // 检查校准时间是否完成
            if ((ros::Time::now() - calibration_start_time).toSec() > calibration_duration) {
                // 确认收到有效数据
                if (lio_odom.header.stamp != ros::Time(0)) {
                    ref_pose.pose = lio_odom.pose.pose;
                    calibrated = true;
                    ROS_INFO("校准完成！参考位置已设置为初始位姿");
                } else {
                    ROS_WARN_THROTTLE(1, "等待有效的里程计数据...");
                }
            }
            // 继续等待校准
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // 处理并发布校准后的数据
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "base_link";

        // 计算相对位置
        pose.pose.position.x = lio_odom.pose.pose.position.x - ref_pose.pose.position.x;
        pose.pose.position.y = lio_odom.pose.pose.position.y - ref_pose.pose.position.y;
        pose.pose.position.z = lio_odom.pose.pose.position.z - ref_pose.pose.position.z;

        // 计算相对姿态（四元数运算）
        tf2::Quaternion q_ref, q_current, q_relative;
        tf2::fromMsg(ref_pose.pose.orientation, q_ref);
        tf2::fromMsg(lio_odom.pose.pose.orientation, q_current);
        q_relative = q_current * q_ref.inverse();
        q_relative.normalize();
        pose.pose.orientation = tf2::toMsg(q_relative);

        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
