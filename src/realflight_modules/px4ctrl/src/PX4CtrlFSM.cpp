#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace std;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t &param_, LinearControl &controller_) : param(param_), controller(controller_) /*, thrust_curve(thrust_curve_)*/
{
	state = READY_TO_FLY;
	hover_pose.setZero();
}

void PX4CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	Desired_State_t des(odom_data);
	static bool traj_trigger_sent = false; // for publish_trigger once

	// STEP1: state machine runs
	switch (state)
	{
	case READY_TO_FLY:
	{
		if (!odom_is_received(now_time))
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No odom!");
			break;
		}
		if (cmd_is_received(now_time))
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. You are sending commands. Stop sending commands now!");
			break;
		}
		if (!rc_is_received(now_time)) // Check this only if RC is connected.
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. No RC received!");
			break;
		}
		if (odom_data.v.norm() > 0.1)
		{
			ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
			break;
		}

		static ros::Time request_POSCTL = ros::Time::now();
		if (state_data.current_state.mode != "POSCTL" 
		&& (ros::Time::now() - request_POSCTL > ros::Duration(2.0))) {
			setMode("POSCTL");
			request_POSCTL = ros::Time::now();
		} 

		// --------- 触发条件：命令 or RC ---------
		const bool cmd_takeoff = (takeoff_land_data.triggered &&
								  takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF);
		const bool rc_takeoff = (rc_data.throttle_up() && state_data.current_state.armed);

		// RC 会把 takeoff_land_data.triggered 置位，因此这里不需要单独 rc_takeoff 变量；
		// 但为避免重复 arm，这里只在尚未通过 RC 解锁时才调用 armVehicle(true)。
		if (cmd_takeoff || rc_takeoff) 
		{
			if (!setMode("OFFBOARD")) {
				ROS_ERROR("[px4ctrl] OFFBOARD switch failed. Abort takeoff.");
				break;
			}
			if (!state_data.current_state.armed) { // 若不是刚才 RC 解锁的，再去 arm
				if (!armVehicle(true)) {
					ROS_ERROR("[px4ctrl] Arming failed. Abort takeoff.");
					break;
				}
			}

			state = AUTO_TAKEOFF;
			set_start_pose_for_takeoff_land(odom_data);
			ROS_INFO("\033[32m[px4ctrl] READY_TO_FLY(L1) --> AUTO_TAKEOFF\033[32m");

			traj_trigger_sent = false;
		}

		break;
	}

	case AUTO_TAKEOFF:
	{
		//到达指定高度
		if (odom_data.p(2) >= (takeoff_land.start_pose(2) + param.takeoff_land.height)) 
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");
		}
		else
		{
			des = get_takeoff_land_des();
		}

		break;
	}

	case AUTO_HOVER:
	{
		if (cmd_is_received(now_time))//收到了发来的轨迹
		{
			state = CMD_CTRL;
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
			
		}
		else if (takeoff_land_data.triggered && takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND)
		{

			state = AUTO_LAND;
			set_start_pose_for_takeoff_land(odom_data);
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND\033[32m");


		}
		else if(!rc_data.check_centered())
		{
			state = RC_CTRL;
			ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> RC_CTRL\033[32m");
		}
		else
		{
			des = get_hover_des();

			if(!traj_trigger_sent) {
                publish_trigger(odom_data.msg);
                traj_trigger_sent = true;
            }
		}

		break;
	}

	case RC_CTRL:
	{
		if(rc_data.check_centered())
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("\033[32m[px4ctrl] RC_CTRL(L2) --> AUTO_HOVER\033[32m");
		}
		else if(extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND && 
				state_data.current_state.armed)
		{//用于手动降落
			state = READY_TO_FLY;
			ROS_INFO("\033[32m[px4ctrl] RC_CTRL(L2) --> READY_TO_FLY\033[32m");
		}
		else
		{
			des = get_rc_des();
		}

		break;
	}

	case CMD_CTRL:
	{
		if (!cmd_is_received(now_time))
		{
			state = AUTO_HOVER;
			set_hov_with_odom();
			ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
		}
		else if(!rc_data.check_centered())
		{
			state = RC_CTRL;
            ROS_INFO("\033[32m[px4ctrl] CMD_CTRL(L3) --> RC_CTRL\033[0m");
		}
		else
		{
			des = get_cmd_des();
		}

		break;
	}

	case AUTO_LAND:
	{
        // 持续检测是否已落地
        if (extended_state_data.current_extended_state.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
            if (armVehicle(false)) {
                    ROS_INFO("[px4ctrl] disarmed after landing");
            }
			ROS_INFO("\033[32m[px4ctrl] AUTO_LAND(L3) --> READY_TO_FLY\033[0m");
            state = READY_TO_FLY;
        }
		else
		{
			des = get_takeoff_land_des();
		}

		break;
	}

	default:
		break;
	}

	// STEP2: solve and update new control commands
	// debug_msg = controller.velocityControl(des, odom_data, imu_data, u, state);
	debug_msg = controller.velocityControl(des, odom_data, u, state);
	debug_msg.header.stamp = now_time;
	debug_pub.publish(debug_msg);
	
	// STEP3: publish control commands to mavros
    publish_velocity_ctrl(u, now_time);
	
	// STEP4: Clear flags beyound their lifetime
	takeoff_land_data.triggered = false;
}

Desired_State_t PX4CtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t PX4CtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.a = cmd_data.a;
	des.j = cmd_data.j;
	des.yaw = cmd_data.yaw;
	des.yaw_rate = cmd_data.yaw_rate;

	return des;
}

Desired_State_t PX4CtrlFSM::get_rc_des()
{
	Desired_State_t des;
	des.p = Eigen::Vector3d(0, 0, 0);
	des.v = Eigen::Vector3d(2.0 * rc_data.ch[1] * (param.rc_reverse.pitch    ? 1 : -1), 
							2.0 * rc_data.ch[0] * (param.rc_reverse.roll     ? 1 : -1), 
							1.0 * rc_data.ch[2] * (param.rc_reverse.throttle ? 1 : -1));
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = 0.0;
	des.yaw_rate = 1.5 * rc_data.ch[3] * (param.rc_reverse.yaw ? 1 : -1);

	return des;
}

Desired_State_t PX4CtrlFSM::get_takeoff_land_des()
{
	Desired_State_t des;
	des.p = takeoff_land.start_pose.head<3>();
	des.v = Eigen::Vector3d(0, 0, param.takeoff_land.speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_quaternion(odom_data.q);
}

void PX4CtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land.start_pose.head<3>() = odom_data.p;
	takeoff_land.start_pose(3) = get_yaw_from_quaternion(odom_data.q);
}

bool PX4CtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

bool PX4CtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - bat_data.rcv_stamp).toSec() < param.msg_timeout.bat;
}

bool PX4CtrlFSM::recv_new_odom()
{
	if (odom_data.recv_new_msg)
	{
		odom_data.recv_new_msg = false;
		return true;
	}

	return false;
}

void PX4CtrlFSM::publish_velocity_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::PositionTarget msg;

	msg.header.stamp = stamp;
	msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

	msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX  |
					mavros_msgs::PositionTarget::IGNORE_PY  |
					mavros_msgs::PositionTarget::IGNORE_PZ  |
					mavros_msgs::PositionTarget::IGNORE_AFX |
					mavros_msgs::PositionTarget::IGNORE_AFY |
					mavros_msgs::PositionTarget::IGNORE_AFZ |
					mavros_msgs::PositionTarget::FORCE      |
					mavros_msgs::PositionTarget::IGNORE_YAW;

	msg.velocity.x = u.velocity.x();
	msg.velocity.y = u.velocity.y();
	msg.velocity.z = u.velocity.z();
	msg.yaw_rate = u.yaw_rate;

	local_ctrl_pub.publish(msg);
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::setMode(const std::string& mode) {
	mavros_msgs::SetMode set_mode;
	set_mode.request.custom_mode = mode;
	
	if (set_FCU_mode_srv.call(set_mode) && set_mode.response.mode_sent) {
		ROS_INFO_STREAM("switch to " << mode << " mode");
		return true;
	}
	ROS_ERROR("Switch Mode rejected by PX4!");
	return false;
}

bool PX4CtrlFSM::armVehicle(bool arm) {
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	
	if (arming_client_srv.call(arm_cmd) && arm_cmd.response.success) {
		ROS_INFO(arm ? "vehicle armd" : "vehicle disarmd");
		return true;
	}
	return false;
}

void PX4CtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
