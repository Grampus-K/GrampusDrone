#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace ego_planner {

    /**
     * @brief EGO-Planner的有限状态机
     * 
     * 该类实现了EGO-Planner的核心状态机逻辑，包括:
     * - 初始化
     * - 等待目标
     * - 轨迹生成
     * - 轨迹重规划
     * - 轨迹执行
     * - 紧急停止
     */
    class EGOReplanFSM {
    public:
        EGOReplanFSM() {}
        ~EGOReplanFSM() {}

        /**
         * @brief 初始化FSM
         * @param nh ROS节点句柄
         */
        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        /* 状态定义 */
        enum FSM_EXEC_STATE {
            INIT,           // 初始化状态
            WAIT_TARGET,    // 等待目标状态
            GEN_NEW_TRAJ,  // 生成新轨迹状态
            REPLAN_TRAJ,   // 重新规划状态
            EXEC_TRAJ,     // 执行轨迹状态
            EMERGENCY_STOP  // 紧急停止状态
        };

        enum TARGET_TYPE {
            MANUAL_TARGET = 1,  // 手动选择目标
            PRESET_TARGET = 2,  // 预设目标
            REFENCE_PATH = 3    // 参考路径
        };

        /* 规划工具 */
        EGOPlannerManager::Ptr planner_manager_;
        PlanningVisualization::Ptr visualization_;
        traj_utils::DataDisp data_disp_;

        /* 参数 */
        int target_type_;
        double no_replan_thresh_, replan_thresh_;
        double waypoints_[50][3];
        int waypoint_num_, wpt_id_;
        double planning_horizen_;
        double emergency_time_;
        bool flag_realworld_experiment_;
        bool enable_fail_safe_;
        bool enable_ground_height_measurement_;
        bool flag_escape_emergency_;

        /* 状态标志 */
        bool have_trigger_, have_target_, have_odom_;
        bool have_new_target_, have_recv_pre_agent_;
        bool touch_goal_;
        FSM_EXEC_STATE exec_state_;
        int continously_called_times_{0};

        /* 状态变量 */
        Eigen::Vector3d start_pt_, start_vel_, start_acc_;      // 起始状态
        Eigen::Vector3d final_goal_;                            // 目标状态
        Eigen::Vector3d local_target_pt_, local_target_vel_;    // 局部目标状态
        Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;        // 里程计状态
        std::vector<Eigen::Vector3d> wps_;

        /* ROS相关 */
        ros::NodeHandle node_;
        ros::Timer exec_timer_, safety_timer_;
        ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_;
        ros::Subscriber broadcast_ploytraj_sub_;
        ros::Publisher poly_traj_pub_, data_disp_pub_;
        ros::Publisher broadcast_ploytraj_pub_, heartbeat_pub_, ground_height_pub_;

        /* 状态机函数 */
        void execFSMCallback(const ros::TimerEvent &e);
        void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
        void printFSMExecState();
        std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

        /* 安全检查 */
        void checkCollisionCallback(const ros::TimerEvent &e);
        bool callEmergencyStop(Eigen::Vector3d stop_pos);

        /* 轨迹规划 */
        bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
        bool planFromGlobalTraj(const int trial_times = 1);
        bool planFromLocalTraj(const int trial_times = 1);
        bool planNextWaypoint(const Eigen::Vector3d next_wp);
        bool modifyInCollisionFinalGoal();

        /* 回调函数 */
        void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        
        /* 轨迹消息转换 */
        void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);

        /* 辅助函数 */
        bool measureGroundHeight(double &height);
    };

} // namespace ego_planner

#endif