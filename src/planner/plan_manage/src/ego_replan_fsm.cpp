#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{
  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    // 初始化状态变量
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    flag_escape_emergency_ = true;
    
    /*  fsm param  */
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, false);

    // 初始化主要模块
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    have_trigger_ = !flag_realworld_experiment_;

    exec_timer_    = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_  = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_      = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    waypoint_sub_  = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    heartbeat_pub_ = nh.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10);
  }

  /**
   * @brief FSM主循环回调函数
   */
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    // 停止定时器以避免阻塞
    exec_timer_.stop(); 

    // 发布心跳消息
    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    // 每500次循环打印一次状态
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    // 核心状态机逻辑
    switch (exec_state_) {
      case INIT: {
        if (!have_odom_) {
          break;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      case WAIT_TARGET: {
        if (!have_target_ || !have_trigger_) {
          break;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        break;
      }

      case GEN_NEW_TRAJ:
      {
        if (planFromGlobalTraj(10)){
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true;
        } else {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
        }
        break;
      }

      case REPLAN_TRAJ: {
        // 重新规划轨迹
        if (planFromLocalTraj(1)) {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        } else {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
      }

      case EXEC_TRAJ: {
        // 执行轨迹并检查是否需要重新规划
        LocalTrajData *info = &planner_manager_->traj_.local_traj;
        double t_cur = std::min(info->duration, 
                              ros::Time::now().toSec() - info->start_time);
        
        // 检查是否到达目标点
        bool touch_goal = (local_target_pt_ - final_goal_).norm() < 1e-2;
        
        // 检查是否接近当前轨迹终点
        const PtsChk_t* chk_ptr = &info->pts_chk;
        bool near_traj_end = (chk_ptr->size() >= 1 && chk_ptr->back().size() >= 1) ? 
                            chk_ptr->back().back().first - t_cur < emergency_time_ : false;

        // 处理不同情况
        if (modifyInCollisionFinalGoal()) {
          // 目标点在障碍物中,已修改目标点
        }
        else if (t_cur > info->duration - 1e-2 && touch_goal) {
          // 到达最终目标点
          have_target_ = false;
          have_trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else if (t_cur > replan_thresh_ || (!touch_goal && near_traj_end)) {
          // 需要重新规划
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
      }

      case EMERGENCY_STOP: {
        // 紧急停止处理
        if (flag_escape_emergency_) {
          callEmergencyStop(odom_pos_);
        }
        else if (enable_fail_safe_ && odom_vel_.norm() < 0.1) {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        flag_escape_emergency_ = false;
        break;
      }
    }

    // 发布数据显示消息
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

    // 重启定时器
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
 
    cout << "[" + pos_call + "]" << "Drone" << ", from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    // 使用const提升性能,使用static避免重复创建
    static const std::string state_str[] = {
      "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"
    };

    // 使用stringstream构建输出,避免多次cout
    std::stringstream ss;
    ss << "\r[FSM]: state: " << state_str[int(exec_state_)] 
       << ", Drone ";

    // 统一检查等待条件
    std::vector<std::string> waiting_for;
    if(!have_odom_) waiting_for.push_back("odom");
    if(!have_target_) waiting_for.push_back("target"); 
    if(!have_trigger_) waiting_for.push_back("trigger");

    // 只在有等待条件时才显示等待信息
    if(!waiting_for.empty()) {
      ss << ". Waiting for ";
      for(size_t i = 0; i < waiting_for.size(); ++i) {
        ss << waiting_for[i];
        if(i < waiting_for.size() - 1) ss << ",";
      }
    }

    ss << std::endl;
    std::cout << ss.str();
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // 检查地面高度(如果启用)
    if (enable_ground_height_measurement_) {
      double height;
      measureGroundHeight(height);
    }

    // 获取当前轨迹信息
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    
    // 如果在等待目标或轨迹无效,则返回
    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0) {
      return;
    }

    // 检查深度信息是否丢失
    if (map->getOdomDepthTimeout()) {
      ROS_ERROR("Depth information lost! Performing emergency stop");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
      return;
    }

    // 计算当前时间点
    const double t_cur = ros::Time::now().toSec() - info->start_time;
    const PtsChk_t pts_chk = info->pts_chk;

    // 定位当前轨迹段
    double t_temp = t_cur;
    int i_start = info->traj.locatePieceIdx(t_temp);
    if (i_start >= (int)pts_chk.size()) {
      return;
    }

    // 找到检查起始点 —— 跳过已执行部分
    size_t j_start = 0;
    for (; i_start < (int)pts_chk.size(); ++i_start) {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start) {
        if (pts_chk[i_start][j_start].first > t_cur) {
          goto start_collision_check;
        }
      }
    }

start_collision_check:
    // 确定检查终点
    const bool touch_goal = ((local_target_pt_ - final_goal_).norm() < 1e-2);
    const size_t i_end = touch_goal ? pts_chk.size() : pts_chk.size() * 3 / 4;

    /*--- 核心碰撞检测循环 ---*/
    for (size_t i = i_start; i < i_end; ++i) {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j) {
        const double t = pts_chk[i][j].first;
        const Eigen::Vector3d p = pts_chk[i][j].second;

        if (map->getInflateOccupancy(p)) {// 检查点是否在障碍物内
          // 尝试重新规划
          if (planFromLocalTraj()) {
            ROS_INFO("Replanning succeeded after collision detected. Progress: %.1f%%", t / info->duration * 100);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          
          // 重规划失败,根据情况执行紧急停止或重规划
          if (t - t_cur < emergency_time_) {
            ROS_WARN("Emergency stop triggered! Time remaining=%.2fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          } else {
            ROS_WARN("Current trajectory has collision, need to replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
      }
      j_start = 0;
    }
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj MINCO_msg;
    polyTraj2ROSMsg(poly_msg, MINCO_msg);
    poly_traj_pub_.publish(poly_msg);

    return true;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/)
  {
    // 设置起点状态
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    // 确定是否使用随机多项式初始化
    bool use_random_init = (timesOfConsecutiveStateCalls().first != 1);

    // 多次尝试规划
    for (int i = 0; i < trial_times; i++) {
      if (callReboundReplan(true, use_random_init)) return true;
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {
    // 获取当前轨迹信息
    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    // 获取当前状态
    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);

    // 尝试不同的规划策略
    // 1) 从当前轨迹重新规划
    if (callReboundReplan(false, false)) return true;

    // 2) 从多项式初始化重试
    if (callReboundReplan(true, false)) return true;

    // 3) 随机化多项式初始化多次尝试
    for (int i = 0; i < trial_times; i++) 
      if (callReboundReplan(true, true)) return true;
 
    return false;
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 获取局部目标点
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pt_, final_goal_,
        local_target_pt_, local_target_vel_,
        touch_goal_);

    // 执行弹性重规划
    bool plan_success = planner_manager_->reboundReplan(
        start_pt_, start_vel_, start_acc_,
        local_target_pt_, local_target_vel_,
        (have_new_target_ || flag_use_poly_init),
        flag_randomPolyTraj, touch_goal_);

    have_new_target_ = false;

    // 如果规划成功,发布轨迹
    if (plan_success) {
      traj_utils::PolyTraj poly_msg;
      traj_utils::MINCOTraj MINCO_msg;
      polyTraj2ROSMsg(poly_msg, MINCO_msg);
      poly_traj_pub_.publish(poly_msg);
    }

    return plan_success;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    // 规划到下一个航点的轨迹
    std::vector<Eigen::Vector3d> one_pt_wps{next_wp};
    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_, odom_vel_, Eigen::Vector3d::Zero(),
        one_pt_wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (!success) {
      ROS_ERROR("Unable to generate global trajectory!");
      return false;
    }

    // 更新目标点与可视化轨迹采样
    final_goal_ = next_wp;
    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
    vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
    }

    have_target_ = true;
    have_new_target_ = true;

    //如果当前不在 WAIT_TARGET，需要等待直到 EXEC_TRAJ，然后触发 REPLAN
    if (exec_state_ != WAIT_TARGET){
      while (exec_state_ != EXEC_TRAJ){
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
    }

    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    return true;
  }

  bool EGOReplanFSM::modifyInCollisionFinalGoal()
  {
    // 如果终点不在障碍物中，则无需修改
    if (!planner_manager_->grid_map_->getInflateOccupancy(final_goal_)) {
      return false;
    }

    // 保存原始目标点用于日志
    const Eigen::Vector3d orig_goal = final_goal_;
    
    // 计算时间步长
    const double t_step = planner_manager_->grid_map_->getResolution() / planner_manager_->pp_.max_vel_;
    const double duration = planner_manager_->traj_.global_traj.duration;

    // 从终点往回找第一个无碰撞点
    for (double t = duration; t > 0; t -= t_step) {
      // 获取轨迹上的点
      Eigen::Vector3d pt = planner_manager_->traj_.global_traj.traj.getPos(t);
      
      // 检查该点是否无碰撞
      if (!planner_manager_->grid_map_->getInflateOccupancy(pt)) {
        // 尝试将该点设为新的目标点
        if (planNextWaypoint(pt)) {
          ROS_INFO("Modified collision waypoint from (%.3f, %.3f, %.3f) to (%.3f, %.3f, %.3f)",
                   orig_goal(0), orig_goal(1), orig_goal(2), 
                   final_goal_(0), final_goal_(1), final_goal_(2));
          return true;
        }
      }

      // 检查是否已经搜索到起点附近
      if (t <= t_step) {
        ROS_ERROR("Unable to find collision-free point on global trajectory");
        break;
      }
    }

    return false;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->goal[2] < -0.1)
      return;

    ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    if (planNextWaypoint(end_wp)){
      have_trigger_ = true;
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {
    // 获取轨迹数据
    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    // 填充多项式轨迹消息
    // poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5;  // 目前仅支持5阶多项式
    
    // 分配内存
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);

    // 填充每段轨迹的系数
    for (int i = 0; i < piece_num; ++i) {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }

    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if (planner_manager_->traj_.local_traj.pts_chk.size() < 3) // means planning have not started
    {
      return false;
    }

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if (ret == -1) // reach map bottom
        {
          return false;
        }
        if (ret == 1) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);

          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner