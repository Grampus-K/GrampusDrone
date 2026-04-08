#include <plan_manage/planner_manager.h>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{
  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;
  }

  /** @brief 弹性重规划函数 - 在当前状态下重新规划轨迹 */
  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc, 
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel, 
      const bool flag_polyInit, const bool flag_randomPolyTraj, const bool touch_goal)
  {
      // 1. 初始化计时和参数
      const auto t_start = ros::Time::now();
      ros::Duration t_init, t_opt;
      cout << "\033[47;30m\n[" << t_start << "] Drone " << " replan \033[0m" << endl;
      ploy_traj_opt_->setIfTouchGoal(touch_goal);
      const double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

      // 2. 计算初始轨迹
      poly_traj::MinJerkOpt initMJO;
      if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt,
                           local_target_vel, flag_polyInit, flag_randomPolyTraj,
                           ts, initMJO)) {
          return false;
      }

      // 3. 获取并验证约束点
      Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
      vector<std::pair<int, int>> segments;
      if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) 
          == PolyTrajOptimizer::CHK_RET::ERR) {
          return false;
      }

      t_init = ros::Time::now() - t_start;

      // 4. 可视化初始路径
      {
          std::vector<Eigen::Vector3d> point_set;
          // point_set.reserve(cstr_pts.cols());
          for (int i = 0; i < cstr_pts.cols(); ++i) {
              point_set.push_back(cstr_pts.col(i));
          }
          visualization_->displayInitPathList(point_set, 0.2, 0);
      }

      // 5. 轨迹优化
      const auto t_opt_start = ros::Time::now();
      bool success = false;
      poly_traj::MinJerkOpt best_MJO;
      {
          // 获取初始轨迹信息
          poly_traj::Trajectory initTraj = initMJO.getTraj();
          const int piece_num = initTraj.getPieceNum();
          Eigen::MatrixXd all_pos = initTraj.getPositions();
          Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, piece_num - 1);
          
          // 设置头尾状态
          Eigen::Matrix<double, 3, 3> headState, tailState;
          headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
          tailState << initTraj.getJuncPos(piece_num), initTraj.getJuncVel(piece_num), initTraj.getJuncAcc(piece_num);
          
          // 执行优化
          double final_cost;
          success = ploy_traj_opt_->optimizeTrajectory(
              headState, tailState, innerPts, initTraj.getDurations(), final_cost);
          best_MJO = ploy_traj_opt_->getMinJerkOpt();
      }
      t_opt = ros::Time::now() - t_opt_start;

      // 6. 处理优化结果
      if (success) {
          // 更新统计信息
          static double sum_time = 0;
          static int count_success = 0;
          sum_time += (t_init + t_opt).toSec();
          count_success++;

          // 输出时间统计
          ROS_INFO("Time: \033[42m%.3fms, \033[0m init:%.3fms, optimize:%.3fms, avg=%.3fms\n",
                   (t_init + t_opt).toSec() * 1000, t_init.toSec() * 1000,
                   t_opt.toSec() * 1000, sum_time / count_success * 1000);

          // 设置本地轨迹并可视化
          setLocalTrajFromOpt(best_MJO, touch_goal);
          cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          visualization_->displayOptimalList(cstr_pts, 0);
          continous_failures_count_ = 0;
      } else {
          // 处理失败情况
          cstr_pts = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          visualization_->displayFailedList(cstr_pts, 0);
          continous_failures_count_++;
      }

      return success;
  }

  /** @brief 计算初始状态 - 只在reboundReplan中用到*/  
  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {
    static bool flag_first_call = true;

    // Case 1: 多项式初始化（首次调用或强制使用）
    if (flag_first_call || flag_polyInit)
    {
      flag_first_call = false;

      // 基础参数设置
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;         // 轨迹内部点
      Eigen::VectorXd piece_dur_vec;   // 每段轨迹持续时间
      int piece_nums;                  // 轨迹段数

      constexpr double init_of_init_totaldur = 2.0;  // 初始轨迹总时长
      headState << start_pt, start_vel, start_acc;   // 起始状态（位置、速度、加速度）
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();// 目标状态

      // 内部点生成方式
      if (!flag_randomPolyTraj)
      {
        // 常规初始化：单段直线轨迹
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur; // 单段持续时间
      }
      else// 随机扰动中间点
      {
        // 随机初始化：生成带扰动的中间点
        // 计算水平方向（与起点-目标连线垂直的水平方向）
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        // 计算垂直方向（与起点-目标连线和水平方向都垂直）
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();

        // 生成随机中间点，扰动幅度随连续失败次数衰减
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      // 生成初始轨迹（初级轨迹）
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      // ----------------------------------
      // 调整时间分配生成最终初始轨迹
      // ----------------------------------

      // 根据路径长度确定分段数（至少2段）
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      // 重新分配时间片（等分总时长）
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);

      // 从初级轨迹采样中间点
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }

      // 校验采样点数量
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }

      // ----------------------------------
      // 生成最终初始轨迹
      // ----------------------------------
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    // Case 2: 从历史轨迹初始化
    else 
    {
      // 检查是否存在有效全局轨迹
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("No valid global trajectory exists");
        return false;
      }

      // ----------------------------------
      // 时间系统计算
      // ----------------------------------
      // 计算已在局部轨迹上运行的时间
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      // 计算距离当前局部轨迹结束的剩余时间
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if (t_to_lc_end < 0)
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }

      // 计算到全局目标点的总剩余时间
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);

      // ----------------------------------
      // 轨迹分段参数设置
      // ----------------------------------
      // 根据路径长度确定分段数（至少2段）                    
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      // 初始化状态矩阵和持续时间
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      // ----------------------------------
      // 从历史轨迹采样中间点
      // ----------------------------------
      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          // 从本地轨迹获取点（时间转换为轨迹时间系）
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          // 从全局轨迹获取点（时间转换为全局时间系）
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1); // 移动到下一时间段
      }

      // ----------------------------------
      // 生成最终轨迹
      // ----------------------------------
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  /**
   * @brief 从全局轨迹中获取局部规划目标点 - 只在fsm中用到
   * 
   * @param[in] planning_horizen 规划视野距离（最远目标点距离）
   * 
   * 工作原理：
   * 1. 沿全局轨迹时间轴搜索，找到首个距离起点超过规划视野的点作为局部目标
   * 2. 若搜索到轨迹末端仍未满足距离条件，则将全局终点作为目标
   * 3. 根据目标点与终点的距离决定速度策略（急停或保持轨迹速度）
   */
  void EGOPlannerManager::getLocalTarget(
    const double planning_horizen, const Eigen::Vector3d &start_pt,
    const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
    Eigen::Vector3d &local_target_vel, bool &touch_goal) 
  {
    // 初始化标志位和轨迹时间参数
    touch_goal = false;
    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    // 计算时间步长（基于规划视野和最大速度的经验值）
    const double time_step = planning_horizen / 20 / pp_.max_vel_; // 将规划视野分成20段进行搜索

    // 沿时间轴搜索目标点 -------------------------------------------------
    double current_time = traj_.global_traj.glb_t_of_lc_tgt; // 从上次目标时间开始搜索
    bool found_target = false;

    // 遍历全局轨迹时间轴（从当前目标时间到轨迹结束时间）
    for (; current_time < traj_.global_traj.global_start_time + traj_.global_traj.duration;
        current_time += time_step) 
    {
        // 获取当前时间对应的轨迹位置（转换为轨迹相对时间）
        const double relative_time = current_time - traj_.global_traj.global_start_time;
        const Eigen::Vector3d candidate_pos = traj_.global_traj.traj.getPos(relative_time);

        // 计算当前位置与起点的距离
        const double dist_to_start = (candidate_pos - start_pt).norm();

        // 找到首个超出规划视野的点作为局部目标
        if (dist_to_start >= planning_horizen) 
        {
            local_target_pos = candidate_pos;
            traj_.global_traj.glb_t_of_lc_tgt = current_time; // 更新全局轨迹的局部目标时间
            found_target = true;
            break;
        }
    }

    // 处理轨迹末端情况 ---------------------------------------------------
    // 若搜索到轨迹末端仍未找到目标点（或剩余时间不足）
    const bool reach_global_end = (current_time - traj_.global_traj.global_start_time) >= 
                                  (traj_.global_traj.duration - 1e-5);
    if (!found_target || reach_global_end) 
    {
        local_target_pos = global_end_pt;
        traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
        touch_goal = true; // 标记已接触全局终点
    }

    // 设置局部目标速度策略 -----------------------------------------------
    // 计算当前目标点与全局终点的距离
    const double dist_to_goal = (global_end_pt - local_target_pos).norm();
    // 最大制动距离公式：v²/(2a)
    const double braking_distance = (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_);

    if (dist_to_goal < braking_distance) 
    {
        // 进入制动距离范围，速度置零准备停止
        local_target_vel = Eigen::Vector3d::Zero();
    } 
    else 
    {
        // 保持全局轨迹的对应速度
        const double relative_time = traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
        local_target_vel = traj_.global_traj.traj.getVel(relative_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal) {
    // 从优化器中提取生成的轨迹
    poly_traj::Trajectory optimized_traj = opt.getTraj();

    // 获取初始约束点集合（用于碰撞检测）
    // 参数说明: getCpsNumPrePiece()表示每段轨迹前需检查的约束点数量
    Eigen::MatrixXd constraint_points = opt.getInitConstraintPoints(getCpsNumPrePiece());

    // 存储待检查的轨迹点集合（分段结构）
    PtsChk_t check_points;

    // 计算需要检查的轨迹点：
    // 1. 使用two_thirds_id策略筛选约束点（保留前2/3的约束点）
    // 2. touch_goal标志决定是否在接近目标时调整检测策略
    bool calculation_success = ploy_traj_opt_->computePointsToCheck(
        optimized_traj,
        ConstraintPoints::two_thirds_id(constraint_points, touch_goal),
        check_points
    );

    /* 检查计算结果有效性（三重验证）：
     * 1. 轨迹点计算必须成功
     * 2. 至少存在一个检测段（防止空数据）
     * 3. 最后一个检测段必须有检测点（确保终点检测） */
    const bool valid_check_points = check_points.size() >= 1 && 
                                  check_points.back().size() >= 1;
    
    if (calculation_success && valid_check_points) {
        // 记录轨迹开始时间（用于后续时间相关计算）
        const double trajectory_start_time = ros::Time::now().toSec();
        
        // 更新本地轨迹数据：
        // 1. 优化后的轨迹
        // 2. 分段检测点集合
        // 3. 轨迹起始时间戳
        traj_.setLocalTraj(optimized_traj, check_points, trajectory_start_time);
    }

    // 返回轨迹点计算状态（无论是否成功更新本地轨迹）
    return calculation_success;
}

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  /** @brief 规划全局路径点 - 只在fsm中用到*/
  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;

    /* 处理中间路径点（inner points）*/
    Eigen::MatrixXd innerPts;
    if (waypoints.size() > 1)
    {
      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    /* 初始化轨迹参数 */
    globalMJO.reset(headState, tailState, waypoints.size());

    /* 时间分配策略 */
    double des_vel = pp_.max_vel_ / 1.5;// 初始期望速度（安全阈值）
    Eigen::VectorXd time_vec(waypoints.size());// 各段持续时间

    /* 双重时间分配策略（最多调整两次）*/
    for (int j = 0; j < 2; ++j)
    {
      // 计算各段持续时间（距离/期望速度）
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      // 生成最小加加速度轨迹
      globalMJO.generate(innerPts, time_vec);

      // 速度约束检查
      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      // 降低期望速度进行重试（指数退避策略）
      des_vel /= 1.5;
    }

    /* 设置全局轨迹 */
    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }
} // namespace ego_planner