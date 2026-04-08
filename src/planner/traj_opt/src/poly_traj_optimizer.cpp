#include "optimizer/poly_traj_optimizer.h"

using namespace std;

#define VERBOSE_OUTPUT false
#define PRINTF_COND(STR, ...) \
  if (VERBOSE_OUTPUT)         \
  printf(STR, __VA_ARGS__)

namespace ego_planner
{
  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      double &final_cost)
  {
    // ========== 输入检查 ==========
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    // ========== 初始化参数 ==========
    // 计时与统计变量
    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_unsafe, flag_success;

    // 轨迹参数配置
    t_now_ = ros::Time::now().toSec();
    piece_num_ = initT.size();
    jerkOpt_.reset(iniState, finState, piece_num_);
    variable_num_ = 4 * (piece_num_ - 1) + 1;

    // 初始化优化变量（虚拟时间映射）
    double x_init[variable_num_];
    memcpy(x_init, initInnerPts.data(), initInnerPts.size() * sizeof(x_init[0]));
    Eigen::Map<Eigen::VectorXd> Vt(x_init + initInnerPts.size(), initT.size());
    RealT2VirtualT(initT, Vt);// 虚拟时间映射

    // ========== LBFGS优化主循环 ==========
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;         // 内存大小
    lbfgs_params.max_iterations = 200;  // 最大迭代次数
    lbfgs_params.min_step = 1e-32;      // 最小步长
    lbfgs_params.past = 3;              // 历史梯度比较窗口
    lbfgs_params.delta = 1.0e-2;        // 收敛阈值
    do
    {
      // ---------- 重置优化状态 ----------
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_unsafe = false;
      flag_success = false;

      // ---------- 执行优化 ----------
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init,
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,  // 成本计算回调
          NULL,                                     // 梯度计算（自动推导）
          PolyTrajOptimizer::earlyExitCallback,     // 提前终止回调
          this,                                     // 用户数据
          &lbfgs_params);

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      // ---------- 结果后处理（检查碰撞） ----------
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;

        std::vector<std::pair<int, int>> segments_nouse;
        if (finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)
        {

          flag_success = true;
          PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
        }
        else
        {
          // A not-blank return value means collision to obstales
          flag_still_unsafe = true;
          restart_nums++;
          PRINTF_COND("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }

      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        PRINTF_COND("iter=%d, time(ms)=%f, rebound\n", iter_num_, time_ms);
      }
      else
      {
        PRINTF_COND("iter=%d, time(ms)=%f, error\n", iter_num_, time_ms);
        ROS_WARN_COND(VERBOSE_OUTPUT, "Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while ((flag_still_unsafe && restart_nums < 3) ||
             (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

    return flag_success;
  }

  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check)
  {
    // ========== 初始化阶段 ==========
    pts_check.clear();
    pts_check.resize(id_cps_end);

    const double RES = grid_map_->getResolution();        // 网格分辨率
    const double RES_2 = RES / 2.0;                       // 有效检测半径（采样点间最小间距）
    const Eigen::VectorXd durations = traj.getDurations();// 各轨迹段时长
    const double total_duration = durations.sum();        // 总轨迹时长

    // 时间轴初始化：记录每段轨迹的起始时间
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);


    // ========== 参数计算 ==========
    double t = 0.0;
    // 自适应步长计算：基于最大速度和轨迹密度的平衡
    double t_step = min(RES / max_vel_, durations.minCoeff() / max(cps_num_prePiece_, 1) / 1.5);
    // 轨迹追踪状态变量
    Eigen::Vector3d pt_last = traj.getPos(0.0);  // 当前采样位置
    int id_cps_curr = 0;                         // 当前控制点索引
    int id_piece_curr = 0;                       // 当前轨迹段索引

    while (true)
    {
      // ---------- 时间边界检查 ----------
      if (t > total_duration)
      {
        if (touch_goal_ && pts_check.size() > 0)
        {
          while (pts_check.back().size() == 0)
          {
            pts_check.pop_back();
          }

          if (pts_check.size() <= 0)
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }
          else
          {
            return true;
          }
        }
        else
        {
          ROS_ERROR("Failed to get points list to check (0x01). touch_goal_=%d, pts_check.size()=%d", touch_goal_, (int)pts_check.size());
          pts_check.clear();
          return false;
        }
      }

      // ---------- 下个关键点预测 ----------
      // 计算当前段下一个检查点触发时间（基于控制点分布）
      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / cps_num_prePiece_ * ((id_cps_curr + 1) - cps_num_prePiece_ * id_piece_curr);
      
      // ---------- 采样点处理 ----------
      if (t >= next_t_stp)
      { // 更新段和控制点索引
        if (id_cps_curr + 1 >= cps_num_prePiece_ * (id_piece_curr + 1))
        {
          ++id_piece_curr;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      Eigen::Vector3d pt = traj.getPos(t);

      // 判断是否需要记录新采样点：
      // 1. 初始段 | 2. 位置突变超过分辨率 | 3. 时间起点
      if (t < 1e-5 || pts_check[id_cps_curr].size() == 0 || (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &pt_data,
      const bool flag_first_init /*= true*/)
  {
    // 初始化基础数据
    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece_);
    poly_traj::Trajectory traj = pt_data.getTraj();

    // 首次初始化时设置控制点
    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** 根据障碍物分割初始轨迹 ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;               // 连续检测到障碍物的最小间隔帧数
    int in_id = -1, out_id = -1;                     // 障碍物段的起止索引
    int same_occ_state_times = ENOUGH_INTERVAL + 1;  // 占用状态持续计数器
    bool occ, last_occ = false;                      // 上一帧的占用状态
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;// 是否找到障碍物起始点和结束点
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_);       // 仅检查前2/3点

    // 生成用于碰撞检测的采样点集合
    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, i_end, pts_check))
    {
      return CHK_RET::ERR;
    }

    // 遍历轨迹点进行障碍物检测
    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);

        // 检测到新障碍物段开始
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        // 检测到障碍物段结束
        else if (!occ && last_occ)
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        // 当完整障碍物段被检测到时记录区间
        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /* // 无碰撞则直接返回 */
    if (segment_ids.size() == 0)
    {
      return CHK_RET::OBS_FREE;
    }

    // ======== 第二阶段：A*路径规划 ========
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      // 执行A*搜索
      ASTAR_RET ret = a_star_->AstarSearch(grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      // 处理A*搜索失败情况
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A corner case 2, I have never exeam it.");
      }
      else
      {
        ROS_WARN_COND(VERBOSE_OUTPUT, "A-star error, force return!");
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    // ======== 第三阶段：约束区间调整 ========
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    // int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("The drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("Local target in collision, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A corner case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR_COND(VERBOSE_OUTPUT, "A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN_COND(VERBOSE_OUTPUT, "Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  bool PolyTrajOptimizer::allowRebound(void) //zxzxzx
  {
    // criterion 1
    if (iter_num_ < 3)
      return false;

    // criterion 2
    double min_product = 1;
    for (int i = 3; i <= cps_.points.cols() - 4; ++i) // ignore head and tail
    {
      double product = ((cps_.points.col(i) - cps_.points.col(i - 1)).normalized()).dot((cps_.points.col(i + 1) - cps_.points.col(i)).normalized());
      if (product < min_product)
      {
        min_product = product;
      }
    }
    if (min_product < 0.87) // 30 degree
      return false;

    // all the criterion passed
    return true;
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    Eigen::VectorXd T(opt->piece_num_);

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_feas_var_costs(3); // 0:obs, 1:feas(v/a/j), 2:variance

    opt->VirtualT2RealT(t, T); // Unbounded virtual time to real time

    opt->jerkOpt_.generate(P, T); // Generate trajectory from {P,T}

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost

    opt->addPVAJGradCost2CT(gradT, obs_feas_var_costs, opt->cps_num_prePiece_);

    if (opt->allowRebound())
    {
      opt->roughlyCheckConstraintPoints(); // Trajectory rebound
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP); // Gradient prepagation

    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost); // Real time back to virtual time

    opt->iter_num_ += 1;

    return smoo_cost + obs_feas_var_costs.sum() + time_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  /*之前算出Cost对MINCO参数的梯度后仍然难以用常用的欧氏空间的求解器来进行优化求解，
    因为本身MINCO参数就是带约束的，假如用常规的优化思路来表达这种约束，
    那对于优化器来说这将是非常"稠密"的（直观上讲，每个优化点处都有一个约束，
    优化器很难同时处理这么多约束）。因此在得到Cost对MINCO参数的梯度后，还要进一步做一个微分映射（变量代换），
    将有约束的变量转化到无约束的变量中，并将优化的梯度传导到中，最终才能用于优化器求解。*/
  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
        // RT = 0.5*VT² + VT + 1 → 导数为 VT + 1
      }
      else
      {// RT = 1 / (0.5*VT² - VT + 1) → 导数为 (1 - VT) / den²
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }
      // 合并梯度：其他成本项 + 时间成本，并转换到虚拟时间
      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    //
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double costp, costv, costa, costj;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    // Eigen::MatrixXd constraint_pts(3, N * K + 1);

    // printf("A\n");

    // int innerLoop;
    double t = 0;
    for (int i = 0; i < N; ++i)
    {

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(1) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(1) += omg * step * costa;
        }

        if (feasibilityGradCostJ(jer, gradj, costj))
        {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * sna;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costj / K + step * gradViolaJt);
          costs(1) += omg * step * costj;
        }

        // printf("L\n");
        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
    }

    // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(2) += var;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
      double dist = ray.dot(cps_.direction[i_dp][j]);
      double dist_err = obs_clearance_ - dist;
      double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

      if (dist_err > 0)
      {
        ret = true;
        costp += wei_obs_ * pow(dist_err, 3);
        gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += wei_obs_soft_ * rsqr * (term - 1.0);
        gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }

  //连续性约束的惩罚

  /*
 * @brief 速度可行性成本计算
 * @param v[in] 当前速度向量（三维坐标）
 * @param gradv[out] 成本函数对速度的梯度 ∂K/∂v
 * @param costv[out] 计算的速度成本值
 * @explanation:
 * 1. 核心逻辑：当速度超过最大允许值时施加立方惩罚
 * 2. 数学形式：costv = wei_feas_mod_ * (‖v‖² - max_vel²)^3
 * 3. 梯度推导：gradv = wei_feas_mod_ * 6 * (‖v‖² - max_vel²)^2 * v
 */
  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJ(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    double jpen = j.squaredNorm() - max_jer_ * max_jer_;
    if (jpen > 0)
    {
      gradj = wei_feas_ * 6 * jpen * jpen * j;
      costj = wei_feas_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }

  /*
  * @brief 本成本函数通过最小化轨迹点间总平方距离来缩短路径长度
  * @param ps[in] 精细采样的轨迹点(finelycheckpoints)集合，用于距离计算（三维坐标，每列一个点）
  * @param gdp[out] 梯度矩阵（三维坐标梯度，列数对应N*K个采样点）
  * @param var[out] 计算的方差成本值
  * @explanation: 
  * 1. 核心思想：惩罚相邻点间距差异，使轨迹段长度均匀化
  * 2. 数学形式：基于四次方距离的加权平均（实际计算距离平方的方差）
  * 3. 梯度计算：通过链式法则推导每个轨迹点的梯度贡献
  */
  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        // cost = ((p_{i+1}[0]-p_{i}[0])^2+(p_{i+1}[1]-p_{i}[1])^2+(p_{i+1}[2]-p_{i}[2])^2))^2
        // so the gradient can be easily written
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }

} // namespace ego_planner