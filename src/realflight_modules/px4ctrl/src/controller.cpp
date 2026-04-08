#include "controller.h"

using namespace std;
using namespace uav_utils;

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

double LinearControl::wrapPi(double angle_rad)
{
  // 使用 fmod 实现稳定的包络：结果范围 (-pi, pi]
  static constexpr double kPi    = 3.14159265358979323846;
  static constexpr double kTwoPi = 6.28318530717958647692;

  double x = std::fmod(angle_rad + kPi, kTwoPi);
  if (x < 0) x += kTwoPi;
  return x - kPi;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param)
{

}

// READY_TO_FLY = 1, // px4ctrl is deactived. FCU is controled by the remote controller only
// AUTO_HOVER, 
// CMD_CTRL,	
// RC_CTRL,
// AUTO_TAKEOFF,
// AUTO_LAND
// quadrotor_msgs::Px4ctrlDebug 
// LinearControl::velocityControl(const Desired_State_t &des,
//     const Odom_Data_t &odom,
//     const Imu_Data_t &imu, 
//     Controller_Output_t &u, const uint8_t state)

quadrotor_msgs::Px4ctrlDebug 
LinearControl::velocityControl(const Desired_State_t &des,
    const Odom_Data_t &odom, Controller_Output_t &u, const uint8_t state)
{
    Eigen::Vector3d Kp,Kv,Kff;
    double Kyaw = 0.0;

    // if(state == 1 )
    // {
    //   Kp << 0.0, 0.0, 0.0;
    //   Kv << 0.0, 0.0, 0.0;
    //   Kff << 0.0, 0.0, 0.0;
    //   Kyaw = 0.0;
    // }
    // else if(state == 2)
    // {
    //   Kp << 1.0, 1.0, 1.0;
    //   Kv << 0.0, 0.0, 0.0;
    //   Kff << 0.0, 0.0, 0.0;
    //   Kyaw = 1.0;
    // }
    // else if(state == 3)
    // {
    //   Kp << 1.0, 1.0, 1.0;
    //   Kv << 0.1, 0.1, 0.1;
    //   Kff << 0.8, 0.8, 0.0;
    //   Kyaw = 1.0;
    // }
    // else if(state == 4)
    // {
    //   Kp << 0.0, 0.0, 0.0;
    //   Kv << 0.0, 0.0, 0.0;
    //   Kff << 1.0, 1.0, 1.0;
    //   Kyaw = 0.0;
    // }
    // else if(state == 5)
    // {
    //   Kp << 1.0, 1.0, 0.0;
    //   Kv << 0.0, 0.0, 0.0;
    //   Kff << 0.0, 0.0, 1.0;
    //   Kyaw = 1.0;
    // }
    // else if(state == 6)
    // {
    //   Kp << 1.0, 1.0, 0.0;
    //   Kv << 0.0, 0.0, 0.0;
    //   Kff << 0.0, 0.0, -1.0;
    //   Kyaw = 1.0;
    // }

    switch (state) {
      case 1: Kp.setZero(); Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;                  // READY_TO_FLY
      case 2: Kp = {1.0,1.0,1.0}; Kv = {0.0,0.0,0.0}; Kff.setZero(); Kyaw = 1.0; break;    // AUTO_HOVER
      case 3: Kp = {0.5,0.5,0.8}; Kv = {0.1,0.1,0.2}; Kff.setOnes(); Kyaw = 1.0; break;      // CMD_CTRL
      case 4: Kp.setZero(); Kv.setZero(); Kff.setOnes(); Kyaw = 0.0; break;                  // RC_CTRL（下游给 body）
      case 5: Kp = {0.2,0.2,0.0}; Kv = {0,0,0.05}; Kff = {0,0, 1}; Kyaw = 1.0; break;         // AUTO_TAKEOFF
      case 6: Kp = {0.2,0.2,0.0}; Kv = {0,0,0.0}; Kff = {0,0,-1}; Kyaw = 1.0; break;         // AUTO_LAND
      default: Kp.setZero(); Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;
    }

    // double yaw_odom = fromQuaternion2yaw(odom.q);
    double yaw_odom = get_yaw_from_quaternion(odom.q);
    double sin = std::sin(yaw_odom);
    double cos = std::cos(yaw_odom);

    Eigen::Vector3d feedforward;//这里需要把ego传过来的速度转为body系，因为发给px4的是body的速度
    if(state == 4)
    {
      feedforward = des.v ;
    }
    else
    {
      feedforward.x() =  des.v.x() * cos + des.v.y() * sin;
      feedforward.y() = -des.v.x() * sin + des.v.y() * cos;
      feedforward.z() =  des.v.z() ;
    }

    Eigen::Vector3d error_p;//同样转成body系
    error_p.x() =  (des.p.x() - odom.p.x()) * cos + (des.p.y() - odom.p.y()) * sin;
    error_p.y() = -(des.p.x() - odom.p.x()) * sin + (des.p.y() - odom.p.y()) * cos;
    error_p.z() =   des.p.z() - odom.p.z();

    //这里因为mavros传回来的里程计速度是body系的，所以odom.v没有做变换
    u.velocity = Kff.asDiagonal() * feedforward + Kp.asDiagonal() * error_p - Kv.asDiagonal() * odom.v;

    // const double yaw_err = wrapPi(des.yaw - yaw_odom);
    if(state == 4)
    {
      u.yaw_rate = des.yaw_rate;
    }
    else
    {
      // u.yaw_rate = Kyaw * yaw_err;
      u.yaw_rate = Kyaw * (des.yaw - yaw_odom);
    }

    //body系
    debug_msg_.des_v_x = u.velocity(0);
    debug_msg_.des_v_y = u.velocity(1);
    debug_msg_.des_v_z = u.velocity(2);

    debug_msg_.des_a_x = des.p(0);
    debug_msg_.des_a_y = des.p(1);
    debug_msg_.des_a_z = des.p(2);

    return debug_msg_;
}





// quadrotor_msgs::Px4ctrlDebug 
// LinearControl::velocityControl(const Desired_State_t &des,
//     const Odom_Data_t &odom,
//     const Imu_Data_t &imu, 
//     Controller_Output_t &u, const uint8_t state)
// {
//     Eigen::Vector3d Kp,Kv,Kff;
//     double Kyaw = 0.0;
//     switch (state) {
//       case 1: Kp.setZero(); Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;                  // READY_TO_FLY
//       case 2: Kp = {0.3,0.3,0.6}; Kv = {0.05,0.05,0.1}; Kff.setZero(); Kyaw = 1.0; break;                  // AUTO_HOVER
//       case 3: Kp = {0.5,0.5,0.8}; Kv = {0.1,0.1,0.2}; Kff.setOnes(); Kyaw = 1.0; break;     // CMD_CTRL
//       case 4: Kp.setZero(); Kv.setZero(); Kff.setOnes(); Kyaw = 0.0; break;                  // RC_CTRL（下游给 body）
//       case 5: Kp = {0.2,0.2,0.8}; Kv = {0,0,0.3}; Kff = {0,0, 1}; Kyaw = 1.0; break;                 // AUTO_TAKEOFF
//       case 6: Kp = {0.2,0.2,0.0}; Kv = {0,0,0.0}; Kff = {0,0,-1}; Kyaw = 1.0; break;                 // AUTO_LAND
//       default: Kp.setZero(); Kv.setZero(); Kff.setZero(); Kyaw = 0.0; break;
//     }

//     // ---------- 1) 世界->机体 3D 旋转 ----------
//     // R_wb: world->body，Eigen 的四元数 toRotationMatrix() 返回的是 R_bw? -> 实际返回的是 R (把向量左乘得 v' = R*v)
//     // 我们要 world->body: R_bw = R_wb^T；如果 odom.q 是 body 在 world 的姿态（常见），那么：
//     const Eigen::Matrix3d R_wb = odom.q.toRotationMatrix(); // 把 body 向 world 的旋转
//     const Eigen::Matrix3d R_bw = R_wb.transpose();          // world -> body

//     Eigen::Vector3d feedforward;//这里需要把ego传过来的速度转为body系，因为发给px4的是body的速度
//     if(state == 4)
//     {
//       feedforward = des.v ;
//     }
//     else
//     {
//       feedforward = R_bw * des.v;
//     }
//     Eigen::Vector3d e_p_b = R_bw * (des.p - odom.p);

//     // 如果你的里程计速度是世界系，请解除注释：
//     // const Eigen::Vector3d odom_v_b = R_bw * odom.v;
//     // 否则（MAVROS 常见：/mavros/local_position/odom 的 twist.twist.linear 是 body），直接用：
//     const Eigen::Vector3d& odom_v_b = odom.v;

//     // ---------- 2) 速度控制律（机体系） ----------
//     u.velocity = Kff.asDiagonal() * feedforward
//                + Kp.asDiagonal()  * e_p_b
//                - Kv.asDiagonal()  * odom_v_b;

//     // ---------- 3) yaw 控制（误差包络 + 率前馈） ----------
//     const double yaw_now = get_yaw_from_quaternion(odom.q);

//     const double yaw_err = wrapPi(des.yaw - yaw_now);

//     if (state == 4) {
//       u.yaw_rate = des.yaw_rate;                 // RC 模式：纯前馈（按需改）
//     } else {
//       u.yaw_rate = Kyaw * yaw_err; // 其它模式：前馈 + P
//     }


//     debug_msg_.des_v_x = u.velocity(0);
//     debug_msg_.des_v_y = u.velocity(1);
//     debug_msg_.des_v_z = u.velocity(2);

//     debug_msg_.des_a_x = des.p(0);
//     debug_msg_.des_a_y = des.p(1);
//     debug_msg_.des_a_z = des.p(2);

//     return debug_msg_;
// }




