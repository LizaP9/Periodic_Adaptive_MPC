#ifndef CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
#define CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H

#include "Gait.h"
#include "cppTypes.h"
#include <ControlFSMData.h>
#include <FootSwingTrajectory.h>
#include <SparseCMPC/SparseCMPC.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
//#include <ros/ros.h>

#include <cstdio>
#include <unitree_legged_msgs/LogData.h>

using Eigen::Array4f;
using Eigen::Array4i;

template<typename T>
struct CMPC_Result
{
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_Jump
{
  static constexpr int START_SEG = 6;
  static constexpr int END_SEG = 0;
  static constexpr int END_COUNT = 2;
  bool jump_pending = false;
  bool jump_in_progress = false;
  bool pressed = false;
  int seen_end_count = 0;
  int last_seg_seen = 0;
  int jump_wait_counter = 0;

  void debug(int seg)
  {
    (void)seg;
    // printf("[%d] pending %d running %d\n", seg, jump_pending, jump_in_progress);
  }

  void trigger_pressed(int seg, bool trigger)
  {
    (void)seg;
    if (!pressed && trigger)
    {
      if (!jump_pending && !jump_in_progress)
      {
        jump_pending = true;
        // printf("jump pending @ %d\n", seg);
      }
    }
    pressed = trigger;
  }

  bool should_jump(int seg)
  {
    debug(seg);

    if (jump_pending && seg == START_SEG)
    {
      jump_pending = false;
      jump_in_progress = true;
      // printf("jump begin @ %d\n", seg);
      seen_end_count = 0;
      last_seg_seen = seg;
      return true;
    }

    if (jump_in_progress)
    {
      if (seg == END_SEG && seg != last_seg_seen)
      {
        seen_end_count++;
        if (seen_end_count == END_COUNT)
        {
          seen_end_count = 0;
          jump_in_progress = false;
          // printf("jump end @ %d\n", seg);
          last_seg_seen = seg;
          return false;
        }
      }
      last_seg_seen = seg;
      return true;
    }

    last_seg_seen = seg;
    return false;
  }
};

class ConvexMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data);
  void initialize();

  template<typename T>
  void run(ControlFSMData<T>& data);
  bool currently_jumping = false;

  Vec3<float> pBody_des;
  Vec3<float> vBody_des;
  Vec3<float> aBody_des;

  Vec3<float> pBody_RPY_des;
  Vec3<float> vBody_Ori_des;

  Vec3<float> pFoot_des[4];
  Vec3<float> vFoot_des[4];
  Vec3<float> aFoot_des[4];

  Vec3<float> Fr_des[4];

  Vec4<float> contact_state;

  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg); // Добавляем callback
  void logDataCallback(const unitree_legged_msgs::LogData::ConstPtr& msg);


private:
  void _SetupCommand(ControlFSMData<float>& data);
  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  void solveSparseMPC(int* mpcTable, ControlFSMData<float>& data);
  void initSparseMPC();

  ControlFSMData<float>* _fsm_data;

  float _yaw_turn_rate;
  float _yaw_des;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  // float _body_height = 0.34;
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  int _iterationsBetweenMPC;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _dyn_params = nullptr;
  int _gait_period;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, bounding, pronking, jumping, galloping, standing, trotRunning, walking, walking2, pacing;
  MixedFrequncyGait random, random2;
  Mat3<float> Kp, Kd, Kp_stance, Kd_stance;
  bool firstRun = true;
  bool firstSwing[4];
  float swingTimeRemaining[4];
  float stand_traj[6];
  int current_gait;
  int gaitNumber;

  Vec3<float> world_position_desired;
  Vec3<float> rpy_int;
  Vec3<float> rpy_comp;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CMPC_Result<float> result;
  float trajAll[12 * 36];
  ros::NodeHandle _nh;

  CMPC_Jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;

  float pitch_cmd = 0.;

  ros::Subscriber ground_truth_sub;
  ros::Publisher log_data_pub;
  void publishLogData();

  Vec3<float> ground_truth_position;
  Vec3<float> ground_truth_velocity;
  Vec3<float> ground_truth_velocity_ang;
  Vec3<float> ground_truth_rpy;
  Vec4<float> ground_truth_quaternion;

  struct MPCLogData {
    ros::Time timestamp;
    Vec3<float> position;
    Vec3<float> rpy;
    Vec3<float> velocity;
    Vec3<float> angular_velocity;
    Vec3<float> forces[4];  // Силы реакции опоры для 4 ног
    float x_drag;
    float r_feet[12];
    float roll;
    float pitch;
    float yaw;
  };

  Eigen::Matrix3f R_x, R_y, R_z, R;
  
  MPCLogData last_mpc_data;

  ros::Subscriber log_data_sub;
  unitree_legged_msgs::LogData last_log_data_received_;
  bool received_log_data_ = false;


};

#endif // CHEETAH_SOFTWARE_CONVEXMPCLOCOMOTION_H
