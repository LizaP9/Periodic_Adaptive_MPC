#pragma once

#include "FloatingBaseModel.h"
#include "Gait_contact.h"
#include "cppTypes.h"
#include <ControlFSMData.h>
#include <FootSwingTrajectory.h>
#include <SparseCMPC/SparseCMPC.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cstdio>

using Eigen::Array4f;
using Eigen::Array4i;

template<typename T>
struct CMPC_result
{
  LegControllerCommand<T> commands[4];
  Vec4<T> contactPhase;
};

struct CMPC_jump
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

class CMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data);
  void initialize();

  void run(ControlFSMData<float>& data);
  void myNewVersion(ControlFSMData<float>& data);
  void myVersion(ControlFSMData<float>& data);
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

private:
  void _SetupCommand(ControlFSMData<float>& data);

  float _yaw_turn_rate;
  float _yaw_des = 0;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  float _body_height = 0.29;
  float _swing_trajectory_height = 0.09;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  FloatingBaseModel<float> _model;
  FBModelState<float> _state;
  DMat<float> _A;
  DMat<float> _Ainv;
  DVec<float> _grav;
  DVec<float> _coriolis;
  ControlFSMData<float>* _data;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  void solveSparseMPC(int* mpcTable, ControlFSMData<float>& data);
  void initSparseMPC();
  void _updateModel(const StateEstimate<float>& state_est, const LegControllerData<float>* leg_data);
  int iterationsBetweenMPC;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _parameters = nullptr;
  int _gait_period;
  int _gait_period_long;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGaitContact trotting, trot_contact, standing, walking, two_leg_balance, give_hand, trot_long;
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
  float pitch_cmd = 0;
  float x_comp_integral = 0;
  Vec3<float> pFoot[4];
  CMPC_result<float> result;
  float trajAll[12 * 36];
  ros::NodeHandle _nh;

  CMPC_jump jump_state;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;
};
