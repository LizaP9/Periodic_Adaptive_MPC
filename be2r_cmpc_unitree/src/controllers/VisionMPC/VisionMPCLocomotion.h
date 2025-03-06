#pragma once

#include <cstdio>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <string.h>

#include "ControlFSMData.h"
#include "FootSwingTrajectory.h"
#include "GraphSearch.h"
#include "SparseCMPC/SparseCMPC.h"
#include "Utilities/SpiralIterator.hpp"
#include "Utilities/Timer.h"
#include "Utilities/Utilities_print.h"
#include "controllers/CMPC/Gait_contact.h"
#include "convexMPC/Gait.h"
#include "convexMPC_interface.h"
#include "cppTypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "visualization_msgs/Marker.h"

#define MAX_STEP_HEIGHT 0.18

using Eigen::Array4f;
using Eigen::Array4i;

class VisionMPCLocomotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisionMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data);
  void initialize();

  void run(const Vec3<float>& vel_cmd,
           const grid_map::GridMap& height_map,
           const grid_map::GridMap& height_map_raw,
           const grid_map::GridMap& map_plane);
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
  void _setupCommand(ControlFSMData<float>& data);
  float _updateTrajHeight(size_t foot);
  void _updateFoothold(Vec3<float>& pf,
                       const Vec3<float>& body_pos,
                       const grid_map::GridMap& height_map,
                       const grid_map::GridMap& height_map_raw,
                       const grid_map::GridMap& map_plane,
                       int leg);

  void _idxMapChecking(Vec3<float>& Pf,
                       int x_idx,
                       int y_idx,
                       int& x_idx_selected,
                       int& y_idx_selected,
                       const grid_map::GridMap& height_map,
                       const grid_map::GridMap& height_map_raw,
                       int leg);

  void _locHeightClearance(const grid_map::GridMap& map, std::string layer, grid_map::Index center, double rad, double threshold);

  ControlFSMData<float>* _data;

  float _yaw_turn_rate;
  float _yaw_des = 0;

  float _roll_des;
  float _pitch_des;

  float _x_vel_des = 0.;
  float _y_vel_des = 0.;

  // High speed running
  float _body_height = 0.29;

  float _body_height_running = 0.29;
  float _body_height_jumping = 0.36;

  double _floor_plane_height;

  void recompute_timing(int iterations_per_mpc);
  void updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode);
  void solveDenseMPC(int* mpcTable, ControlFSMData<float>& data);
  void solveSparseMPC(int* mpcTable, ControlFSMData<float>& data);
  void initSparseMPC();
  int _iterationsBetweenMPC;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* _dyn_params = nullptr;
  int _gait_period;
  int horizonLength;
  int default_iterations_between_mpc;
  float dt;
  float dtMPC;
  int _iterationCounter = 0;
  Vec3<float> f_ff[4];
  Vec4<float> swingTimes;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  OffsetDurationGait trotting, standing, walking, two_leg_balance;
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
  float trajAll[12 * 36];
  ros::Publisher _pub_des_traj[4];
  ros::NodeHandle _nh;
  visualization_msgs::Marker marker[4];
  ros::Publisher _vis_pub[4];
  ros::Timer _timer;

  vectorAligned<Vec12<double>> _sparseTrajectory;

  SparseCMPC _sparseCMPC;
};

Eigen::Array2i checkBoundaries(const grid_map::GridMap& map, int col, int row);