#pragma once

#include "CMPC/CMPC_Locomotion_cv.h"
#include "FSM_State.h"
#include <FootSwingTrajectory.h>
#include <Utilities/Timer.h>
#include <controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>

#define ITERATIONS_BETWEEN_MPC 13

/**
 *
 */
template<typename T>
class FSM_State_Testing_Cv : public FSM_State<T>
{
public:
  FSM_State_Testing_Cv(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  void test1();
  void test2(float h);
  void gravTest();
  void safeJointTest();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

  CMPCLocomotion_Cv* CMPC;
  WBC_Ctrl<T>* _wbc_ctrl;
  LocomotionCtrlData<T>* _wbc_data;
  FloatingBaseModel<T> _model;
  void LocomotionControlStep();

  bool locomotionSafe();
  std::string map_topic_raw;
  std::string map_topic_filter;
  std::string map_topic_plane;

private:
  ros::NodeHandle _nh;
  ros::Subscriber _map_sub;
  ros::Subscriber _map_raw_sub;
  ros::Subscriber _robot_pose_sub;
  ros::Subscriber _map_plane_sub;

  grid_map::GridMap _grid_map;
  grid_map::GridMap _grid_map_raw;
  grid_map::GridMap _grid_map_plane;

  // Keep track of the control iterations
  int iter = 0;
  std::vector<Vec3<T>> _ini_foot_pos;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  void _elevMapCallback(const grid_map_msgs::GridMapConstPtr& msg);
  void _elevMapRawCallback(const grid_map_msgs::GridMapConstPtr& msg);
  void _elevMapPlaneCallback(const grid_map_msgs::GridMapConstPtr& msg);

  bool firstSwing[4];
  Vec3<float> pFoot[4];
};
