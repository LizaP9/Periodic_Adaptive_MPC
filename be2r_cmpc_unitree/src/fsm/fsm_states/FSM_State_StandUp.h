#pragma once

#include "FSM_State.h"
#include "cppTypes.h"

/**
 *
 */
template<typename T>
class FSM_State_StandUp : public FSM_State<T>
{
public:
  FSM_State_StandUp(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void run();

  void standUpImpedance();
  void standUpJointPD();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

  TransitionData<T> testTransition();

private:
  // Keep track of the control iterations
  int iter = 0;
  std::vector<Vec3<T>> _ini_foot_pos = {};
  std::vector<Vec3<T>> _init_joint_q = {};
  std::vector<Vec3<T>> _stand_joint_q = {};
  float progress = 0;

  Mat3<T> Kp_joint = {};
  Mat3<T> Kd_joint = {};

  Mat3<T> Kp_cartesian = {};
  Mat3<T> Kd_cartesian = {};
};
