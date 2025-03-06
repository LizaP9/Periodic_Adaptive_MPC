#ifndef FSM_STATE_TESTING_H
#define FSM_STATE_TESTING_H

#include "CMPC/CMPC_Locomotion.h"
#include "FSM_State.h"
#include <FootSwingTrajectory.h>
#include <Utilities/Timer.h>
#include <controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

#define ITERATIONS_BETWEEN_MPC 13

/**
 *
 */
template<typename T>
class FSM_State_Testing : public FSM_State<T>
{
public:
  FSM_State_Testing(ControlFSMData<T>* _controlFSMData);

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

  CMPCLocomotion* CMPC;
  WBC_Ctrl<T>* _wbc_ctrl;
  LocomotionCtrlData<T>* _wbc_data;
  FloatingBaseModel<T> _model;
  void LocomotionControlStep();

  bool locomotionSafe();

private:
  // Keep track of the control iterations
  int iter = 0;
  std::vector<Vec3<T>> _ini_foot_pos;
  DVec<T> _ini_jpos;
  FootSwingTrajectory<float> footSwingTrajectories[4];
  bool firstSwing[4];
  Vec3<float> pFoot[4];
};

#endif // FSM_STATE_TESTING_H
