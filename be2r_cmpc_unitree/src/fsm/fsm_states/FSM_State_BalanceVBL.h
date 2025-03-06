#pragma once

#include "BalanceController/BalanceController.hpp"
#include "BalanceController/BalanceControllerVBL.hpp"
#include "BalanceController/ReferenceGRF.hpp"
#include "FSM_State.h"
#include <Utilities/Timer.h>

/**
 *
 */
template <typename T>
class FSM_State_BalanceVBL : public FSM_State<T>
{
public:
  FSM_State_BalanceVBL(ControlFSMData<T>* _controlFSMData);

  // Behavior to be carried out when entering a state
  void onEnter();

  // Run the normal behavior for the state
  void runBalanceController();
  void runBalanceControllerVBL();
  void run();

  // Checks for any transition triggers
  FSM_StateName checkTransition();

  // Manages state specific transitions
  TransitionData<T> transition();

  // Behavior to be carried out when exiting a state
  void onExit();

private:
  // Keep track of the control iterations
  ControlFSMData<T>* _data;
  Mat34<T> footFeedForwardForces; // feedforward forces at the feet
  int iter = 0;
  BalanceController* balanceController;
  BalanceControllerVBL* balance_controller_vbl;
  ReferenceGRF* reference_grf;
};
