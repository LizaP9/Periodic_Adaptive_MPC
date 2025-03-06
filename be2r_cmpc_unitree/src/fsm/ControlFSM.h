#ifndef CONTROLFSM_H
#define CONTROLFSM_H

#include <iostream>
#include <thread>

// Contains all of the control related data
#include "ControlFSMData.h"

// Checks the robot state and commands for safety
#include "SafetyChecker.h"

// FSM States
#include "FSM_State.h"
#include "FSM_State_BackFlip.h"
#include "FSM_State_BalanceStand.h"
#include "FSM_State_BalanceVBL.h"
#include "FSM_State_LayDown.h"
#include "FSM_State_Locomotion.h"
#include "FSM_State_Passive.h"
#include "FSM_State_RecoveryStand.h"
#include "FSM_State_StandUp.h"
#include "FSM_State_Testing.h"
#include "FSM_State_Testing_cv.h"
#include "FSM_State_Vision.h"

// #include "FSM_State_FrontJump.h"
// #include "FSM_State_ImpedanceControl.h"
// #include "FSM_State_JointPD.h"

/**
 * Enumerate all of the operating modes
 */
enum class FSM_OperatingMode
{
  NORMAL,
  TRANSITIONING,
  ESTOP,
  EDAMP
};

/**
 *
 */
template<typename T>
struct FSM_StatesList
{
  FSM_State<T>* invalid;
  FSM_State_Passive<T>* passive;
  FSM_State_StandUp<T>* standUp;
  FSM_State_Locomotion<T>* locomotion;
  FSM_State_BalanceStand<T>* balanceStand;
  FSM_State_LayDown<T>* laydown;
  FSM_State_Vision<T>* vision;
  FSM_State_Testing<T>* testing;
  FSM_State_RecoveryStand<T>* recoveryStand;
  FSM_State_BackFlip<T>* backflip;
  FSM_State_BalanceVBL<T>* balance_vbl;
  FSM_State_Testing_Cv<T>* testingCV;

  // FSM_State_JointPD<T>* jointPD;
  // FSM_State_ImpedanceControl<T>* impedanceControl;
  // FSM_State_FrontJump<T>* frontJump;
};

/**
 *
 */
template<typename T>
struct FSM_ControllerList
{
};

/**
 * Control FSM handles the FSM states from a higher level
 */
template<typename T>
class ControlFSM
{
public:
  ControlFSM(Quadruped<T>* quadruped,
             StateEstimatorContainer<T>* stateEstimator,
             LegController<T>* legController,
             GaitScheduler<T>* gaitScheduler,
             GamepadCommand* gamepad_command,
             StaticParams* controlParameters,
             be2r_cmpc_unitree::ros_dynamic_paramsConfig* userParameters,
             Debug* debug);

  // Initializes the Control FSM instance
  void initialize();

  // Runs the FSM logic and handles the state transitions and normal runs
  void runFSM();

  // This will be removed and put into the SafetyChecker class
  FSM_OperatingMode safetyPreCheck();

  //
  FSM_OperatingMode safetyPostCheck();

  // Gets the next FSM_State from the list of created states when requested
  FSM_State<T>* getNextState(FSM_StateName stateName);

  // Prints the current FSM status
  void printInfo(int opt);

  // Contains all of the control related data
  ControlFSMData<T> data;

  // FSM state information
  FSM_StatesList<T> statesList; // holds all of the FSM States
  FSM_State<T>* currentState;   // current FSM state
  FSM_State<T>* nextState;      // next FSM state
  FSM_StateName nextStateName;  // next FSM state name

  // Checks all of the inputs and commands for safety
  SafetyChecker<T>* safetyChecker;

  TransitionData<T> transitionData;

private:
  // Operating mode of the FSM
  FSM_OperatingMode operatingMode;

  // Choose how often to print info, every N iterations
  int printNum = 10000; // N*(0.001s) in simulation time

  std::thread* t1 = nullptr;

  // Track the number of iterations since last info print
  int printIter = 0; // make larger than printNum to not print

  int iter = 0;
};

#endif // CONTROLFSM_H
