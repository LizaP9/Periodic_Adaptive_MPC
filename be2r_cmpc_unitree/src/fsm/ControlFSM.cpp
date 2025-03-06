/*============================ Control FSM ============================*/
/**
 * The Finite State Machine that manages the robot's controls. Handles
 * calls to the FSM State functions and manages transitions between all
 * of the states.
 */

#include "ControlFSM.h"

using namespace std;

void execBash(string msg)
{
  string str = "rosrun dynamic_reconfigure dynparam set /unitree_ctrl FSM_State " + msg;
  system(str.c_str());
}

/**
 * Constructor for the Control FSM. Passes in all of the necessary
 * data and stores it in a struct. Initializes the FSM with a starting
 * state and operating mode.
 *
 * @param quadruped the quadruped information
 * @param stateEstimator contains the estimated states
 * @param legController interface to the leg controllers
 * @param _gaitScheduler controls scheduled foot contact modes
 * @param controlParameters passes in the control parameters from the GUI
 */
template<typename T>
ControlFSM<T>::ControlFSM(Quadruped<T>* quadruped,
                          StateEstimatorContainer<T>* stateEstimator,
                          LegController<T>* legController,
                          GaitScheduler<T>* gaitScheduler,
                          GamepadCommand* gamepad_command,
                          StaticParams* staticParams,
                          be2r_cmpc_unitree::ros_dynamic_paramsConfig* userParameters,
                          Debug* debug)
{
  // Add the pointers to the ControlFSMData struct
  data.quadruped = quadruped;
  data.stateEstimator = stateEstimator;
  data.legController = legController;
  data.gaitScheduler = gaitScheduler;
  data.staticParams = staticParams;
  data.userParameters = userParameters;
  data.gamepad_command = gamepad_command;
  data.debug = debug;

  // Initialize and add all of the FSM States to the state list
  statesList.invalid = nullptr;
  statesList.passive = new FSM_State_Passive<T>(&data);
  statesList.standUp = new FSM_State_StandUp<T>(&data);
  statesList.locomotion = new FSM_State_Locomotion<T>(&data);
  statesList.balanceStand = new FSM_State_BalanceStand<T>(&data);
  statesList.laydown = new FSM_State_LayDown<T>(&data);
  statesList.vision = new FSM_State_Vision<T>(&data);
  statesList.testing = new FSM_State_Testing<T>(&data);
  statesList.recoveryStand = new FSM_State_RecoveryStand<T>(&data);
  statesList.backflip = new FSM_State_BackFlip<T>(&data);
  statesList.balance_vbl = new FSM_State_BalanceVBL<T>(&data);
  statesList.testingCV = new FSM_State_Testing_Cv<T>(&data);

  // statesList.jointPD = new FSM_State_JointPD<T>(&data);
  // statesList.impedanceControl = new FSM_State_ImpedanceControl<T>(&data);
  // statesList.frontJump = new FSM_State_FrontJump<T>(&data);

  safetyChecker = new SafetyChecker<T>(&data);

  // Initialize the FSM with the Passive FSM State
  initialize();
}

/**
 * Initialize the Control FSM with the default settings. SHould be set to
 * Passive state and Normal operation mode.
 */
template<typename T>
void ControlFSM<T>::initialize()
{
  // Initialize a new FSM State with the control data
  currentState = statesList.passive;

  // Enter the new current state cleanly
  currentState->onEnter();

  // Initialize to not be in transition
  nextState = currentState;

  // Initialize FSM mode to normal operation
  operatingMode = FSM_OperatingMode::NORMAL;
}

/**
 * Called each control loop iteration. Decides if the robot is safe to
 * run controls and checks the current state for any transitions. Runs
 * the regular state behavior if all is normal.
 */
template<typename T>
void ControlFSM<T>::runFSM()
{

  // Check the robot state for safe operation
  operatingMode = safetyPreCheck();

  // Run the robot control code if operating mode is not unsafe
  if (operatingMode != FSM_OperatingMode::ESTOP)
  {
    if (operatingMode == FSM_OperatingMode::EDAMP)
    {
      // we do this only once, because we dont change operation mode from estop/edamp to normal
      static bool flag = true;
      static unsigned long iter_start = 0;
      static const unsigned long iter_duration = 1000;

      if (flag)
      {
        flag = false;
        iter_start = iter;
        currentState = statesList.passive;
        currentState->onEnter();
        nextStateName = currentState->stateName;
      }

      // ROS_INFO_ONCE("start");

      if (iter > (iter_start + iter_duration))
      {
        operatingMode = FSM_OperatingMode::ESTOP;
        // ROS_INFO("STOP!");
      }

      data.legController->edampCommand(3.0);
    }

    // Run normal controls if no transition is detected
    if (operatingMode == FSM_OperatingMode::NORMAL)
    {
      if (data.gamepad_command->down && (FSM_StateName::PASSIVE != currentState->stateName))
      {
        data.userParameters->FSM_State = 0;
        t1 = new std::thread(execBash, "0");
        ROS_WARN("PASSIVE");
      }

      if (data.gamepad_command->up && (FSM_StateName::STAND_UP != currentState->stateName))
      {
        data.userParameters->FSM_State = 1;

        t1 = new std::thread(execBash, "1");
        ROS_WARN("STAND UP");
      }

      if (data.gamepad_command->left && (FSM_StateName::TESTING != currentState->stateName))
      {
        data.userParameters->FSM_State = 12;

        t1 = new std::thread(execBash, "12");
        ROS_WARN("TESTING");
      }

      if (data.gamepad_command->right && (FSM_StateName::BALANCE_STAND != currentState->stateName))
      {
        data.userParameters->FSM_State = 3;

        t1 = new std::thread(execBash, "3");
        ROS_WARN("BALANCE STAND");
      }

      if (data.gamepad_command->chord_op_l1 && (FSM_StateName::TESTING_CV != currentState->stateName))
      {
        data.userParameters->FSM_State = 14;

        t1 = new std::thread(execBash, "14");
        ROS_WARN("TRANSITION TO TESTING CV");
      }

      // Check the current state for any transition
      nextStateName = currentState->checkTransition();

      // Detect a commanded transition
      if (nextStateName != currentState->stateName)
      {
        // Set the FSM operating mode to transitioning
        operatingMode = FSM_OperatingMode::TRANSITIONING;

        // Get the next FSM State by name
        nextState = getNextState(nextStateName);

        // Print transition initialized info
        printInfo(1);
      }
      else
      {
        // Run the iteration for the current state normally
        currentState->run();
      }
    }

    // Run the transition code while transition is occuring
    if (operatingMode == FSM_OperatingMode::TRANSITIONING)
    {
      transitionData = currentState->transition();

      // Check the robot state for safe operation
      safetyPostCheck();

      // Run the state transition
      if (transitionData.done)
      {
        // Exit the current state cleanly
        currentState->onExit();

        // Print finalizing transition info
        // printInfo(2);

        // Complete the transition
        currentState = nextState;

        // Enter the new current state cleanly
        currentState->onEnter();

        // Return the FSM to normal operation mode
        operatingMode = FSM_OperatingMode::NORMAL;
      }
    }
    else
    {
      // Check the robot state for safe operation
      safetyPostCheck();
    }
  }
  else
  { // if ESTOP
    currentState = statesList.passive;
    currentState->onEnter();
    nextStateName = currentState->stateName;
    data.legController->zeroCommand();
  }

  // Print the current state of the FSM
  printInfo(0);

  // Increase the iteration counter
  iter++;
}

/**
 * Checks the robot state for safe operation conditions. If it is in
 * an unsafe state, it will not run the normal control code until it
 * is safe to operate again.
 *
 * @return the appropriate operating mode
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPreCheck()
{
  // Check for safe orientation if the current state requires it
  if (currentState->checkSafeOrientation && data.userParameters->FSM_State != K_RECOVERY_STAND)
  {
    if (!safetyChecker->checkSafeOrientation())
    {
      // operatingMode = FSM_OperatingMode::ESTOP;
      operatingMode = FSM_OperatingMode::EDAMP;
      ROS_ERROR_STREAM("Broken: Orientation Safety Check FAIL!");
    }
  }

  if (data.userParameters->joint_limits && currentState->checkJointLimits && data.userParameters->FSM_State != K_RECOVERY_STAND)
  {
    if (!safetyChecker->checkJointLimits())
    {
      // operatingMode = FSM_OperatingMode::ESTOP;
      operatingMode = FSM_OperatingMode::EDAMP;
      ROS_ERROR_STREAM("Broken: Joint limits check FAIL!");
    }
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Checks the robot state for safe operation commands after calculating the
 * control iteration. Prints out which command is unsafe. Each state has
 * the option to enable checks for commands that it cares about.
 *
 * Should this EDamp / EStop or just continue?
 * Should break each separate check into its own function for clarity
 *
 * @return the appropriate operating mode
 */
template<typename T>
FSM_OperatingMode ControlFSM<T>::safetyPostCheck()
{
  // Check for safe desired foot positions
  if (currentState->checkPDesFoot)
  {
    safetyChecker->checkPDesFoot();
  }

  // Check for safe desired feedforward forces
  if (currentState->checkForceFeedForward)
  {
    safetyChecker->checkForceFeedForward();
  }

  // Default is to return the current operating mode
  return operatingMode;
}

/**
 * Returns the approptiate next FSM State when commanded.
 *
 * @param  next commanded enumerated state name
 * @return next FSM state
 */
template<typename T>
FSM_State<T>* ControlFSM<T>::getNextState(FSM_StateName stateName)
{
  // Choose the correct FSM State by enumerated state name
  switch (stateName)
  {
    case FSM_StateName::INVALID:
      return statesList.invalid;

    case FSM_StateName::PASSIVE:
      return statesList.passive;

      // case FSM_StateName::JOINT_PD:
      //   return statesList.jointPD;

      // case FSM_StateName::IMPEDANCE_CONTROL:
      //   return statesList.impedanceControl;

    case FSM_StateName::STAND_UP:
      return statesList.standUp;

    case FSM_StateName::BALANCE_STAND:
      return statesList.balanceStand;

    case FSM_StateName::BALANCE_VBL:
      return statesList.balance_vbl;

    case FSM_StateName::LAYDOWN:
      return statesList.laydown;

    case FSM_StateName::TESTING:
      return statesList.testing;

    case FSM_StateName::LOCOMOTION:
      return statesList.locomotion;

    case FSM_StateName::RECOVERY_STAND:
      return statesList.recoveryStand;

    case FSM_StateName::VISION:
      return statesList.vision;

    case FSM_StateName::BACKFLIP:
      return statesList.backflip;

    case FSM_StateName::TESTING_CV:
      return statesList.testingCV;

      // case FSM_StateName::FRONTJUMP:
      //   return statesList.frontJump;

    default:
      return statesList.invalid;
  }
}

/**
 * Prints Control FSM info at regular intervals and on important events
 * such as transition initializations and finalizations. Separate function
 * to not clutter the actual code.
 *
 * @param printing mode option for regular or an event
 */
template<typename T>
void ControlFSM<T>::printInfo(int opt)
{
  switch (opt)
  {
    case 0: // Normal printing case at regular intervals
      // Increment printing iteration
      printIter++;

      // Print at commanded frequency
      if (printIter == printNum)
      {
        std::cout << "[CONTROL FSM] Printing FSM Info...\n";
        std::cout << "---------------------------------------------------------\n";
        std::cout << "Iteration: " << iter << "\n";
        if (operatingMode == FSM_OperatingMode::NORMAL)
        {
          std::cout << "Operating Mode: NORMAL in " << currentState->stateString << "\n";
        }
        else if (operatingMode == FSM_OperatingMode::TRANSITIONING)
        {
          std::cout << "Operating Mode: TRANSITIONING from " << currentState->stateString << " to " << nextState->stateString
                    << "\n";
        }
        else if (operatingMode == FSM_OperatingMode::ESTOP)
        {
          std::cout << "Operating Mode: ESTOP\n";
        }
        std::cout << "Gait Type: " << data.gaitScheduler->gaitData.gaitName << "\n";
        std::cout << std::endl;

        // Reset iteration counter
        printIter = 0;
      }

      // Print robot info about the robot's status
      // data._gaitScheduler->printGaitInfo();

      break;

    case 1: // Initializing FSM State transition
      std::cout << "[CONTROL FSM] Transition initialized from " << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;

    case 2: // Finalizing FSM State transition
      std::cout << "[CONTROL FSM] Transition finalizing from " << currentState->stateString << " to " << nextState->stateString
                << "\n"
                << std::endl;

      break;
  }
}

// template class ControlFSM<double>; This should be fixed... need to make
// RobotRunner a template
template class ControlFSM<float>;
