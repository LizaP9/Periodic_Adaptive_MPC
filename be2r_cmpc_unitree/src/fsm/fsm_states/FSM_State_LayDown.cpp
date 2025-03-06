/*============================= Lay Down ==============================*/
/**
 * Transitionary state that is called for the robot to lay down
 */

#include "FSM_State_LayDown.h"

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_LayDown<T>::FSM_State_LayDown(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::LAYDOWN, "LAY_DOWN")
  , _ini_foot_pos(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  // f = boost::bind(&callbackROSros, _1, _2);
  // server.setCallback(f);

  // ROS_INFO("START SERVER");
}

template<typename T>
void FSM_State_LayDown<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;

  for (size_t leg(0); leg < 4; ++leg)
  {
    _ini_foot_pos[leg] = this->_data->legController->datas[leg].p;
  }
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_LayDown<T>::run()
{
  T progress = 0.5 * iter * this->_data->staticParams->controller_dt;

  if (progress > 1.)
  {
    progress = 1.;
    this->_data->legController->setEnabled(false);
  }

  auto& seResult = this->_data->stateEstimator->getResult();
  float mass = 8;
  Vec3<float> leg_force;
  leg_force << 0, 0, 0;
  float force = -mass * 9.81 / 4;
  leg_force = seResult.rBody * Vec3<float>(0, 0, force);

  // for real
  float p = 1200;
  float d = 15;

  // for sim
  // float p = 800;
  // float d = 15;

  for (int i = 0; i < 4; i++)
  {
    this->_data->legController->commands[i].kpCartesian = Vec3<T>(p, p, p).asDiagonal();
    this->_data->legController->commands[i].kdCartesian = Vec3<T>(d, d, d).asDiagonal();

    this->_data->legController->commands[i].pDes = _ini_foot_pos[i];
    this->_data->legController->commands[i].pDes[2] =
      progress * (-0.07) + (1. - progress) * _ini_foot_pos[i][2];

    this->_data->legController->commands[i].forceFeedForward = leg_force;
  }

  // cout << "z ini: " << _ini_foot_pos[0][2] << " z des: " <<
  // this->_data->legController->commands[0].pDes[2] << endl;
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_LayDown<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State)
  {
    case K_LAY_DOWN:
      break;

    case K_STAND_UP:
      // Requested switch to Stand Up
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    case K_PASSIVE: // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_LAY_DOWN << " to "
                << this->_data->userParameters->FSM_State << std::endl;
  }

  // Get the next state
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_LayDown<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
    case FSM_StateName::PASSIVE: // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    default:
      std::cout << "[CONTROL FSM] Something went wrong in transition" << std::endl;
  }

  // Return the transition data to the FSM
  return this->transitionData;
}

/**
 * Cleans up the state information on exiting the state.
 */
template<typename T>
void FSM_State_LayDown<T>::onExit()
{
  // Nothing to clean up when exiting
}

// template class FSM_State_LayDown<double>;
template class FSM_State_LayDown<float>;
