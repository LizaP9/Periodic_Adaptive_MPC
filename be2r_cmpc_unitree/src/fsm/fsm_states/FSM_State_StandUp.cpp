/*============================= Stand Up ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_StandUp.h"
#include "cppTypes.h"
#include "ros_read_param.h"
#include <iostream>
#include <iterator>
#include <ostream>

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_StandUp<T>::FSM_State_StandUp(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::STAND_UP, "STAND_UP"),
    _ini_foot_pos(4),
    _init_joint_q(4),
    _stand_joint_q(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
  this->checkJointLimits = true;

  readRosParam("standup/Kp_joint0", Kp_joint(0, 0));
  readRosParam("standup/Kp_joint1", Kp_joint(1, 1));
  readRosParam("standup/Kp_joint2", Kp_joint(2, 2));

  readRosParam("standup/Kd_joint0", Kd_joint(0, 0));
  readRosParam("standup/Kd_joint1", Kd_joint(1, 1));
  readRosParam("standup/Kd_joint2", Kd_joint(2, 2));

  readRosParam("standup/Kp_cartesian_x", Kp_cartesian(0, 0));
  readRosParam("standup/Kp_cartesian_y", Kp_cartesian(1, 1));
  readRosParam("standup/Kp_cartesian_z", Kp_cartesian(2, 2));

  readRosParam("standup/Kd_cartesian_x", Kd_cartesian(0, 0));
  readRosParam("standup/Kd_cartesian_y", Kd_cartesian(1, 1));
  readRosParam("standup/Kd_cartesian_z", Kd_cartesian(2, 2));
}

template<typename T>
void FSM_State_StandUp<T>::onEnter()
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
    _init_joint_q[leg] = this->_data->legController->datas[leg].q;
    _stand_joint_q[leg](0) = 0.0;
    _stand_joint_q[leg](1) = -1.05;
    _stand_joint_q[leg](2) = 2.1;
  }

  progress = 0;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_StandUp<T>::run()
{
  standUpImpedance();
  // standUpJointPD();
}

template<typename T>
void FSM_State_StandUp<T>::standUpImpedance()
{
  T hMax = 0.33;
  progress = 0.5 * iter * this->_data->staticParams->controller_dt;

  if (progress > 1.)
  {
    progress = 1.;
  }

  auto& seResult = this->_data->stateEstimator->getResult();
  float mass = this->_data->quadruped->_bodyMass;
  Vec3<float> leg_force;
  leg_force << 0, 0, 0;
  float force = -mass * 9.81 / 4;
  leg_force = seResult.rBody * Vec3<float>(0, 0, force);

  for (int i = 0; i < 4; i++)
  {
    // for real with gravity compensation
    this->_data->legController->commands[i].kpCartesian = Kp_cartesian;
    this->_data->legController->commands[i].kdCartesian = Kd_cartesian;

    this->_data->legController->commands[i].pDes = _ini_foot_pos[i];
    this->_data->legController->commands[i].pDes[2] = progress * (-hMax) + (1. - progress) * _ini_foot_pos[i][2];

    this->_data->debug->last_p_local_stance[i] =
      ros::toMsg(this->_data->legController->datas[i].p + this->_data->quadruped->getHipLocation(i));

    this->_data->legController->commands[i].forceFeedForward = leg_force;
  }
}

template<typename T>
void FSM_State_StandUp<T>::standUpJointPD()
{
  T hMax = 0.33;
  T progress = 0.5 * iter * this->_data->staticParams->controller_dt;

  if (progress > 1.)
  {
    progress = 1.;
  }

  auto& seResult = this->_data->stateEstimator->getResult();
  float mass = this->_data->quadruped->_bodyMass;
  Vec3<float> leg_force;
  leg_force << 0, 0, 0;
  float force = -mass * 9.81 / 4;
  leg_force = seResult.rBody * Vec3<float>(0, 0, force);

  Vec3<float> pDes[4];
  Vec3<float> q_des[4];

  for (int i = 0; i < 4; i++)
  {
    pDes[i] = _ini_foot_pos[i];
    pDes[i][2] = progress * (-hMax) + (1. - progress) * _ini_foot_pos[i][2];

    q_des[i] = this->findAngles(i, pDes[i]);
    q_des[i](1) *= -1;
    q_des[i](2) *= -1;

    // for real with gravity compensation
    this->_data->legController->commands[i].kpJoint = Kp_joint;
    this->_data->legController->commands[i].kdJoint = Kd_joint;

    // this->_data->legController->commands[i].qDes = _stand_joint_q[i] * progress + (1.0 - progress) * _init_joint_q[i];
    this->_data->legController->commands[i].qDes = q_des[i];
    this->_data->legController->commands[i].qdDes = Vec3<float>::Zero();

    this->_data->debug->last_p_local_stance[i] =
      ros::toMsg(this->_data->legController->datas[i].p + this->_data->quadruped->getHipLocation(i));

    this->_data->legController->commands[i].forceFeedForward = leg_force;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_StandUp<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State)
  {
    case K_STAND_UP:
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_BALANCE_VBL:
      this->nextStateName = FSM_StateName::BALANCE_VBL;
      break;

    case K_LOCOMOTION:
      this->nextStateName = FSM_StateName::LOCOMOTION;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;

    case K_PASSIVE: // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_LAY_DOWN:
      this->nextStateName = FSM_StateName::LAYDOWN;
      break;

    case K_TESTING:
      this->nextStateName = FSM_StateName::TESTING;
      break;

    case K_TESTING_CV:
      this->nextStateName = FSM_StateName::TESTING_CV;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_STAND_UP << " to "
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
TransitionData<T> FSM_State_StandUp<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
    case FSM_StateName::PASSIVE: // normal
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_VBL:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LOCOMOTION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LAYDOWN:
      this->transitionData.done = true;
      break;

    case FSM_StateName::TESTING:
      this->transitionData.done = true;
      break;

    case FSM_StateName::TESTING_CV:
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
void FSM_State_StandUp<T>::onExit()
{
  // Nothing to clean up when exiting
}

// template class FSM_State_StandUp<double>;
template class FSM_State_StandUp<float>;
