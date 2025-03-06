/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_BalanceStand.h"
#include <controllers/WBC_Ctrl/LocomotionCtrl/LocomotionCtrl.hpp>
#include <iostream>

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_BalanceStand<T>::FSM_State_BalanceStand(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND, "BALANCE_STAND")
{
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();

  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();

  _wbc_ctrl->setFloatingBaseWeight(1000.);
}

template<typename T>
void FSM_State_BalanceStand<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Always set the gait to be standing in this state
  this->_data->gaitScheduler->gaitData._nextGait = GaitType::STAND;

  _ini_body_pos = (this->_data->stateEstimator->getResult()).position;

  for (uint8_t i = 0; i < 4; i++)
  {
    _ini_foot_pos[i] = this->_data->legController->datas[i].p;
  }

  // cout << _ini_body_pos << endl;

  if (_ini_body_pos[2] < 0.2)
  {
    _ini_body_pos[2] = 0.2;
  }

  last_height_command = _ini_body_pos[2];

  _ini_body_ori_rpy = (this->_data->stateEstimator->getResult()).rpy;
  _body_weight = this->_data->quadruped->_bodyMass * 9.81;

  flag = false;
}

void execBashBalance(string msg)
{
  string str = "rosrun dynamic_reconfigure dynparam set /unitree_ctrl test " + msg;
  system(str.c_str());
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_BalanceStand<T>::run()
{
  Vec4<T> contactState;
  contactState << 0.5, 0.5, 0.5, 0.5;
  this->_data->stateEstimator->setContactPhase(contactState);

  // circle
  if (this->_data->gamepad_command->circle)
  {
    this->_data->userParameters->test = 2;
    t1 = new std::thread(execBashBalance, "2");
    this->_data->gamepad_command->circle = false;
  }
  // wave
  else if (this->_data->gamepad_command->triangle)
  {
    this->_data->userParameters->test = 1;
    t1 = new std::thread(execBashBalance, "1");
    this->_data->gamepad_command->triangle = false;
  }
  // standart balance stand
  else if (this->_data->gamepad_command->cross)
  {
    this->_data->userParameters->test = 0;
    t1 = new std::thread(execBashBalance, "0");
    this->_data->gamepad_command->cross = false;
  }
  // give hand
  else if (this->_data->gamepad_command->rectangle)
  {
    this->_data->userParameters->test = 3;
    t1 = new std::thread(execBashBalance, "3");
    this->_data->gamepad_command->rectangle = false;
  }

  switch (this->_data->userParameters->test)
  {
    case 0:
      BalanceStandStepDefault();
      break;

    case 1:
      BalanceStandStepWave();
      break;

    case 2:
      BalanceStandStepCircle();
      break;

    case 3:
      BalanceStandGiveHand();
      break;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_BalanceStand<T>::checkTransition()
{
  // Get the next state
  _iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State)
  {
    case K_BALANCE_STAND:
      // Normal operation for state based transitions

      // Need a working state estimator for this
      /*if (velocity > v_max) {
        // Notify the State of the upcoming next state
        this->nextStateName = FSM_StateName::LOCOMOTION;

        // Transition instantaneously to locomotion state on request
        this->transitionDuration = 0.0;

        // Set the next gait in the scheduler to
        this->_data->_gaitScheduler->gaitData._nextGait = GaitType::TROT;

      }*/

      // TEST: in place to show automatic non user requested transitions
      /*if (_iter >= 5458) {
        this->nextStateName = FSM_StateName::LOCOMOTION;
        this->_data->userParameters->FSM_State = K_LOCOMOTION;
        this->transitionDuration = 0.0;
        this->_data->_gaitScheduler->gaitData._nextGait =
            GaitType::AMBLE;  // TROT; // Or get whatever is in
                              // main_control_settings
        _iter = 0;
      }*/
      break;

    case K_LOCOMOTION:
      // Requested change to balance stand
      this->nextStateName = FSM_StateName::LOCOMOTION;

      // Transition instantaneously to locomotion state on request
      this->transitionDuration = 0.0;

      // Set the next gait in the scheduler to
      this->_data->gaitScheduler->gaitData._nextGait = GaitType::TROT;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

      // case K_STAND_UP:
      //   this->nextStateName = FSM_StateName::STAND_UP;
      //   // Transition time is immediate
      //   this->transitionDuration = 0.0;
      //   break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_RECOVERY_STAND:
      this->nextStateName = FSM_StateName::RECOVERY_STAND;
      // Transition time is immediate
      this->transitionDuration = 0.0;
      break;

    case K_BACKFLIP:
      this->nextStateName = FSM_StateName::BACKFLIP;
      this->transitionDuration = 0.;
      break;

    case K_TESTING:
      this->nextStateName = FSM_StateName::TESTING;
      this->transitionDuration = 0.;
      break;

    case K_LAY_DOWN:
      this->nextStateName = FSM_StateName::LAYDOWN;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_BALANCE_STAND << " to "
                << this->_data->userParameters->FSM_State << std::endl;
  }

  // Return the next state name to the FSM
  return this->nextStateName;
}

/**
 * Handles the actual transition for the robot between states.
 * Returns true when the transition is completed.
 *
 * @return true if transition is complete
 */
template<typename T>
TransitionData<T> FSM_State_BalanceStand<T>::transition()
{
  // this->_data->legController->is_low_level = true;

  // Switch FSM control mode
  switch (this->nextStateName)
  {
    case FSM_StateName::LOCOMOTION:
      BalanceStandStepDefault();

      _iter++;
      if (_iter >= this->transitionDuration * 1000)
      {
        this->transitionData.done = true;
      }
      else
      {
        this->transitionData.done = false;
      }

      break;

    case FSM_StateName::PASSIVE:
      this->turnOffAllSafetyChecks();
      this->transitionData.done = true;
      break;

    case FSM_StateName::RECOVERY_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BACKFLIP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::TESTING:
      this->transitionData.done = true;
      break;

    case FSM_StateName::LAYDOWN:
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
void FSM_State_BalanceStand<T>::onExit()
{
  _iter = 0;
}

template<typename T>
T LinearInterpolation(T initPos, T targetPos, double rate)
{
  T p;
  rate = std::min(std::max(rate, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStepDefault()
{
  static size_t ticks = 0;
  ticks++;

  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy; // original
  _wbc_data->pBody_RPY_des[0] = 0;
  _wbc_data->pBody_RPY_des[1] = 0;

  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->legController->commands[leg].kdCartesian;
    geometry_msgs::Point point;
    point = ros::toMsg(this->_data->legController->datas[leg].p + this->_data->quadruped->getHipLocation(leg));
    this->_data->debug->last_p_local_stance[leg] = point;
  }

  // Orientation from joy
  _wbc_data->pBody_RPY_des[1] = 0.4 * this->_data->gamepad_command->right_stick_analog[1];
  _wbc_data->pBody_RPY_des[0] = 0.4 * this->_data->gamepad_command->right_stick_analog[0];
  _wbc_data->pBody_RPY_des[2] -= 0.4 * this->_data->gamepad_command->left_stick_analog[0];

  static size_t dance_cntr = 0;
  if (ticks >= 1000)
  {
    ticks = 0;
    dance_cntr++;
  }

  static double deg = M_PI / 180;
  _wbc_data->pBody_RPY_des[0] = 0.;

  // Height
  _wbc_data->pBody_des[2] += 0.08 * this->_data->gamepad_command->left_stick_analog[1];

  _wbc_data->vBody_Ori_des.setZero();

  for (size_t i(0); i < 4; ++i)
  {
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight / 4.;
    _wbc_data->contact_state[i] = true;
  }

  // if (this->_data->gamepad_command->trigger_pressed)
  // {
  //   _wbc_data->pBody_des[2] = 0.05;

  //   if (last_height_command - _wbc_data->pBody_des[2] > 0.001)
  //   {
  //     _wbc_data->pBody_des[2] = last_height_command - 0.001;
  //   }
  // }

  // last_height_command = _wbc_data->pBody_des[2];

  _wbc_ctrl->run(_wbc_data, *this->_data);

  this->_data->debug->body_info.pos_des = ros::toMsg(_wbc_data->pBody_des);
  this->_data->debug->body_info.euler_des.x = _wbc_data->pBody_RPY_des[0];
  this->_data->debug->body_info.euler_des.y = _wbc_data->pBody_RPY_des[1];
  this->_data->debug->body_info.euler_des.z = _wbc_data->pBody_RPY_des[2];
}

template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStepCircle()
{
  static size_t ticks = 0;
  ticks++;

  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy; // original
  _wbc_data->pBody_RPY_des[0] = 0;
  _wbc_data->pBody_RPY_des[1] = 0;

  _wbc_data->pBody_des[2] = 0.23 + std::cos((double(ticks) / 1000.) * 6.28) * 0.08;

  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->legController->commands[leg].kdCartesian;
    geometry_msgs::Point point;
    point = ros::toMsg(this->_data->legController->datas[leg].p + this->_data->quadruped->getHipLocation(leg));
    this->_data->debug->last_p_local_stance[leg] = point;
  }

  static size_t dance_cntr = 0;
  if (ticks >= 1000)
  {
    ticks = 0;
    dance_cntr++;
  }
  // std::cout << "ticks = " << ticks << std::endl;
  static double deg = M_PI / 180;
  _wbc_data->pBody_RPY_des[0] = 0.;
  _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1] + std::sin((double(ticks) / 1000.) * 6.28) * 20 * deg;
  _wbc_data->pBody_RPY_des[2] = _ini_body_ori_rpy[2] + std::cos((double(ticks) / 1000.) * 6.28) * 20 * deg;

  // Height
  _wbc_data->pBody_des[2] += 0.08 * this->_data->gamepad_command->left_stick_analog[1];

  _wbc_ctrl->run(_wbc_data, *this->_data);
}

template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandStepWave()
{
  static size_t ticks = 0;
  ticks++;

  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy; // original
  _wbc_data->pBody_RPY_des[0] = 0;
  _wbc_data->pBody_RPY_des[1] = 0;

  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->legController->commands[leg].kdCartesian;
    geometry_msgs::Point point;
    point = ros::toMsg(this->_data->legController->datas[leg].p + this->_data->quadruped->getHipLocation(leg));
    this->_data->debug->last_p_local_stance[leg] = point;
  }

  static size_t dance_cntr = 0;
  if (ticks >= 1000)
  {
    ticks = 0;
    dance_cntr++;
  }
  // std::cout << "ticks = " << ticks << std::endl;
  static double deg = M_PI / 180;
  _wbc_data->pBody_RPY_des[0] = 0.;

  _wbc_data->pBody_des[2] = 0.23 + std::cos((double(ticks) / 1000.) * 6.28) * 0.08;
  _wbc_data->pBody_RPY_des[1] = _ini_body_ori_rpy[1] + std::sin((double(ticks) / 1000.) * 6.28) * 20 * deg;

  _wbc_data->vBody_Ori_des.setZero();

  for (size_t i(0); i < 4; ++i)
  {
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight / 4.;
    _wbc_data->contact_state[i] = true;
  }

  // if (this->_data->gamepad_command->trigger_pressed)
  // {
  //   _wbc_data->pBody_des[2] = 0.05;

  //   if (last_height_command - _wbc_data->pBody_des[2] > 0.001)
  //   {
  //     _wbc_data->pBody_des[2] = last_height_command - 0.001;
  //   }
  // }

  // last_height_command = _wbc_data->pBody_des[2];

  _wbc_ctrl->run(_wbc_data, *this->_data);
}

template<typename T>
void FSM_State_BalanceStand<T>::BalanceStandGiveHand()
{
  Vec3<T> pDes_backup[4];
  Vec3<T> vDes_backup[4];
  Mat3<T> Kp_backup[4];
  Mat3<T> Kd_backup[4];

  auto& seResult = this->_data->stateEstimator->getResult();

  float rate = 0.5;

  _wbc_data->pBody_des = _ini_body_pos;
  _wbc_data->vBody_des.setZero();
  _wbc_data->aBody_des.setZero();

  _wbc_data->pBody_RPY_des = _ini_body_ori_rpy; //original
  _wbc_data->pBody_RPY_des[0] = 0;
  _wbc_data->pBody_RPY_des[1] = 0;

  for (int leg(0); leg < 4; ++leg)
  {
    pDes_backup[leg] = this->_data->legController->commands[leg].pDes;
    vDes_backup[leg] = this->_data->legController->commands[leg].vDes;
    Kp_backup[leg] = this->_data->legController->commands[leg].kpCartesian;
    Kd_backup[leg] = this->_data->legController->commands[leg].kdCartesian;
  }

  for (size_t i(0); i < 4; ++i)
  {
    // pFoot in global frame
    _wbc_data->pFoot_des[i].setZero();
    _wbc_data->vFoot_des[i].setZero();
    _wbc_data->aFoot_des[i].setZero();
    _wbc_data->Fr_des[i].setZero();
    _wbc_data->Fr_des[i][2] = _body_weight / 4.;
    _wbc_data->contact_state[i] = true;
  }

  Vec3<float> p_des(0, 0, 0);

  // p des local
  // p_des(0) = 0.15;
  // p_des(1) = -0.1;
  // p_des(2) = -0.132;

  // Vec3<float> p0_act = this->_data->legController->datas[0].p;

  // p_des = p0_act;
  // p_des(2) = -0.132;

  static unsigned long iter_start = 0;

  float progress = rate * _iter * this->_data->staticParams->controller_dt;

  if (progress > 1)
  {
    progress = 1;
  }

  // _wbc_data->pBody_des[0] += -0.06;
  // _wbc_data->pBody_des[1] += 0.06;
  // _wbc_data->pBody_des[2] = 0.22;

  Vec3<float> p_body_des(0, 0, 0);
  Vec3<float> delta_p_body(0, 0, 0);

  delta_p_body << -0.06, 0.03, 0;

  p_body_des[0] = _ini_body_pos[0] - 0.06;
  // p_body_des[1] = _ini_body_pos[1] + 0.06;
  p_body_des[1] = _ini_body_pos[1] + 0.03;

  p_body_des = seResult.rBody.transpose() * (delta_p_body) + _ini_body_pos;
  p_body_des[2] = 0.22;

  // _wbc_data->pBody_des[0] = _ini_body_pos[0] - 0.06;
  // _wbc_data->pBody_des[1] = _ini_body_pos[1] + 0.06;
  // _wbc_data->pBody_des[2] = 0.22;

  // _wbc_data->pBody_des = LinearInterpolation(_ini_body_pos, _wbc_data->pBody_des, progress);
  _wbc_data->pBody_des = LinearInterpolation(_ini_body_pos, p_body_des, progress);

  // _wbc_data->pBody_RPY_des[1] = -0.2;
  float pitch_des = _ini_body_ori_rpy[1] - 0.2;
  float pitch_start = _ini_body_ori_rpy[1];
  _wbc_data->pBody_RPY_des[1] = LinearInterpolation(pitch_start, pitch_des, progress);

  float epsilon = 0.005;
  // Vec3<float> p_err = _wbc_data->pBody_des - (this->_data->stateEstimator->getResult()).position;
  Vec3<float> p_err = p_body_des - (this->_data->stateEstimator->getResult()).position;

  if (abs(p_err(0) < epsilon) && abs(p_err(1) < epsilon) && (flag == false))
  {
    flag = true;
    iter_start = _iter;
    _ini_foot_pos[0] = this->_data->legController->datas[0].p;
    ROS_WARN("low error");
  }

  if (flag)
  {
    float progress_local = rate * (_iter - iter_start) * this->_data->staticParams->controller_dt;
    // cout << progress_local << endl;

    if (progress_local > 1)
    {
      progress_local = 1;
    }

    p_des = _ini_foot_pos[0];

    p_des(0) += 0.1;
    p_des(1) += 0.05;
    // p_des(0) = 0.1;
    // p_des(1) = 0.05;
    p_des(2) = -0.13;
    p_des = LinearInterpolation(_ini_foot_pos[0], p_des, progress_local);

    Vec3<float> p_w_des(0, 0, 0);
    p_w_des = seResult.position + seResult.rBody.transpose() * (this->_data->quadruped->getHipLocation(0) + p_des);

    for (size_t i(0); i < 4; ++i)
    {
      _wbc_data->Fr_des[i].setZero();
      _wbc_data->Fr_des[i][2] = _body_weight / 3.;
      _wbc_data->contact_state[i] = true;
    }

    _wbc_data->pFoot_des[0] = p_w_des;
    _wbc_data->contact_state[0] = false;
    _wbc_data->Fr_des[0][2] = 0;
  }

  _wbc_data->vBody_Ori_des.setZero();

  _wbc_ctrl->run(_wbc_data, *this->_data);

  // if (abs(p_err(0) < epsilon) && abs(p_err(1) < epsilon))
  // {
  //   // this->_data->legController->commands[0].zero();
  //   this->_data->legController->commands[0].tauFeedForward(0) = 0;
  //   this->_data->legController->commands[0].tauFeedForward(1) = 0;
  //   this->_data->legController->commands[0].tauFeedForward(2) = 0;

  //   this->_data->legController->commands[0].forceFeedForward(0) = 0;
  //   this->_data->legController->commands[0].forceFeedForward(1) = 0;
  //   this->_data->legController->commands[0].forceFeedForward(2) = 0;
  //   ROS_WARN_ONCE("give hand (no tau) 2");
  // }

  this->_data->debug->body_info.pos_des = ros::toMsg(_wbc_data->pBody_des);
  this->_data->debug->body_info.euler_des.x = _wbc_data->pBody_RPY_des[0];
  this->_data->debug->body_info.euler_des.y = _wbc_data->pBody_RPY_des[1];
  this->_data->debug->body_info.euler_des.z = _wbc_data->pBody_RPY_des[2];

  for (uint8_t foot = 0; foot < 4; foot++)
  {
    geometry_msgs::Point point;
    point = ros::toMsg(this->_data->legController->datas[foot].p + this->_data->quadruped->getHipLocation(foot));
    this->_data->debug->last_p_local_stance[foot] = point;
    this->_data->debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(_wbc_data->pFoot_des[foot]);
    this->_data->debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(seResult.position + seResult.rBody.transpose() * (this->_data->quadruped->getHipLocation(0) + this->_data->legController->datas[foot].p));
  }

  this->_data->debug->all_legs_info.leg.at(0).p_des = ros::toMsg(p_des);
}

// template class FSM_State_BalanceStand<double>;
template class FSM_State_BalanceStand<float>;
