/*============================= Testing ==============================*/
/**
 * State for be2r testing cases
 */

#include "FSM_State_Testing_cv.h"

using namespace std;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_Testing_Cv<T>::FSM_State_Testing_Cv(ControlFSMData<T>* _controlFSMData)
  : FSM_State<T>(_controlFSMData, FSM_StateName::TESTING_CV, "TESTING_CV"),
    _ini_foot_pos(4)
{
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;

  CMPC = new CMPCLocomotion_Cv(_controlFSMData->staticParams->controller_dt, ITERATIONS_BETWEEN_MPC, _controlFSMData);

  this->turnOnAllSafetyChecks();

  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;

  // Initialize GRF and footstep locations to 0s
  this->footFeedForwardForces = Mat34<T>::Zero();
  this->footstepLocations = Mat34<T>::Zero();
  _wbc_ctrl = new LocomotionCtrl<T>(_controlFSMData->quadruped->buildModel());
  _wbc_data = new LocomotionCtrlData<T>();

  ros::readParam("~map_topic_filter", map_topic_filter, std::string("elevation_map_raw"));
  ros::readParam("~map_topic_raw", map_topic_raw, std::string("elevation_map_raw"));
  ros::readParam("~map_plane_seg", map_topic_plane, std::string("elevation_map_raw"));
  _map_sub = _nh.subscribe<grid_map_msgs::GridMap>(map_topic_filter, 1, &FSM_State_Testing_Cv<T>::_elevMapCallback, this);
  _map_raw_sub = _nh.subscribe<grid_map_msgs::GridMap>(map_topic_raw, 1, &FSM_State_Testing_Cv<T>::_elevMapRawCallback, this);
  _map_plane_sub =
    _nh.subscribe<grid_map_msgs::GridMap>(map_topic_plane, 1, &FSM_State_Testing_Cv<T>::_elevMapPlaneCallback, this);
}

template<typename T>
void FSM_State_Testing_Cv<T>::_elevMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, _grid_map);
  if (!_grid_map.isDefaultStartIndex())
    _grid_map.convertToDefaultStartIndex();
  CMPC->setGridMapFilter(_grid_map);
}

template<typename T>
void FSM_State_Testing_Cv<T>::_elevMapRawCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, _grid_map_raw);
  if (!_grid_map_raw.isDefaultStartIndex())
    _grid_map_raw.convertToDefaultStartIndex();
  CMPC->setGridMapRaw(_grid_map_raw);
}

template<typename T>
void FSM_State_Testing_Cv<T>::_elevMapPlaneCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, _grid_map_plane);
  if (!_grid_map_plane.isDefaultStartIndex())
    _grid_map_plane.convertToDefaultStartIndex();
  CMPC->setGridMapPlane(_grid_map_plane);
}

template<typename T>
void FSM_State_Testing_Cv<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  CMPC->initialize();

  this->_data->gaitScheduler->gaitData._nextGait = GaitType::TROT;

  // Reset iteration counter
  iter = 0;

  for (size_t leg(0); leg < 4; ++leg)
  {
    _ini_foot_pos[leg] = this->_data->legController->datas[leg].p;

    firstSwing[leg] = true;
  }
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
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_Testing_Cv<T>::run()
{
  LocomotionControlStep();
}

template<typename T>
void FSM_State_Testing_Cv<T>::test1()
{
  static bool start = false;
  static Vec3<T> qInit(0, 0, 0);
  static Vec3<T> qDes(0, 0, 0);
  static int sin_count = 0;
  static float sin_mid_q[3] = { 0.0, 1.2, -2.0 };
  static Vec3<T> Kp(0, 0, 0);
  static Vec3<T> Kd(0, 0, 0);
  static Vec3<T> tau(0, 0, 0);

  if (start == false)
  {
    qInit = this->_data->legController->datas[0].q;
    start = true;
  }

  float rate = iter / 200.0; // needs count to 200
  // Kp[0] = 5.0;
  // Kp[1] = 5.0;
  // Kp[2] = 5.0;
  // Kd[0] = 1.0;
  // Kd[1] = 1.0;
  // Kd[2] = 1.0;

  Kp[0] = this->_data->userParameters->Kp_joint_0;
  Kp[1] = this->_data->userParameters->Kp_joint_1;
  Kp[2] = this->_data->userParameters->Kp_joint_2;
  Kd[0] = this->_data->userParameters->Kd_joint_0;
  Kd[1] = this->_data->userParameters->Kd_joint_1;
  Kd[2] = this->_data->userParameters->Kd_joint_2;

  qDes[0] = LinearInterpolation(qInit[0], sin_mid_q[0], rate);
  qDes[1] = LinearInterpolation(qInit[1], sin_mid_q[1], rate);
  qDes[2] = LinearInterpolation(qInit[2], sin_mid_q[2], rate);

  float sin_joint1, sin_joint2;

  sin_count++;
  sin_joint1 = 0.5 * sin(3 * M_PI * sin_count / 3000.0);
  sin_joint2 = -0.5 * sin(1.8 * M_PI * sin_count / 3000.0);
  qDes[0] = sin_mid_q[0];
  // qDes[1] = -sin_mid_q[1] - sin_joint1;
  qDes[1] = -sin_mid_q[1];
  qDes[2] = -sin_mid_q[2] - sin_joint2;

  // tau[0] =
  //   Kp[0] * (qDes[0] - this->_data->legController->datas[0].q(0)) + Kd[0] * (0 - this->_data->legController->datas[0].qd(0));
  // tau[1] =
  //   Kp[1] * (qDes[1] - this->_data->legController->datas[0].q(1)) + Kd[1] * (0 - this->_data->legController->datas[0].qd(1));
  // tau[2] =
  //   Kp[2] * (qDes[2] - this->_data->legController->datas[0].q(2)) + Kd[2] * (0 - this->_data->legController->datas[0].qd(2));

  this->_data->legController->commands[0].kpJoint(0, 0) = Kp[0];
  this->_data->legController->commands[0].kpJoint(1, 1) = Kp[1];
  this->_data->legController->commands[0].kpJoint(2, 2) = Kp[2];
  this->_data->legController->commands[0].kdJoint(0, 0) = Kd[0];
  this->_data->legController->commands[0].kdJoint(1, 1) = Kd[1];
  this->_data->legController->commands[0].kdJoint(2, 2) = Kd[2];

  this->_data->legController->commands[0].qDes = qDes;
  this->_data->legController->commands[0].qdDes = Vec3<T>(0, 0, 0);

  // this->_data->legController->commands[0].kpJoint(0, 0) = 0;
  // this->_data->legController->commands[0].kpJoint(1, 1) = 0;
  // this->_data->legController->commands[0].kpJoint(2, 2) = 0;
  // this->_data->legController->commands[0].kdJoint(0, 0) = 0;
  // this->_data->legController->commands[0].kdJoint(1, 1) = 0;
  // this->_data->legController->commands[0].kdJoint(2, 2) = 0;
  // this->_data->legController->commands[0].qDes = Vec3<T>(0, 0, 0);
  // this->_data->legController->commands[0].qdDes = Vec3<T>(0, 0, 0);

  // this->_data->legController->commands[0].tauFeedForward = tau;
}

template<typename T>
void FSM_State_Testing_Cv<T>::safeJointTest()
{
  for (size_t i = 0; i < 4; i++)
  {
    this->_data->legController->commands[i].kpJoint(0, 0) = 0;
    this->_data->legController->commands[i].kpJoint(1, 1) = 0;
    this->_data->legController->commands[i].kpJoint(2, 2) = 0;

    this->_data->legController->commands[i].kdJoint(0, 0) = 3;
    this->_data->legController->commands[i].kdJoint(1, 1) = 3;
    this->_data->legController->commands[i].kdJoint(2, 2) = 3;

    this->_data->legController->commands[i].qdDes(0) = 0;
    this->_data->legController->commands[i].qdDes(1) = 0;
    this->_data->legController->commands[i].qdDes(2) = 0;
  }

  // this->_data->legController->edampCommand(4);
}

// impedance test
template<typename T>
void FSM_State_Testing_Cv<T>::test2(float h)
{
  // float rate = 1;
  float rate = 0.5;
  float duration = 1 / rate;
  auto& seResult = this->_data->stateEstimator->getResult();
  static bool is_start = true;

  Vec3<float> p0(0, -0.15, -0.2);
  Vec3<float> p1(0, -0.25, -0.2);

  // Vec3<float> p0(0, -0.1, -0.2);
  // Vec3<float> p1(0, -0.3, -0.2);

  // near sholder
  // x 0.047
  // y -0.15
  // z -0.073

  // far 1
  // x 0.068
  // y -0.255
  // z -0.259

  // far 2
  // x -0.178
  // y -0.163
  // z -0.2
  Vec3<float> pDes(0.047, -0.15, -0.073);
  // static Vec3<float> pDes1(0.047, -0.15, -0.073);
  // static Vec3<float> pDes1(0.047, -0.15, -0.1);
  // static Vec3<float> pDes1(-0.178, -0.163, -0.2);
  // static Vec3<float> pDes0(_ini_foot_pos[0]);
  // static Vec3<float> pDes0(0.068, -0.255, -0.259);
  static bool flag = false;

  T progress = rate * iter * this->_data->staticParams->controller_dt;

  auto _model = this->_data->quadruped->buildModel();

  FBModelState<float> _state;
  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);

  _state.bodyOrientation = seResult.orientation;
  _state.bodyPosition = seResult.position;
  DVec<T> _full_config(cheetah::num_act_joint + 7);

  _full_config.setZero();

  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = seResult.omegaBody[i];
    _state.bodyVelocity[i + 3] = seResult.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = this->_data->legController->datas[leg].q[i];
      _state.qd[3 * leg + i] = this->_data->legController->datas[leg].qd[i];

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    }
  }

  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  auto _A = _model.getMassMatrix();
  auto _grav = _model.getGravityForce();
  auto _coriolis = _model.getCoriolisForce();

  // cout << "grav: " << _grav << endl;
  Vec3<float> tau = _grav.segment(6, 3);

  // tau(0) = tau(0);
  // tau(1) = 0;
  // tau(2) = 0;
  // cout << "grav leg0: " << tau << endl;
  // cout << "c+g: " << _grav+_coriolis << endl;

  if (progress > 1)
  {
    // progress = duration;
    progress = 0;
    iter = 0;
    flag = !flag;
    is_start = false;
  }

  // for real
  // float p = 1200;
  // float d = 15;

  // for sim
  // float p = 800;
  // float d = 15;

  this->_data->legController->setLegEnabled(0, true);
  this->_data->legController->setLegEnabled(1, false);
  this->_data->legController->setLegEnabled(2, false);
  this->_data->legController->setLegEnabled(3, false);

  uint8_t foot = 0;

  if (firstSwing[foot])
  {
    firstSwing[foot] = false;
    footSwingTrajectories[foot].setHeight(h);
    footSwingTrajectories[foot].setInitialPosition(_ini_foot_pos[foot]);
    footSwingTrajectories[foot].setFinalPosition(p0);
  }

  this->_data->legController->commands[foot].kpCartesian =
    Vec3<float>(this->_data->userParameters->Kp_cartesian_0, this->_data->userParameters->Kp_cartesian_1,
                this->_data->userParameters->Kp_cartesian_2)
      .asDiagonal();
  this->_data->legController->commands[foot].kdCartesian =
    Vec3<float>(this->_data->userParameters->Kd_cartesian_0, this->_data->userParameters->Kd_cartesian_1,
                this->_data->userParameters->Kd_cartesian_2)
      .asDiagonal();

  // this->_data->legController->commands[foot].pDes = pDes;
  // this->_data->legController->commands[foot].vDes = Vec3<float>::Constant(0);
  this->_data->legController->commands[foot].tauFeedForward = tau;

  if (!is_start)
  {
    if (flag == 0)
    {
      footSwingTrajectories[foot].setInitialPosition(p1);
      footSwingTrajectories[foot].setFinalPosition(p0);
    }
    else if (flag == 1)
    {
      footSwingTrajectories[foot].setInitialPosition(p0);
      footSwingTrajectories[foot].setFinalPosition(p1);
    }
  }

  footSwingTrajectories[foot].computeSwingTrajectoryBezier(progress, 2);

  Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
  Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
  // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) -
  // this->_data->quadruped->getHipLocation(foot); Vec3<float> vDesLeg = seResult.rBody *
  // (vDesFootWorld - seResult.vWorld);

  // this->_data->legController->commands[foot].pDes = pDesLeg;
  // this->_data->legController->commands[foot].vDes = vDesLeg;
  this->_data->legController->commands[foot].pDes = pDesFootWorld;
  this->_data->legController->commands[foot].vDes = vDesFootWorld;
  this->_data->debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesFootWorld);
  this->_data->debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesFootWorld);

  // Vec3<float> p_des = footSwingTrajectories[0].getPosition();
  // Vec3<float> v_des = footSwingTrajectories[0].getVelocity();

  // static Vec3<float> q_des(0, 0, 0);
  // static Vec3<float> dq_des(0, 0, 0);
  // q_des(1) = -0.5 * sin((float)iter / 1000.0) - 0.5 - 0.5;
  // q_des(2) = 0.5 * sin((float)iter / 1000.0) + 0.5 + 1.5;

  // Vec3<float> q_des(0, 0, 0);
  // Vec3<float> dq_des(0, 0, 0);

  // q_des = this->findAngles(0, p_des);

  // this->jointPDControl(0, q_des, dq_des);
  // this->lowLeveljointPDControl(0, q_des, dq_des);
  // this->_data->legController->is_low_level = true;

  // Vec3<float> p_act = this->_data->legController->datas[0].p;
  // Vec3<float> q_eval = this->findAngles(0, p_act);
  // cout << "q_eval: " << q_eval << endl;

  static Vec3<T> q(0, 0, 0);
  static Vec3<T> dq(0, 0, 0);
  static Vec3<T> ddq(0, 0, 0);
  static T dt = 0.002;

  Mat3<T> M = _A.block(6, 6, 3, 3);
  Vec3<T> C = _coriolis.block(6, 0, 3, 1);
  Vec3<T> G = _grav.block(6, 0, 3, 1);
  Vec3<T> tau_final(0, 0, 0);
  tau_final =
    tau + this->_data->legController->datas[0].J.transpose() *
            (this->_data->legController->commands[0].kpCartesian * (pDesFootWorld - this->_data->legController->datas[0].p) +
             this->_data->legController->commands[0].kdCartesian * (vDesFootWorld - this->_data->legController->datas[0].v));

  ddq = M.inverse() * (tau_final - C - G);
  dq = dq + ddq * dt;

  // ROS_INFO("time");

  // cout << "C+G: " << C + G << endl;
  // cout << "M: " << M << endl;
  cout << "ddq: " << ddq << endl;
  cout << "dq: " << dq << endl;
  // cout << "tau_final: " << tau_final << endl;
}

template<typename T>
void FSM_State_Testing_Cv<T>::gravTest()
{
  float rate = 0.5;
  auto& seResult = this->_data->stateEstimator->getResult();
  static bool is_start = true;

  Vec3<float> p0(0, -0.15, -0.2);
  Vec3<float> p1(0, -0.25, -0.2);

  Vec3<float> pDes(0.047, -0.15, -0.073);

  static bool flag = false;

  T progress = rate * iter * this->_data->staticParams->controller_dt;

  auto _model = this->_data->quadruped->buildModel();

  FBModelState<float> _state;
  _state.q = DVec<T>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<T>::Zero(cheetah::num_act_joint);

  _state.bodyOrientation = seResult.orientation;
  _state.bodyPosition = seResult.position;
  DVec<T> _full_config(cheetah::num_act_joint + 7);

  _full_config.setZero();

  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = seResult.omegaBody[i];
    _state.bodyVelocity[i + 3] = seResult.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = this->_data->legController->datas[leg].q[i];
      _state.qd[3 * leg + i] = this->_data->legController->datas[leg].qd[i];

      _full_config[3 * leg + i + 6] = _state.q[3 * leg + i];
    }
  }

  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  auto _A = _model.getMassMatrix();
  auto _grav = _model.getGravityForce();
  auto _coriolis = _model.getCoriolisForce();

  cout << "grav: " << _grav << endl;
  Vec3<float> tau = _grav.segment(6, 3);

  cout << "grav leg0: " << tau << endl;

  this->_data->legController->setLegEnabled(0, true);
  this->_data->legController->setLegEnabled(1, false);
  this->_data->legController->setLegEnabled(2, false);
  this->_data->legController->setLegEnabled(3, false);

  for (int foot = 0; foot < 4; foot++)
  {
    this->_data->legController->commands[foot].tauFeedForward = tau;
  }
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_Testing_Cv<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State)
  {
    case K_TESTING_CV:
      break;

    case K_STAND_UP:
      // Requested switch to Stand Up
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    case K_PASSIVE: // normal c
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    case K_VISION:
      this->nextStateName = FSM_StateName::VISION;
      break;

    case K_TESTING:
      this->nextStateName = FSM_StateName::TESTING;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from " << K_TESTING_CV << " to "
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
TransitionData<T> FSM_State_Testing_Cv<T>::transition()
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

    case FSM_StateName::BALANCE_STAND:
      this->transitionData.done = true;
      break;

    case FSM_StateName::VISION:
      this->transitionData.done = true;
      break;

    case FSM_StateName::TESTING:
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
void FSM_State_Testing_Cv<T>::onExit()
{
  // Nothing to clean up when exiting
  // this->_data->legController->zeroCommand();

  this->_data->legController->setEnabled(false);
}

template<typename T>
void FSM_State_Testing_Cv<T>::LocomotionControlStep()
{
  // Contact state logic
  // estimateContact();

  CMPC->myVersion(*this->_data);

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
  }

  if (this->_data->userParameters->use_wbc)
  {
    _wbc_data->pBody_des = CMPC->pBody_des;
    _wbc_data->vBody_des = CMPC->vBody_des;
    _wbc_data->aBody_des = CMPC->aBody_des;

    _wbc_data->pBody_RPY_des = CMPC->pBody_RPY_des;
    _wbc_data->vBody_Ori_des = CMPC->vBody_Ori_des;

    for (size_t i(0); i < 4; ++i)
    {
      _wbc_data->pFoot_des[i] = CMPC->pFoot_des[i];
      _wbc_data->vFoot_des[i] = CMPC->vFoot_des[i];
      _wbc_data->aFoot_des[i] = CMPC->aFoot_des[i];
      _wbc_data->Fr_des[i] = CMPC->Fr_des[i];
    }
    _wbc_data->contact_state = CMPC->contact_state;
    _wbc_ctrl->run(_wbc_data, *this->_data);
  }

  for (int leg(0); leg < 4; ++leg)
  {
    // originally commented
    this->_data->legController->commands[leg].pDes = pDes_backup[leg];
    this->_data->legController->commands[leg].vDes = vDes_backup[leg];

    this->_data->legController->commands[leg].kpCartesian = Kp_backup[leg];
    this->_data->legController->commands[leg].kdCartesian = Kd_backup[leg];
  }
}

template<typename T>
bool FSM_State_Testing_Cv<T>::locomotionSafe()
{
  auto& seResult = this->_data->stateEstimator->getResult();

  const T max_roll = 40;
  const T max_pitch = 40;

  if (std::fabs(seResult.rpy[0]) > ori::deg2rad(max_roll))
  {
    printf("Unsafe locomotion: roll is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[0]), max_roll);
    return false;
  }

  if (std::fabs(seResult.rpy[1]) > ori::deg2rad(max_pitch))
  {
    printf("Unsafe locomotion: pitch is %.3f degrees (max %.3f)\n", ori::rad2deg(seResult.rpy[1]), max_pitch);
    return false;
  }

  for (int leg = 0; leg < 4; leg++)
  {
    auto p_leg = this->_data->legController->datas[leg].p;
    if (p_leg[2] > 0)
    {
      printf("Unsafe locomotion: leg %d is above hip (%.3f m)\n", leg, p_leg[2]);
      return false;
    }

    if (std::fabs(p_leg[1] > 0.18))
    {
      printf("Unsafe locomotion: leg %d's y-position is bad (%.3f m)\n", leg, p_leg[1]);
      return false;
    }

    auto v_leg = this->_data->legController->datas[leg].v.norm();
    if (std::fabs(v_leg) > 9.)
    {
      printf("Unsafe locomotion: leg %d is moving too quickly (%.3f m/s)\n", leg, v_leg);
      return false;
    }
  }

  return true;
}

// template class FSM_State_Testing_Cv<double>;
template class FSM_State_Testing_Cv<float>;
