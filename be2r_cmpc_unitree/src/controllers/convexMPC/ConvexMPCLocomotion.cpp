#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <Utilities/utilities.h>
#include <iostream>

#include "ConvexMPCLocomotion.h"
#include "GraphSearch.h"
#include "convexMPC_interface.h"

#include "Gait.h"

// оригинальные параметры MPC+WBC
//  #define GAIT_PERIOD 14
#define HORIZON 16

#define GAIT_PERIOD 20
// #define GAIT_PERIOD 34 //1000 Hz

// лучшие параметры для только MPC
//  #define GAIT_PERIOD 18
//  #define HORIZON 5

#define STEP_HEIGHT 0.06
#define BODY_HEIGHT 0.24

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int iterations_between_mpc, ControlFSMData<float>* data)
  : _fsm_data(data),
    _iterationsBetweenMPC(iterations_between_mpc),
    _dyn_params(data->userParameters),
    _gait_period(_dyn_params->gait_period),
    horizonLength(_fsm_data->staticParams->horizon),
    dt(_dt),
    trotting(_gait_period, Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0), Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0), "Trotting"),
    bounding(_gait_period, Vec4<int>(5, 5, 0, 0), Vec4<int>(4, 4, 4, 4), "Bounding"),
    pronking(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(8, 8, 8, 8), "Pronking"),
    jumping(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(2, 2, 2, 2), "Jumping"),
    galloping(_gait_period, Vec4<int>(0, 2, 7, 9), Vec4<int>(4, 4, 4, 4), "Galloping"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    trotRunning(_gait_period, Vec4<int>(0, 5, 5, 0), Vec4<int>(4, 4, 4, 4), "Trot Running"),
    walking(_gait_period, Vec4<int>(2 * _gait_period / 4., 0, _gait_period / 4., 3 * _gait_period / 4.), Vec4<int>(0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period), "Walking"),
    walking2(_gait_period, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7), "Walking2"),
    pacing(_gait_period, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"),
    random(_gait_period, Vec4<int>(9, 13, 13, 9), 0.4, "Flying nine thirteenths trot"),
    random2(_gait_period, Vec4<int>(8, 16, 16, 8), 0.5, "Double Trot")
{
  ground_truth_sub = _nh.subscribe("/ground_truth_odom", 1, &ConvexMPCLocomotion::groundTruthCallback, this);
  log_data_pub = _nh.advertise<unitree_legged_msgs::LogData>("/log_data", 10);
  log_data_sub = _nh.subscribe("/log_data", 10, &ConvexMPCLocomotion::logDataCallback, this);

  ROS_WARN_STREAM("Current gait period " << _dyn_params->gait_period);
  dtMPC = dt * _iterationsBetweenMPC;
  default_iterations_between_mpc = _iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, _iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 120); // original (3d arg prev: 1200, 650)
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  _yaw_des = 0;
  _pitch_des = 0.;

  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  _iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float>& data)
{
  _body_height = _dyn_params->body_height;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);

  _yaw_turn_rate = data.gamepad_command->right_stick_analog[0];
  x_vel_cmd = data.gamepad_command->left_stick_analog[1];
  y_vel_cmd = data.gamepad_command->left_stick_analog[0];

  _yaw_turn_rate *= data.staticParams->max_turn_rate;
  x_vel_cmd *= data.staticParams->max_vel_x;
  y_vel_cmd *= data.staticParams->max_vel_y;

  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

  //_yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  _yaw_des = 0.;
  _roll_des = 0.;
  _pitch_des = 0.;

  // Update PD coefs
  Kp = Vec3<float>(_dyn_params->Kp_cartesian_0, _dyn_params->Kp_cartesian_1, _dyn_params->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_dyn_params->Kd_cartesian_0, _dyn_params->Kd_cartesian_1, _dyn_params->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data)
{
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;

  auto& seResult = data.stateEstimator->getResult();

  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.3;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // cout << "[ConvexMPCLocomotion] get res done" << endl;

  // pick gait
  Gait* gait = &trotting;
  if (gaitNumber == 1)
  {
    gait = &bounding;
  }
  else if (gaitNumber == 2)
  {
    gait = &pronking;
  }
  else if (gaitNumber == 3)
  {
    gait = &jumping;
  }
  else if (gaitNumber == 4)
  {
    gait = &standing;
  }
  else if (gaitNumber == 5)
  {
    gait = &trotRunning;
  }
  else if (gaitNumber == 6)
  {
    gait = &galloping;
  }
  // else if (gaitNumber == 7)
  // {
  //   gait = &random2;
  // }
  else if (gaitNumber == 8)
  {
    gait = &pacing;
  }
  else if (gaitNumber == 10)
  {
    gait = &walking;
  }
  else if (gaitNumber == 11)
  {
    gait = &walking2;
  }
  current_gait = gaitNumber;

  // gait->updatePeriod(_dyn_params->gait_period);
  //  gait->restoreDefaults();
  gait->setIterations(_iterationsBetweenMPC, iterationCounter);
  //  gait->earlyContactHandle(seResult.contactSensor, _iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //                      Pitch compensation
  static Vec3<float> pDesFootWorldStance[4] = { pFoot[0], pFoot[1], pFoot[2], pFoot[3] };

  // Integral-esque pitche and roll compensation
  if (fabs(v_robot[0]) > .2) // avoid dividing by zero
  {
    rpy_int[1] += dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 8); // turn off for pronking

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position + seResult.rBody.transpose() * (data.quadruped->getHipLocation(i) + data.legController->datas[i].p);
  }

  if (gait != &standing)
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  if (gait == &standing)
  {
    _yaw_des = 0.0;
  }


  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    // world_position_desired[2] = seResult.rpy[2];
    world_position_desired[2] = _body_height;
    //_yaw_des = seResult.rpy[2];
    _yaw_des = 0.;

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_dyn_params->Swing_traj_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);
    }

    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
  {
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);
  }

  float side_sign[4] = { -1, 1, -1, 1 };
  float interleave_y[4] = { -0.08, 0.08, 0.02, -0.02 };
  float interleave_gain = -0.2;
  float v_abs = std::fabs(v_des_robot[0]);

  for (int i = 0; i < 4; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }

    footSwingTrajectories[i].setHeight(_dyn_params->Swing_traj_height);

    Vec3<float> offset(0, side_sign[i] * data.quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (data.quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    float pfx_rel = seResult.vWorld[0] * (.5 + _dyn_params->cmpc_bonus_swing) * stance_time + .03f * (seResult.vWorld[0] - v_des_world[0]) + (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC + .03f * (seResult.vWorld[1] - v_des_world[1]) + (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = 0.0;

    footSwingTrajectories[i].setFinalPosition(Pf);
    data.debug->all_legs_info.leg[i].swing_pf.x = Pf(0);
    data.debug->all_legs_info.leg[i].swing_pf.y = Pf(1);
    data.debug->all_legs_info.leg[i].swing_pf.z = Pf(2);
  }

  // calc gait
  iterationCounter++;

  // gait
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);
  static bool is_stance[4] = { 0, 0, 0, 0 };
  static Vec3<float> p_fw[4] = {};
  static Vec3<float> p_fl[4] = {};
  static float delta_yaw[4] = {};
  static Vec3<float> delta_p_bw[4] = {};

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    if ((is_stance[foot] == 0) && !(swingState > 0))
    {
      is_stance[foot] = 1;

      // foot position in world frame at contanct
      pDesFootWorldStance[foot] = pFoot[foot];
      data.debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];

      p_fl[foot] = data.legController->datas[foot].p + data.quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    delta_p_bw[foot] += seResult.vBody * dt;
    delta_yaw[foot] += seResult.omegaBody(2) * dt;
    data.debug->last_p_local_stance[foot] = ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;
      pDesFootWorldStance[foot] = pFoot[foot];

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> vDesFootWorld = Vec3<float>::Zero();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      // temporary debug
      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);

      if (!data.userParameters->use_wbc) // wbc off
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;

        data.legController->commands[foot].forceFeedForward = f_ff[foot];
        data.legController->commands[foot].kdJoint = Vec3<float>(_dyn_params->Kd_joint_0, _dyn_params->Kd_joint_1, _dyn_params->Kd_joint_2).asDiagonal();
      }
      else
      { // Stance foot damping
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;
      }

      se_contactState[foot] = contactState;

      data.debug->all_legs_info.leg.at(foot).p_des = ros::toMsg(pDesLeg);
      data.debug->all_legs_info.leg.at(foot).v_des = ros::toMsg(vDesLeg);
      data.debug->all_legs_info.leg.at(foot).p_w_act = ros::toMsg(pFoot[foot]);
      data.debug->all_legs_info.leg.at(foot).v_w_act = ros::toMsg(vActFootWorld);
      data.debug->all_legs_info.leg.at(foot).p_w_des = ros::toMsg(pDesFootWorld);
      data.debug->all_legs_info.leg.at(foot).v_w_des = ros::toMsg(vDesFootWorld);

      data.debug->leg_force[foot] = ros::toMsg(f_ff[foot]);
    }

    publishLogData();

  }

  data.stateEstimator->setContactPhase(se_contactState);
  data.stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = 0. ; //v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  data.debug->body_info.pos_des.x = pBody_des[0];
  data.debug->body_info.pos_des.y = pBody_des[1];
  data.debug->body_info.pos_des.z = pBody_des[2];

  data.debug->body_info.vel_des.linear.x = vBody_des[0];
  data.debug->body_info.vel_des.linear.y = vBody_des[1];
  data.debug->body_info.vel_des.linear.z = vBody_des[2];

  data.debug->body_info.euler_des.x = pBody_RPY_des[0];
  data.debug->body_info.euler_des.y = pBody_RPY_des[1];
  data.debug->body_info.euler_des.z = pBody_RPY_des[2];

  data.debug->body_info.vel_des.angular.x = vBody_Ori_des[0];
  data.debug->body_info.vel_des.angular.y = vBody_Ori_des[1];
  data.debug->body_info.vel_des.angular.z = vBody_Ori_des[2];

  contact_state = gait->getContactState();
  // END of WBC Update
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data)
{
  (void)data;
  printf("call to old CMPC with double!\n");
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
{
  // _iterationsBetweenMPC = 30;
  if ((iterationCounter % _iterationsBetweenMPC) == 0)
  {
    auto seResult = data.stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    // float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    // printf("Position error: %.3f, integral %.3f\n", pxy_err[0],
    // x_comp_integral);

    // Stand gait
    if (current_gait == 4)
    {
      float trajInitial[12] = { _roll_des, _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/, (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/, (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/, (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/, (float)_body_height /*fsm->main_control_settings.p_des[2]*/, 0, 0, 0, 0, 0, 0 };

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];
    }
    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if (xStart - p[0] > max_pos_error)
        xStart = p[0] + max_pos_error;
      if (p[0] - xStart > max_pos_error)
        xStart = p[0] - max_pos_error;

      if (yStart - p[1] > max_pos_error)
        yStart = p[1] + max_pos_error;
      if (p[1] - yStart > max_pos_error)
        yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      float trajInitial[12] = { (float)rpy_comp[0], // 0
                                (float)rpy_comp[1], // 1
                                _yaw_des,           // 2
                                // yawStart,    // 2
                                xStart,              // 3
                                yStart,              // 4
                                (float)_body_height, // 5
                                0,                   // 6
                                0,                   // 7
                                _yaw_turn_rate,      // 8
                                v_des_world[0],      // 9
                                v_des_world[1],      // 10
                                0 };                 // 11

      for (int i = 0; i < horizonLength; i++)
      {
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];

        if (i == 0) // start at current position  TODO consider not doing this
        {
          // trajAll[3] = hw_i->state_estimator->se_pBody[0];
          // trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }

    Timer solveTimer;

    if (_dyn_params->cmpc_use_sparse > 0.5)
    {
      solveSparseMPC(mpcTable, data);
    }
    else
    {
      solveDenseMPC(mpcTable, data);
    }
    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

auto cross_mat = [](const Eigen::Matrix3f& I_inv, const Eigen::Vector3f& r) -> Eigen::Matrix3f {
  Eigen::Matrix3f cm;
  cm << 0.f,   -r(2),  r(1),
        r(2),   0.f,   -r(0),
        -r(1),   r(0),   0.f;
  return I_inv * cm;
};

Eigen::Matrix<float, 6, 1> f_ext;

void ConvexMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // original
  float Q[12] = { 0.25, 0.25, 10, 10, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1 };

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
  // float alpha = 4e-7; // make setting eventually: DH
  //std::cout << "seResult.position:\n" << seResult.position << "\n";
  //std::cout << "ground_truth_position:\n" << ground_truth_position << "\n";

  Vec3<float> p_v(seResult.position[0], seResult.position[1], ground_truth_position[2]);

  float* p = p_v.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();




//  МЕСТО ДЛЯ РЕАЛИЗАЦИИ ПРИЕМА ДАННЫХ ОТ ТОПИКА /log_data!!!!!!!!!!!!!!!!!!!
  if (received_log_data_) {
      // Создаем матрицу A_(k-1) размером 13x13 и заполняем её
      Eigen::Matrix<float, 13, 13> A_prev = Eigen::Matrix<float, 13, 13>::Zero();
      A_prev(3, 9) = 1.f;
      A_prev(11, 9) = last_log_data_received_.x_drag;  // Используем x_drag из предыдущего сообщения
      A_prev(4, 10) = 1.f;
      A_prev(5, 11) = 1.f;
      A_prev(11, 12) = 1.f;
      
      // Вычисляем матрицу поворота R_yaw из предыдущей ориентации (используем euler_act.z)
      //float yaw_prev = last_log_data_received_.euler_act.z;
      Eigen::Matrix3f R_yaw;
      R_yaw << last_log_data_received_.R_00, last_log_data_received_.R_01, last_log_data_received_.R_02,
              last_log_data_received_.R_10, last_log_data_received_.R_11, last_log_data_received_.R_12,
              last_log_data_received_.R_20, last_log_data_received_.R_21, last_log_data_received_.R_22;

      //std::cout << "R_yaw_test:\n" << R_yaw << '\n' << std::endl;

      A_prev.block<3, 3>(0, 6) = R_yaw.transpose();
      
      // Создаем матрицу B_(k-1) размером 13x12
      Eigen::Matrix<float, 13, 12> B_prev = Eigen::Matrix<float, 13, 12>::Zero();
      // Задаем инерционную матрицу I_world (значения можно заменить на актуальные)
      Eigen::Matrix3f I_body;
      Eigen::Matrix3f I_world;

      I_body << 0.07f, 0,      0,
                0,     0.26f,  0,
                0,     0,      0.242f;

      I_world = R_yaw * I_body * R_yaw.transpose();

      Eigen::Matrix3f I_inv = I_world.inverse();

      float m_robot = 12.f;  // масса робота (замените, если нужно)
      
      // Восстанавливаем положения ног (r_feet) из предыдущего сообщения
      Eigen::Matrix<float, 3, 4> r_feet_prev;
      r_feet_prev << last_log_data_received_.r_x_1, last_log_data_received_.r_x_2, last_log_data_received_.r_x_3, last_log_data_received_.r_x_4,
                    last_log_data_received_.r_y_1, last_log_data_received_.r_y_2, last_log_data_received_.r_y_3, last_log_data_received_.r_y_4,
                    last_log_data_received_.r_z_1, last_log_data_received_.r_z_2, last_log_data_received_.r_z_3, last_log_data_received_.r_z_4;
      

      // Заполнение блоков матрицы B_prev
      for (int b = 0; b < 4; b++) {
          B_prev.block<3, 3>(6, b * 3) = cross_mat(I_inv, r_feet_prev.col(b));
          // Блок для сил: B.block(9, b*3, 3, 3) = I/ m
          B_prev.block<3, 3>(9, b * 3) = Eigen::Matrix3f::Identity() / m_robot;
      }

      //std::cout << "A_prev:\n" << A_prev << '\n' << std::endl;
      // std::cout << "B_prev:\n" << B_prev << '\n' << std::endl;

      
      // Формируем текущий вектор состояния x_k (размер 13x1): [p; v; w; q]
      Eigen::Matrix<float, 13, 1> x_k;
      // p, v, w, q получаем из текущего результата state estimator:
      for (int i = 0; i < 3; i++) {
          x_k(i + 3)     = p[i];         // p (позиция)
          x_k(i + 9) = v[i];         // v (линейная скорость)
          x_k(i + 6) = w[i];         // w (угловая скорость)
      }

      x_k(0) = roll;
      x_k(1) = pitch;
      x_k(2) = yaw;

      x_k(12) = -9.81f;

      //std::cout << "x_k:\n" << x_k << '\n' << std::endl;

      
      // Из предыдущего сообщения извлекаем x_(k-1) и u_(k-1)
      // Для примера: x_prev составляем из pos_act, vel_act и euler_act; u_prev – из сил ног
      Eigen::Matrix<float, 13, 1> x_prev;
      x_prev(0)  = last_log_data_received_.euler_act.x;
      x_prev(1) = last_log_data_received_.euler_act.y;
      x_prev(2) = last_log_data_received_.euler_act.z;

      x_prev(3) = last_log_data_received_.pos_act.x;
      x_prev(4) = last_log_data_received_.pos_act.y;
      x_prev(5) = last_log_data_received_.pos_act.z;

      x_prev(6) = last_log_data_received_.vel_act.angular.x;
      x_prev(7) = last_log_data_received_.vel_act.angular.y;
      x_prev(8) = last_log_data_received_.vel_act.angular.z;

      x_prev(9) = last_log_data_received_.vel_act.linear.x;
      x_prev(10) = last_log_data_received_.vel_act.linear.y;
      x_prev(11) = last_log_data_received_.vel_act.linear.z;

      // Предположим, что в качестве ориентации берем углы Эйлера из euler_act (3 элемента) и дополняем до 4-элементного вектора
      x_prev(12) = -9.81f;
    
      // u_prev составляем из сил для каждой ноги (каждая сила – 3 элемента, итого 12)
      Eigen::Matrix<float, 12, 1> u_prev;
      // u_prev(0) = last_log_data_received_.foot_force0_x;
      // u_prev(1) = last_log_data_received_.foot_force0_y;
      // u_prev(2) = last_log_data_received_.foot_force0_z;
      // u_prev(3) = last_log_data_received_.foot_force1_x;
      // u_prev(4) = last_log_data_received_.foot_force1_y;
      // u_prev(5) = last_log_data_received_.foot_force1_z;
      // u_prev(6) = last_log_data_received_.foot_force2_x;
      // u_prev(7) = last_log_data_received_.foot_force2_y;
      // u_prev(8) = last_log_data_received_.foot_force2_z;
      // u_prev(9) = last_log_data_received_.foot_force3_x;
      // u_prev(10)= last_log_data_received_.foot_force3_y;
      // u_prev(11)= last_log_data_received_.foot_force3_z;

      u_prev(0) = -last_log_data_received_.foot_force0_x;
      u_prev(1) = -last_log_data_received_.foot_force0_y;
      u_prev(2) = -last_log_data_received_.foot_force0_z;
      u_prev(3) = -last_log_data_received_.foot_force1_x;
      u_prev(4) = -last_log_data_received_.foot_force1_y;
      u_prev(5) = -last_log_data_received_.foot_force1_z;
      u_prev(6) = -last_log_data_received_.foot_force2_x;
      u_prev(7) = -last_log_data_received_.foot_force2_y;
      u_prev(8) = -last_log_data_received_.foot_force2_z;
      u_prev(9) = -last_log_data_received_.foot_force3_x;
      u_prev(10)= -last_log_data_received_.foot_force3_y;
      u_prev(11)= -last_log_data_received_.foot_force3_z;
      
      // Вычисляем внешнюю силу: f_external = x_k - A_prev * x_prev - B_prev * u_prev
      Eigen::Matrix<float, 13, 1> f_external = x_k - A_prev * x_prev - B_prev * u_prev;

      Eigen::Matrix<float, 6, 1> f_ext_MPC = f_external.segment(6, 6);


      // f_ext = f_ext_MPC;

      // std::cout << "f_external\n: " << f_ext << std::endl;

      f_ext << -f_ext_MPC[0], -f_ext_MPC[1], f_ext_MPC[2], f_ext_MPC[3], f_ext_MPC[4], f_ext_MPC[5];

      //std::cout << "f_external\n: " << f_ext << std::endl;
      

  }
  // Если данные не получены, можно добавить обработку ошибки или вывести предупреждение
  else {
      ROS_WARN("Данные из /log_data еще не получены. Пропускаем вычисление f_external.");
  }





  float r[12];
  for (int i = 0; i < 12; i++)
  {
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];
  }


  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  // Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * _iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  // setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  //std::cout << "x_comp_integral:\n" << x_comp_integral << "\n";
  

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    // x_comp_integral += _dyn_params->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += _dyn_params->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_dyn_params->jcqp_max_iter, _dyn_params->jcqp_rho, _dyn_params->jcqp_sigma, _dyn_params->jcqp_alpha, _dyn_params->jcqp_terminate, _dyn_params->use_jcqp);
  // t1.stopPrint("Setup MPC");
  // printf("MPC Setup time %f ms\n", t1.getMs());

  //Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, roll, pitch, yaw, weights, trajAll, alpha, mpcTable);
  // t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg * 3 + axis);

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    //std::cout << "rBody_real:\n" <<seResult.rBody << std::endl;

    // Update for WBC
    Fr_des[leg] = f;


    last_mpc_data.timestamp = ros::Time::now();

    last_mpc_data.position = seResult.position;
    last_mpc_data.rpy = seResult.rpy;
    last_mpc_data.velocity = seResult.vWorld;
    last_mpc_data.angular_velocity = seResult.omegaWorld;

    for (int leg = 0; leg < 4; leg++) {
      last_mpc_data.forces[leg] = f_ff[leg];  // Силы в системе координат робота
    }

    for (int i = 0; i < 12; i++) {
      last_mpc_data.r_feet[i] = r[i];
    }

    last_mpc_data.x_drag = x_comp_integral;

    last_mpc_data.roll = roll;
    last_mpc_data.pitch = pitch;
    last_mpc_data.yaw = yaw;

  }
  //std::cout << f << '\n' << std::endl;
}

void ConvexMPCLocomotion::solveSparseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data.stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++)
  {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++)
  {
    for (u32 j = 0; j < 12; j++)
    {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++)
  {
    for (u32 axis = 0; axis < 3; axis++)
    {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++)
  {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
    // printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC()
{
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  double mass = 12;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++)
  {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  // weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(1.0);
  // _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}


void ConvexMPCLocomotion::groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ground_truth_position[0] = msg->pose.pose.position.x;
    ground_truth_position[1] = msg->pose.pose.position.y;
    ground_truth_position[2] = msg->pose.pose.position.z;

    ground_truth_velocity[0] = msg->twist.twist.linear.x;
    ground_truth_velocity[1] = msg->twist.twist.linear.y;
    ground_truth_velocity[2] = msg->twist.twist.linear.z;

    ground_truth_velocity_ang[0] = msg->twist.twist.angular.x;
    ground_truth_velocity_ang[1] = msg->twist.twist.angular.y;
    ground_truth_velocity_ang[2] = msg->twist.twist.angular.z;

    Eigen::Quaternionf q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);

    Eigen::Matrix3f R = q.toRotationMatrix();

    float yaw = std::atan2(R(1, 0), R(0, 0));
    float pitch = std::asin(-R(2, 0));
    float roll = std::atan2(R(2, 1), R(2, 2));

    ground_truth_rpy = {roll, pitch, yaw};

}



void ConvexMPCLocomotion::publishLogData()
{
    unitree_legged_msgs::LogData log_msg;

    // Используем сохраненные данные из момента вызова MPC
    log_msg.header.stamp = last_mpc_data.timestamp;

    log_msg.pos_act.x = last_mpc_data.position[0];
    log_msg.pos_act.y = last_mpc_data.position[1];
    log_msg.pos_act.z = ground_truth_position[2];

    log_msg.euler_act.x = last_mpc_data.rpy[0];
    log_msg.euler_act.y = last_mpc_data.rpy[1];
    log_msg.euler_act.z = last_mpc_data.rpy[2];

    log_msg.vel_act.linear.x = last_mpc_data.velocity[0];
    log_msg.vel_act.linear.y = last_mpc_data.velocity[1];
    log_msg.vel_act.linear.z = last_mpc_data.velocity[2];

    log_msg.vel_act.angular.x = last_mpc_data.angular_velocity[0];
    log_msg.vel_act.angular.y = last_mpc_data.angular_velocity[1];
    log_msg.vel_act.angular.z = last_mpc_data.angular_velocity[2];


    log_msg.foot_force0_x = last_mpc_data.forces[0][0];
    log_msg.foot_force0_y = last_mpc_data.forces[0][1];
    log_msg.foot_force0_z = last_mpc_data.forces[0][2];

    log_msg.foot_force1_x = last_mpc_data.forces[1][0];
    log_msg.foot_force1_y = last_mpc_data.forces[1][1];
    log_msg.foot_force1_z = last_mpc_data.forces[1][2];

    log_msg.foot_force2_x = last_mpc_data.forces[2][0];
    log_msg.foot_force2_y = last_mpc_data.forces[2][1];
    log_msg.foot_force2_z = last_mpc_data.forces[2][2];

    log_msg.foot_force3_x = last_mpc_data.forces[3][0];
    log_msg.foot_force3_y = last_mpc_data.forces[3][1];
    log_msg.foot_force3_z = last_mpc_data.forces[3][2];



    // // Заполнение pos_act (текущая позиция)
    // log_msg.pos_act.x = ground_truth_position[0];
    // log_msg.pos_act.y = ground_truth_position[1];
    // log_msg.pos_act.z = ground_truth_position[2];

    // // Заполнение ориентации (углы Эйлера)
    // log_msg.euler_act.x = ground_truth_rpy[0];
    // log_msg.euler_act.y = ground_truth_rpy[1];
    // log_msg.euler_act.z = ground_truth_rpy[2];

    // // Заполнение скоростей
    // log_msg.vel_act.linear.x = ground_truth_velocity[0];
    // log_msg.vel_act.linear.y = ground_truth_velocity[1];
    // log_msg.vel_act.linear.z = ground_truth_velocity[2];

    // log_msg.vel_act.angular.x = ground_truth_velocity_ang[0];
    // log_msg.vel_act.angular.y = ground_truth_velocity_ang[1];
    // log_msg.vel_act.angular.z = ground_truth_velocity_ang[2];


    // Заполнение pos_des (желаемая позиция)
    log_msg.pos_des.x = pBody_des[0];
    log_msg.pos_des.y = pBody_des[1];
    log_msg.pos_des.z = pBody_des[2];

    // Заполнение euler_des (желаемые углы Эйлера)
    log_msg.euler_des.x = pBody_RPY_des[0];
    log_msg.euler_des.y = pBody_RPY_des[1];
    log_msg.euler_des.z = pBody_RPY_des[2];

    // Заполнение vel_des
    log_msg.vel_des.linear.x = vBody_des[0];
    log_msg.vel_des.linear.y = vBody_des[1];
    log_msg.vel_des.linear.z = vBody_des[2];
    
    log_msg.vel_des.angular.x = vBody_Ori_des[0];
    log_msg.vel_des.angular.y = vBody_Ori_des[1];
    log_msg.vel_des.angular.z = vBody_Ori_des[2];

    //std::cout << "r:\n" << log_msg.r << "\n";

    log_msg.x_drag = last_mpc_data.x_drag;


    log_msg.r_x_1 = last_mpc_data.r_feet[0];
    log_msg.r_x_2 = last_mpc_data.r_feet[1];
    log_msg.r_x_3 = last_mpc_data.r_feet[2];
    log_msg.r_x_4 = last_mpc_data.r_feet[3];

    log_msg.r_y_1 = last_mpc_data.r_feet[4];
    log_msg.r_y_2 = last_mpc_data.r_feet[5];
    log_msg.r_y_3 = last_mpc_data.r_feet[6];
    log_msg.r_y_4 = last_mpc_data.r_feet[7];

    log_msg.r_z_1 = last_mpc_data.r_feet[8];
    log_msg.r_z_2 = last_mpc_data.r_feet[9];
    log_msg.r_z_3 = last_mpc_data.r_feet[10];
    log_msg.r_z_4 = last_mpc_data.r_feet[11];



    // Поворот вокруг оси X (Roll)
    R_x << 1, 0, 0,
           0, cos(last_mpc_data.roll), -sin(last_mpc_data.roll),
           0, sin(last_mpc_data.roll), cos(last_mpc_data.roll);

    // Поворот вокруг оси Y (Pitch)
    R_y << cos(last_mpc_data.pitch), 0, sin(last_mpc_data.pitch),
           0, 1, 0,
           -sin(last_mpc_data.pitch), 0, cos(last_mpc_data.pitch);

    // Поворот вокруг оси Z (Yaw)
    R_z << cos(last_mpc_data.yaw), -sin(last_mpc_data.yaw), 0,
           sin(last_mpc_data.yaw), cos(last_mpc_data.yaw), 0,
           0, 0, 1;

    // Общая матрица поворота
    R = R_z * R_y * R_x;

    log_msg.R_00 = R(0, 0);
    log_msg.R_01 = R(0, 1);
    log_msg.R_02 = R(0, 2);
    log_msg.R_10 = R(1, 0);
    log_msg.R_11 = R(1, 1);
    log_msg.R_12 = R(1, 2);
    log_msg.R_20 = R(2, 0);
    log_msg.R_21 = R(2, 1);
    log_msg.R_22 = R(2, 2);



    // Публикуем сообщение
    log_data_pub.publish(log_msg);
}


void ConvexMPCLocomotion::logDataCallback(const unitree_legged_msgs::LogData::ConstPtr& msg)
{
    last_log_data_received_ = *msg;
    received_log_data_ = true;
}
