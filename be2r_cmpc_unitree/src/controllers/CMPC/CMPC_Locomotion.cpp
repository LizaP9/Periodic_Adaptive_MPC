#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <math.h>
#include <streambuf>

#include "CMPC_Locomotion.h"
#include "ControlFSMData.h"
#include "GraphSearch.h"
#include "convexMPC_interface.h"

#include "Gait.h"

// оригинальные параметры MPC+WBC
//  #define GAIT_PERIOD 14

#define GAIT_PERIOD 14
// #define GAIT_PERIOD 22
// #define GAIT_PERIOD 34 //1000 Hz

// #define GAIT_PERIOD_WALKING 26
#define GAIT_PERIOD_WALKING 32

// лучшие параметры для только MPC
//  #define GAIT_PERIOD 18

#define STEP_HEIGHT 0.06
#define BODY_HEIGHT 0.24

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

CMPCLocomotion::CMPCLocomotion(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data)
  : _data(data),
    iterationsBetweenMPC(_iterations_between_mpc),
    _parameters(_data->userParameters),
    _gait_period(_parameters->gait_period),
    _gait_period_long(32),
    horizonLength(_data->staticParams->horizon),
    dt(_dt),
    trotting(_gait_period,
             Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
             Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0),
             "Trotting"),
    trot_long(_gait_period_long,
              Vec4<int>(0, _gait_period_long / 2.0, _gait_period_long / 2.0, 0),
              Vec4<int>(24, 24, 24, 24),
              "Trotting long"),
    trot_contact(_gait_period,
                 Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
                 Vec4<int>(_gait_period * 0.25, _gait_period * 0.25, _gait_period * 0.25, _gait_period * 0.25),
                 "Trot contact"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    give_hand(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "GiveHand"),
    walking(
      GAIT_PERIOD_WALKING,
      Vec4<int>(2 * GAIT_PERIOD_WALKING / 4., 0, GAIT_PERIOD_WALKING / 4., 3 * GAIT_PERIOD_WALKING / 4.),
      Vec4<int>(0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING, 0.75 * GAIT_PERIOD_WALKING),
      "Walking"), // for real
    two_leg_balance(_gait_period,
                    Vec4<int>(0, 0, 0, 0),
                    Vec4<int>(_gait_period, _gait_period, _gait_period, 0),
                    "Two legs balance")
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  // setup_problem(dtMPC, horizonLength, 0.4, 150); // original
  setup_problem(dtMPC, horizonLength, 0.4, 300); // original

  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  _model = _data->quadruped->buildModel();
  _state.q = DVec<float>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<float>::Zero(cheetah::num_act_joint);
}

void CMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;

  iterationCounter = 0;
}

void CMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void CMPCLocomotion::_SetupCommand(ControlFSMData<float>& data)
{
  float x_vel_cmd, y_vel_cmd;
  float filter_x(0.01);
  float filter_y(0.01);

  _yaw_turn_rate = data.gamepad_command->right_stick_analog[0];
  x_vel_cmd = data.gamepad_command->left_stick_analog[1];
  y_vel_cmd = data.gamepad_command->left_stick_analog[0];

  _yaw_turn_rate *= data.staticParams->max_turn_rate;
  x_vel_cmd *= data.staticParams->max_vel_x;
  y_vel_cmd *= data.staticParams->max_vel_y;

  pitch_cmd = 0.4 * data.gamepad_command->right_stick_analog[1];

  _x_vel_des = _x_vel_des * (1 - filter_x) + x_vel_cmd * filter_x;
  _y_vel_des = _y_vel_des * (1 - filter_y) + y_vel_cmd * filter_y;

  if ((M_PI - abs(_yaw_des)) <= 0.1)
  {
    _yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  }
  else
  {
    _yaw_des += dt * _yaw_turn_rate;
  }

  if (current_gait == 13 || current_gait == 4 || current_gait == 11)
  {
    world_position_desired[0] = stand_traj[0] + 0.05 * data.gamepad_command->left_stick_analog[1];
    world_position_desired[1] = stand_traj[1] + 0.05 * data.gamepad_command->left_stick_analog[0];
    _body_height = stand_traj[2];

    _pitch_des = 0.2 * data.gamepad_command->right_stick_analog[1];
    _yaw_des = stand_traj[5] + 0.2 * data.gamepad_command->right_stick_analog[0];
  }

  _body_height = _parameters->body_height;
  _swing_trajectory_height = _parameters->Swing_traj_height;

  if (data.gamepad_command->triangle && (gaitNumber == 15))
  {
    data.gamepad_command->stairs_mode = StairsMode::UP;
    _body_height = 0.29;
    _swing_trajectory_height = 0.17;
  }

  if (data.gamepad_command->cross && (gaitNumber == 15))
  {
    data.gamepad_command->stairs_mode = StairsMode::DOWN;
    _body_height = 0.20;
    _swing_trajectory_height = 0.06;
  }

  // Update PD coefs
  Kp = Vec3<float>(_parameters->Kp_cartesian_0, _parameters->Kp_cartesian_1, _parameters->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_parameters->Kd_cartesian_0, _parameters->Kd_cartesian_1, _parameters->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

void CMPCLocomotion::run(ControlFSMData<float>& data)
{
  // myVersion(data);
  myNewVersion(data);
}

void CMPCLocomotion::myVersion(ControlFSMData<float>& data)
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
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = seResult.rpy[0];
    stand_traj[4] = seResult.rpy[1];
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // Check if transition to two legs standing
  if (((gaitNumber == 13) && current_gait != 13) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = seResult.rpy[0];
    stand_traj[4] = seResult.rpy[1];
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // Check if transition to give hand
  if (((gaitNumber == 11) && current_gait != 11) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = seResult.position[2];
    // stand_traj[2] = 0.15;
    stand_traj[3] = seResult.rpy[0];
    stand_traj[4] = seResult.rpy[1];
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait_contact* gait = &trotting;
  current_gait = gaitNumber;

  if (current_gait == 13)
  {
    gait = &two_leg_balance;
    // gait = &trot_contact;
  }
  else if (current_gait == 4)
  {
    gait = &standing;
  }
  else if (current_gait == 9)
  {
    gait = &trotting;
  }
  else if (current_gait == 10)
  {
    gait = &walking;
  }
  else if (current_gait == 11)
  {
    gait = &give_hand;
  }
  else if (current_gait == 15)
  {
    gait = &trot_long;
  }

  // gait->updatePeriod(_parameters->gait_period);
  // gait->restoreDefaults();
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  // gait->earlyContactHandle(data.stateEstimator->getContactSensorData(), iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world(0, 0, 0);

  if (gait != &give_hand)
  {
    v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  }

  Vec3<float> v_robot = seResult.vWorld;
  static float z_des[4] = { 0 };

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else if (current_gait != 11)
  {
    // estimated pitch of plane and 0.07 rad pitch correction on 1 m/s Vdes
    _pitch_des =
      pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane - 0.07 * _x_vel_des;
    // _pitch_des = data.stateEstimator->getResult().est_pitch_plane;
  }

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] =
      seResult.position + seResult.rBody.transpose() * (data.quadruped->getHipLocation(i) + data.legController->datas[i].p);
  }

  if ((gait != &standing) || (gait != &give_hand))
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_swing_trajectory_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

      z_des[i] = pFoot[i](2);
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

  // gait
  // trot leg 0 starts in stance because offset is 0
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  for (size_t leg_num = 0; leg_num < 4; leg_num++)
  {
    data.debug->all_legs_info.leg[leg_num].stance_time = contactStates[leg_num];
    data.debug->all_legs_info.leg[leg_num].swing_time = swingStates[leg_num];
    data.debug->all_legs_info.leg[leg_num].phase = gait->getCurrentGaitPhase();
    data.debug->all_legs_info.leg[leg_num].is_contact = data.stateEstimator->getContactSensorData()(leg_num);
  }

  updateMPCIfNeeded(mpcTable, data, omniMode);
  _updateModel(data.stateEstimator->getResult(), data.legController->datas);

  Vec4<float> se_contactState(0, 0, 0, 0);
  se_contactState = data.stateEstimator->getContactSensorData().cast<float>();

  static bool is_stance[4] = { 0, 0, 0, 0 };
  static Vec3<float> p_fw[4] = {};
  static Vec3<float> p_fl[4] = {};
  static float delta_yaw[4] = {};
  static Vec3<float> delta_p_bw[4] = {};
  static uint32_t stand_iterator[4] = { 0, 0, 0, 0 };
  static float z_stand_avr[4] = { 0, 0, 0, 0 };

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    if ((is_stance[foot] == 0) && !(swingState > 0))
    {
      is_stance[foot] = 1;

      // foot position in world frame at contact
      data.debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];

      p_fl[foot] = data.legController->datas[foot].p + data.quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    float Kf = 1;
    delta_p_bw[foot] += seResult.vBody * dt * Kf;
    delta_yaw[foot] += seResult.omegaBody(2) * dt * Kf;
    data.debug->last_p_local_stance[foot] =
      ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        data.debug->all_legs_info.leg[foot].swing_ps.x = pFoot[foot](0);
        data.debug->all_legs_info.leg[foot].swing_ps.y = pFoot[foot](1);
        data.debug->all_legs_info.leg[foot].swing_ps.z = pFoot[foot](2);

        z_des[foot] = pFoot[foot][2];
        swingTimeRemaining[foot] = swingTimes[foot];
      }
      else
      {
        swingTimeRemaining[foot] -= dt;
      }

      footSwingTrajectories[foot].setHeight(_swing_trajectory_height);

      //
      Vec3<float> offset(0, side_sign[foot] * data.quadruped->_abadLinkLength, 0);

      Vec3<float> pRobotFrame = data.quadruped->getHipLocation(foot) + offset;

      pRobotFrame[1] += interleave_y[foot] * v_abs * interleave_gain;
      float stance_time = gait->getCurrentStanceTime(dtMPC, foot);

      Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

      Vec3<float> des_vel;
      des_vel[0] = _x_vel_des;
      des_vel[1] = _y_vel_des;
      des_vel[2] = 0.0;

      Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[foot]);

      float p_rel_max = 0.3f;

      // Using the estimated velocity is correct
      float pfx_rel = seResult.vWorld[0] * stance_time / 2.0 + 0.03f * (v_des_world[0] - seResult.vWorld[0]) +
                      0.5f * seResult.position[2] / 9.81f * seResult.vWorld[1] * _yaw_turn_rate;
      float pfy_rel = seResult.vWorld[1] * stance_time / 2.0 + 0.03f * (v_des_world[1] - seResult.vWorld[1]) -
                      0.5f * seResult.position[2] / 9.81f * seResult.vWorld[0] * _yaw_turn_rate;

      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] += pfx_rel;
      Pf[1] += pfy_rel;
      Pf[2] = z_des[foot];

      footSwingTrajectories[foot].setFinalPosition(Pf);
      data.debug->all_legs_info.leg[foot].swing_pf.x = Pf(0);
      data.debug->all_legs_info.leg[foot].swing_pf.y = Pf(1);
      data.debug->all_legs_info.leg[foot].swing_pf.z = Pf(2);
      //--------------------------------------------------------------

      // for visual
      geometry_msgs::PoseStamped pose_traj;
      Vec3<float> p_des_traj(0, 0, 0);
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();

      // 20 segment trajcetory
      for (size_t i = 0; i < 21; i++)
      {
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + ((1.0 - swingState) / 21.0 * (float)i),
                                                                 swingTimes[foot]);
        p_des_traj = footSwingTrajectories[foot].getPosition();

        pose_traj.pose.position = ros::toMsg(p_des_traj);

        data.debug->leg_traj_des[foot].poses.push_back(pose_traj);
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<float> pActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].p +
      // data.quadruped->getHipLocation(foot)) + seResult.position;
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

      Vec3<float> tau;
      tau = _grav.tail(12).block<3, 1>(foot * 3, 0);

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
        data.legController->commands[foot].tauFeedForward = tau;
      }
    }
    else // foot is in stance
    {
      stand_iterator[foot]++;

      firstSwing[foot] = true;

      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();

      geometry_msgs::Point point;
      point = ros::toMsg(data.legController->datas[foot].p + data.quadruped->getHipLocation(foot));
      data.debug->last_p_local_stance[foot] = point;

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(1.0, swingTimes[foot]);
      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld(0, 0, 0);
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data.quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (data.legController->datas[foot].v) + seResult.vWorld;

      if (!data.userParameters->use_wbc) // wbc off
      {
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp_stance;
        data.legController->commands[foot].kdCartesian = Kd_stance;

        data.legController->commands[foot].forceFeedForward = f_ff[foot];
        data.legController->commands[foot].kdJoint =
          Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2).asDiagonal();
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
  }

  data.stateEstimator->setContactPhase(se_contactState);
  data.stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.0;
  pBody_RPY_des[1] = _pitch_des;
  // pBody_RPY_des[1] = 0.0;
  // pBody_RPY_des[2] = 0.0;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.0;
  vBody_Ori_des[1] = 0.0;
  vBody_Ori_des[2] = _yaw_turn_rate;

  data.debug->body_info.pos_des = ros::toMsg(pBody_des);

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

  // calc gait
  iterationCounter++;
}

void CMPCLocomotion::myNewVersion(ControlFSMData<float>& data)
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
    stand_traj[2] = seResult.position[2];
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait_contact* gait = &trotting;
  current_gait = gaitNumber;

  if (current_gait == 9)
  {
    gait = &trotting;
  }
  else if (current_gait == 15)
  {
    gait = &trot_long;
  }
  else if (current_gait == 4)
  {
    gait = &standing;
  }

  // gait->updatePeriod(_dyn_params->gait_period);
  gait->restoreDefaults();
  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  gait->earlyContactHandle(seResult.contactSensor, iterationsBetweenMPC, iterationCounter);

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

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

  static float z_des[4] = { 0 };

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else if (current_gait != 11)
  {
    // estimated pitch of plane and pitch correction depends on Vdes
    // _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane + 0.1;
    _pitch_des = pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane;

    if (_x_vel_des > 0)
    {
      _pitch_des += -0.3 * _x_vel_des / data.staticParams->max_vel_x;
    }
    else
    {
      _pitch_des += -0.2 * _x_vel_des / data.staticParams->max_vel_x;
    }
  }

  for (int i = 0; i < 4; i++)
  {
    pFoot[i] =
      seResult.position + seResult.rBody.transpose() * (data.quadruped->getHipLocation(i) + data.legController->datas[i].p);
  }

  if (gait != &standing)
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_swing_trajectory_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

      z_des[i] = pFoot[i](2);
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

  // calculate swing pf
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

    footSwingTrajectories[i].setHeight(_swing_trajectory_height);

    Vec3<float> offset(0, side_sign[i] * (data.quadruped->_abadLinkLength), 0);

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

    float pfx_rel = seResult.vWorld[0] * stance_time / 2.0 + 0.03f * (v_des_world[0] - seResult.vWorld[0]) +
                    0.5f * seResult.position[2] / 9.81f * seResult.vWorld[1] * _yaw_turn_rate;
    float pfy_rel = seResult.vWorld[1] * stance_time / 2.0 + 0.03f * (v_des_world[1] - seResult.vWorld[1]) -
                    0.5f * seResult.position[2] / 9.81f * seResult.vWorld[0] * _yaw_turn_rate;

    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    Pf[2] = z_des[i];

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

      data.debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];

      p_fl[foot] = data.legController->datas[foot].p + data.quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    delta_p_bw[foot] += seResult.vBody * dt;
    delta_yaw[foot] += seResult.omegaBody(2) * dt;
    data.debug->last_p_local_stance[foot] =
      ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        data.debug->all_legs_info.leg[foot].swing_ps.x = pFoot[foot](0);
        data.debug->all_legs_info.leg[foot].swing_ps.y = pFoot[foot](1);
        data.debug->all_legs_info.leg[foot].swing_ps.z = pFoot[foot](2);

        z_des[foot] = pFoot[foot][2];
      }

      // for visual -----------------------------------------------------------------------
      geometry_msgs::PoseStamped pose_traj;
      Vec3<float> p_des_traj(0, 0, 0);
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();

      // 20 segment trajcetory
      for (size_t i = 0; i < 21; i++)
      {
        footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + ((1.0 - swingState) / 21.0 * (float)i),
                                                                 swingTimes[foot]);
        p_des_traj = footSwingTrajectories[foot].getPosition();

        pose_traj.pose.position = ros::toMsg(p_des_traj);

        data.debug->leg_traj_des[foot].poses.push_back(pose_traj);
      }
      // for visual -----------------------------------------------------------------------

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
        data.legController->commands[foot].pDes = pDesLeg;
        data.legController->commands[foot].vDes = vDesLeg;
        data.legController->commands[foot].kpCartesian = Kp;
        data.legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      // for visual ---------------------------------------------------------
      data.debug->leg_traj_des[foot].poses.clear();
      data.debug->leg_traj_des[foot].header.stamp = ros::Time::now();
      data.debug->all_legs_info.leg[foot].swing_pf = ros::toMsg(pFoot[foot]);
      // for visual ---------------------------------------------------------

      geometry_msgs::Point point;
      point = ros::toMsg(data.legController->datas[foot].p + data.quadruped->getHipLocation(foot));
      data.debug->last_p_local_stance[foot] = point;

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
        data.legController->commands[foot].kdJoint =
          Vec3<float>(_parameters->Kd_joint_0, _parameters->Kd_joint_1, _parameters->Kd_joint_2).asDiagonal();
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
  }

  data.stateEstimator->setContactPhase(se_contactState);
  data.stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  // pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[0] = rpy_comp[0];
  // pBody_RPY_des[1] = rpy_comp[1];
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

void CMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
{
  if ((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data.stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;

    const float max_pos_error = .1;
    float xStart = world_position_desired[0];
    float yStart = world_position_desired[1];

    if (xStart - p[0] > max_pos_error)
    {
      xStart = p[0] + max_pos_error;
    }
    if (p[0] - xStart > max_pos_error)
    {
      xStart = p[0] - max_pos_error;
    }

    if (yStart - p[1] > max_pos_error)
    {
      yStart = p[1] + max_pos_error;
    }
    if (p[1] - yStart > max_pos_error)
    {
      yStart = p[1] - max_pos_error;
    }

    world_position_desired[0] = xStart;
    world_position_desired[1] = yStart;

    float trajInitial[12] = { pBody_RPY_des[0], // 0 roll des
                              pBody_RPY_des[1], // 1 pitch des
                              pBody_RPY_des[2], // 2 yaw des
                              pBody_des[0],     // 3 x body des
                              pBody_des[1],     // 4 y body des
                              pBody_des[2],     // 5 z body des
                              vBody_Ori_des[0], // 6 velocity roll des
                              vBody_Ori_des[1], // 7 velocity pitch des
                              vBody_Ori_des[2], // 8 velocity yaw des
                              vBody_des[0],     // 9 vx body des
                              vBody_des[1],     // 10 vy body des
                              vBody_des[2] };   // 11 vz body des

    // i - horizon step, j - traj element
    for (int i = 0; i < horizonLength; i++)
    {
      for (int j = 0; j < 12; j++)
      {
        trajAll[12 * i + j] = trajInitial[j];
      }

      if (i > 0)
      {
        trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
        trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
      }
    }

    Timer solveTimer;

    // dense matrix - contain mostly NON zero values
    solveDenseMPC(mpcTable, data);
  }
}

void CMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // Q roll pitch yaw x_des y_des z_des v_roll_des v_pitch_des v_yaw_des vx_des vy_des vz_des
  //  original
  float Q[12] = { 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1 };

  float Q_roll = data.staticParams->Q_roll;
  float Q_pitch = data.staticParams->Q_pitch;
  float Q_yaw = data.staticParams->Q_yaw;
  float Q_x = data.staticParams->Q_x;
  float Q_y = data.staticParams->Q_y;
  float Q_z = data.staticParams->Q_z;
  float Q_w_roll = data.staticParams->Q_w_roll;
  float Q_w_pitch = data.staticParams->Q_w_pitch;
  float Q_w_yaw = data.staticParams->Q_w_yaw;
  float Q_vx = data.staticParams->Q_vx;
  float Q_vy = data.staticParams->Q_vy;
  float Q_vz = data.staticParams->Q_vz;

  Q[0] = Q_roll;
  Q[1] = Q_pitch;
  Q[2] = Q_yaw;
  Q[3] = Q_x;
  Q[4] = Q_y;
  Q[5] = Q_z;
  Q[6] = Q_w_roll;
  Q[7] = Q_w_pitch;
  Q[8] = Q_w_yaw;
  Q[9] = Q_vx;
  Q[10] = Q_vy;
  Q[11] = Q_vz;

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  // float alpha = 4e-5; // make setting eventually
  float alpha = data.staticParams->alpha;
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

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

  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 120);
  update_x_drag(x_comp_integral);

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho, _parameters->jcqp_sigma, _parameters->jcqp_alpha,
                         _parameters->jcqp_terminate, _parameters->use_jcqp);
  // t1.stopPrint("Setup MPC");
  // printf("MPC Setup time %f ms\n", t1.getMs());

  Timer t2;
  // cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, roll, pitch, yaw, weights, trajAll, alpha, mpcTable);
  // t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;

    for (int axis = 0; axis < 3; axis++)
    {
      f[axis] = get_solution(leg * 3 + axis);
    }

    f_ff[leg] = -seResult.rBody * f;

    // Update for WBC
    Fr_des[leg] = f;
  }
}

// void CMPCLocomotion::solveSparseMPC(int* mpcTable, ControlFSMData<float>& data)
// {
//   // X0, contact trajectory, state trajectory, feet, get result!
//   (void)mpcTable;
//   (void)data;
//   auto seResult = data.stateEstimator->getResult();

//   std::vector<ContactState> contactStates;
//   for (int i = 0; i < horizonLength; i++)
//   {
//     contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
//   }

//   for (int i = 0; i < horizonLength; i++)
//   {
//     for (u32 j = 0; j < 12; j++)
//     {
//       _sparseTrajectory[i][j] = trajAll[i * 12 + j];
//     }
//   }

//   Vec12<float> feet;
//   for (u32 foot = 0; foot < 4; foot++)
//   {
//     for (u32 axis = 0; axis < 3; axis++)
//     {
//       feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
//     }
//   }

//   _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
//   _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
//   _sparseCMPC.setStateTrajectory(_sparseTrajectory);
//   _sparseCMPC.setFeet(feet);
//   _sparseCMPC.run();

//   Vec12<float> resultForce = _sparseCMPC.getResult();

//   for (u32 foot = 0; foot < 4; foot++)
//   {
//     Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
//     // printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
//     f_ff[foot] = -seResult.rBody * force;
//     Fr_des[foot] = force;
//   }
// }

void CMPCLocomotion::initSparseMPC()
{
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0, 0, 0.26, 0, 0, 0, 0.242;
  double mass = 9;
  double maxForce = 150;

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

void CMPCLocomotion::_updateModel(const StateEstimate<float>& state_est, const LegControllerData<float>* leg_data)
{
  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  

  for (size_t i(0); i < 3; ++i)
  {
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i + 3] = state_est.vBody[i];

    for (size_t leg(0); leg < 4; ++leg)
    {
      _state.q[3 * leg + i] = leg_data[leg].q[i];
      _state.qd[3 * leg + i] = leg_data[leg].qd[i];
    }
  }

  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  _coriolis = _model.getCoriolisForce();
  _Ainv = _A.inverse();

  // cout << "Mass Matrix: " << _A << endl;
  // cout << "C+G: " << _grav+_coriolis << endl;
  // cout << "Gravity: " << _grav << endl;
}
