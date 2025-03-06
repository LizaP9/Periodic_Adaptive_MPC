#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include <cmath>
#include <iostream>
#include <iterator>
#include <math.h>
#include <streambuf>

#include "CMPC_Locomotion_cv.h"
#include "ControlFSMData.h"
#include "GraphSearch.h"
#include "convexMPC_interface.h"

#include "Gait.h"

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

CMPCLocomotion_Cv::CMPCLocomotion_Cv(float _dt, int _iterations_between_mpc, ControlFSMData<float>* data)
  : _data(data),
    _parameters(_data->userParameters),
    _gait_period(_parameters->gait_period),
    _gait_period_long(32),
    trotting(_gait_period,
             Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
             Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0),
             "Trotting"),
    trot_long(_gait_period_long,
              Vec4<int>(0, _gait_period_long / 2.0, _gait_period_long / 2.0, 0),
              Vec4<int>(24, 24, 24, 24),
              "Trotting long"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    walking(_gait_period,
            Vec4<int>(2 * _gait_period / 4., 0, _gait_period / 4., 3 * _gait_period / 4.),
            Vec4<int>(0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period),
            "Walking"),
    iterationsBetweenMPC(_iterations_between_mpc),
    horizonLength(_data->staticParams->horizon),
    dt(_dt),
    _grid_map_raw(),
    _grid_map_filter(),
    _grid_map_plane(),
    _tf_buffer(),
    _tf_listener(_tf_buffer)
{
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  // setup_problem(dtMPC, horizonLength, 0.4, 150); // original
  setup_problem(dtMPC, horizonLength, 0.4, 150); // original
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

  // _initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  _model = _data->quadruped->buildModel();
  _state.q = DVec<float>::Zero(cheetah::num_act_joint);
  _state.qd = DVec<float>::Zero(cheetah::num_act_joint);

  _doorstep_case = true;
  _max_cell = 0;
  _gait = nullptr;
}

void CMPCLocomotion_Cv::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;

  iterationCounter = 0;
}

void CMPCLocomotion_Cv::_recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void CMPCLocomotion_Cv::_SetupCommand(float cmd_vel_x, float cmd_vel_y)
{
  _body_height = _parameters->body_height;
  float x_vel_cmd;
  float y_vel_cmd;
  float filter_x(0.005);
  float filter_y(0.005);

  _yaw_turn_rate = _data->gamepad_command->right_stick_analog[0] * _data->staticParams->max_turn_rate;
  x_vel_cmd = cmd_vel_x * _data->staticParams->max_vel_x;
  y_vel_cmd = cmd_vel_y * _data->staticParams->max_vel_y;

  _pitch_cmd = 0.4 * _data->gamepad_command->right_stick_analog[1];

  _x_vel_des = _x_vel_des * (1 - filter_x) + x_vel_cmd * filter_x;
  _y_vel_des = _y_vel_des * (1 - filter_y) + y_vel_cmd * filter_y;

  // _yaw_des = _data->stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  // _yaw_des += dt * _yaw_turn_rate;

  if ((M_PI - abs(_yaw_des)) <= 0.1)
  {
    _yaw_des = _data->stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  }
  else
  {
    _yaw_des += dt * _yaw_turn_rate;
  }

  if (current_gait == 13 || current_gait == 4 || current_gait == 11)
  {
    world_position_desired[0] = stand_traj[0] + 0.05 * cmd_vel_x;
    world_position_desired[1] = stand_traj[1] + 0.05 * cmd_vel_y;
    _body_height = stand_traj[2];

    _pitch_des = 0.2 * _data->gamepad_command->right_stick_analog[1];
    _yaw_des = stand_traj[5] + 0.2 * _data->gamepad_command->right_stick_analog[0];
  }

  _body_height = _parameters->body_height;

  // Update PD coefs
  Kp = Vec3<float>(_parameters->Kp_cartesian_0, _parameters->Kp_cartesian_1, _parameters->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_parameters->Kd_cartesian_0, _parameters->Kd_cartesian_1, _parameters->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

void CMPCLocomotion_Cv::run(ControlFSMData<float>& data)
{
  // myVersion(data, _grid_map_filter, _grid_map_raw, _grid_map_plane);
}

void CMPCLocomotion_Cv::myVersion(ControlFSMData<float>& data)
{
  bool omniMode = false;
  // long_step_vel = _data->userParameters->cmpc_use_sparse;
  // long_step_trigger = _data->userParameters->cmpc_use_sparse;

  // Command Setup
  // if (!long_step_vel)
  _SetupCommand(_data->gamepad_command->left_stick_analog[1], _data->gamepad_command->left_stick_analog[0]);
  // else
  // {
  //   std::vector<float> vel = calcDesVel();
  //   _SetupCommand(vel[0], vel[1]);
  // }
  _gait_des = data.userParameters->cmpc_gait;

  auto& seResult = data.stateEstimator->getResult();

  // Check if transition to standing
  if (((_gait_des == 4) && current_gait != 4) || firstRun)
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
  _gait = &trotting;
  current_gait = _gait_des;

  if (current_gait == 9)
  {
    _gait = &trotting;
  }
  else if (current_gait == 15)
  {
    _gait = &trot_long;
  }
  else if (current_gait == 4)
  {
    _gait = &standing;
  }

  // _gait->updatePeriod(_dyn_params->gait_period);
  _gait->restoreDefaults();
  _gait->setIterations(iterationsBetweenMPC, iterationCounter);
  _gait->earlyContactHandle(seResult.contactSensor, iterationsBetweenMPC, iterationCounter);

  _recompute_timing(default_iterations_between_mpc);

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
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (_gait_des != 8); // turn off for pronking

  static float z_des[4] = { 0 };

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else if (current_gait != 11)
  {
    // estimated pitch of plane and pitch correction depends on Vdes
    _pitch_des = _pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane;
    // _pitch_des = _pitch_cmd + data.stateEstimator->getResult().rpy[1] + data.stateEstimator->getResult().est_pitch_plane +
    // 0.1;

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

  if (_gait != &standing)
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
      // footSwingTrajectories[i].setHeight(_data->userParameters->Swing_traj_height);
      footSwingTrajectories[i].setHeight(_updateTrajHeight(i));

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      data.debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      data.debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      data.debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      // data.debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      // data.debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      // data.debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

      z_des[i] = pFoot[i](2);
    }

    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
  {
    swingTimes[l] = _gait->getCurrentSwingTime(dtMPC, l);
    _findPF(v_des_world, l);
  }

  // Calculate PF

  // calc gait
  iterationCounter++;

  // gait
  Vec4<float> contactStates = _gait->getContactState();
  Vec4<float> swingStates = _gait->getSwingState();
  int* mpcTable = _gait->getMpcTable();

  _updateMPCIfNeeded(mpcTable, data, omniMode);

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
      // Calculate P0
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

      // Update PF
      // if (!long_step_vel)
      //{
      // Vec3<float> pf = footSwingTrajectories[foot].getFinalPosition();
      // _updateFoothold(pf, seResult.position, foot);
      // footSwingTrajectories[foot].setFinalPosition(pf);
      // data.debug->all_legs_info.leg[foot].swing_pf.x = pf(0);
      // data.debug->all_legs_info.leg[foot].swing_pf.y = pf(1);
      // data.debug->all_legs_info.leg[foot].swing_pf.z = pf(2);
      //}

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

      _hipSefetyCircle(pDesLeg, pDesFootWorld, foot);

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
  data.stateEstimator->setSwingPhase(_gait->getSwingState());

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

  contact_state = _gait->getContactState();
  // END of WBC Update
}

void CMPCLocomotion_Cv::_updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
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
    _solveDenseMPC(mpcTable, data);

    // printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void CMPCLocomotion_Cv::_solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // Q roll pitch yaw x_des y_des z_des v_roll_des v_pitch_des v_yaw_des vx_des vy_des vz_des
  //  original
  //  float Q[12] = {0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1};

  // my
  //  float Q[12] = { 2.5, 2.5, 10, 300, 300, 1000, 0, 0, 0.5, 1.5, 1.5, 1 };
  //  float Q[12] = { 0.25, 4, 7, 4, 4, 20, 0.1, 0.1, 3, 0.4, 0.4, 0.2 };
  // float Q[12] = { 10, 10, 15, 15, 15, 20, 0.5, 0.5, 3, 0.4, 0.4, 0.2 }; //+- norm
  float Q[12] = { 10, 10, 15, 3, 3, 30, 0.5, 0.5, 3, 0.4, 0.4, 0.2 };

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

  // from sparse
  //  float Q[12] = { 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2 };

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = data.staticParams->alpha;
  // float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
  {
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];
  }

  // printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  setup_problem(dtMPC, horizonLength, 0.4, 150);
  update_x_drag(x_comp_integral);

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    // x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

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

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void CMPCLocomotion_Cv::_initSparseMPC()
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

void CMPCLocomotion_Cv::_updateModel(const StateEstimate<float>& state_est, const LegControllerData<float>* leg_data)
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

Eigen::Array2i checkBoundariess(const grid_map::GridMap& map, int col, int row)
{
  col = std::clamp(col, 0, map.getSize()(1));
  row = std::clamp(row, 0, map.getSize()(0));
  return Eigen::Array2i(col, row);
}

void CMPCLocomotion_Cv::_updateFoothold(Vec3<float>& pf, const Vec3<float>& body_pos_arg, const int& leg)
{
  // Положение лапы в СК тела
  //  Vec3<float> scale(1.2, 1, 1);
  static bool first = true;
  static Vec3<float> freeze_pose;
  Vec3<float> body_pos;
  Vec3<float> p0;
  if (firstSwing[leg])
  {
    p0 = pFoot[leg];
  }
  else
  {
    p0 = footSwingTrajectories[leg].getInitialPosition();
  }
  Vec3<float> local_pf;
  Vec3<float> local_p0;

  if (_data->debug->is_map_upd_stop)
  {
    if (first)
    {
      freeze_pose = body_pos_arg;
      first = false;
    }
  }
  // else
  // {
  body_pos = body_pos_arg;
  local_pf = pf - body_pos;
  local_p0 = p0 - body_pos;
  first = true;
  // }
  //  Vec3<float> local_pf_scaled = foot.cwiseProduct(scale) - body_pos;
  //  std::cout << "pf in base frame: " << std::endl << local_pf << std::endl;

  // Координаты цм робота на карте (обычно в центре)
  int row_idx_body_com = _grid_map_raw.getSize()(0) / 2;
  int col_idx_body_com = _grid_map_raw.getSize()(1) / 2;
  if (_data->debug->is_map_upd_stop)
  {
    col_idx_body_com -= ceil((body_pos_arg[0] - freeze_pose[0]) / _grid_map_raw.getResolution());
    row_idx_body_com += ceil((body_pos_arg[1] - freeze_pose[1]) / _grid_map_raw.getResolution());
    col_idx_body_com = std::clamp(col_idx_body_com, 0, _grid_map_raw.getSize()(1) - 1);
    row_idx_body_com = std::clamp(row_idx_body_com, 0, _grid_map_raw.getSize()(0) - 1);
    // TODO:Call enable update service when touch map edge
  }

  // Минус для преобразования координат
  int x_idx = col_idx_body_com - ceil(local_pf[0] / _grid_map_raw.getResolution());
  int y_idx = row_idx_body_com - ceil(local_pf[1] / _grid_map_raw.getResolution());
  x_idx = std::clamp(x_idx, 0, _grid_map_raw.getSize()(1) - 1);
  y_idx = std::clamp(y_idx, 0, _grid_map_raw.getSize()(0) - 1);

  int x_idx_selected = x_idx;
  int y_idx_selected = y_idx;

  int p0_x_idx = col_idx_body_com - ceil(local_p0[0] / _grid_map_raw.getResolution());
  int p0_y_idx = row_idx_body_com - ceil(local_p0[1] / _grid_map_raw.getResolution());

  auto p0_h = _grid_map_raw.at("elevation", checkBoundariess(_grid_map_raw, p0_x_idx, p0_y_idx));

  _idxMapChecking(local_pf, x_idx, y_idx, x_idx_selected, y_idx_selected, leg);

  // Минус для преобразования координат
  pf[0] = (col_idx_body_com - x_idx_selected) * _grid_map_raw.getResolution() + body_pos[0];
  pf[1] = (row_idx_body_com - y_idx_selected) * _grid_map_raw.getResolution() + body_pos[1];
  auto pf_h = _grid_map_raw.at("elevation", checkBoundariess(_grid_map_raw, x_idx_selected, y_idx_selected));
  if (leg == 3)
  {
    ROS_INFO_STREAM_THROTTLE(1, " Leg 3 height map = " << pf_h);
  }
  // check map on flat terrain
  // static std::vector<float> pf_buffer;
  // if (!std::isnan(pf_h) && pf_buffer.size() < 4000)
  // {
  //   pf_buffer.push_back(pf_h);
  // }
  // else
  // {
  //   double sum = 0.0;
  //   for (auto it : pf_buffer)
  //   {
  //     sum += it;
  //   }
  //   cout << "Mean pf = " << sum / pf_buffer.size() << endl;
  //   pf_buffer.clear();
  // }
  // _max_cell =
  //   _findMaxInMapByLine(height_map_filter, grid_map::Index(x_idx_selected, y_idx_selected), grid_map::Index(p0_x_idx,
  //   p0_y_idx));

  // cout << "max cell = " << _max_cell << endl;

  // ЕВРИСТИКА
  // static double start_offset = 0.6;
  // if (_data->stateEstimator->getResult().position(0) > start_offset)
  // {
  //   static double step_length = 0.8;
  //   double x = _data->stateEstimator->getResult().position(0) - start_offset;
  //   double k = x / step_length;
  //   if (k >= 1.0)
  //     k = 1.0;
  //   _data->debug->z_offset = k * 0.40;
  // }
  // _body_height_heuristics("vio");

  pf_h -= p0_h;
  pf[2] = (std::isnan(pf_h)) ? 0.0 : pf_h;
  if (pf[2] > MAX_STEP_HEIGHT)
  {
    pf[2] = MAX_STEP_HEIGHT;
    ROS_INFO_STREAM_THROTTLE(1, " Leg " << leg << " height limit = " << pf_h);
  }
}

void CMPCLocomotion_Cv::_body_height_heuristics(std::string type)
{
  if (type == "map")
  {
    grid_map::Index robot_com(_grid_map_plane.getSize()(0) / 2., _grid_map_plane.getSize()(1) / 2.);
    auto base_footprint_h = _grid_map_plane.at("smooth_planar", robot_com);
    _data->debug->z_offset = base_footprint_h;
  }
  else if (type == "vio")
  {
    geometry_msgs::TransformStamped odom2camera;
    static geometry_msgs::Transform camera2base;
    camera2base.translation.x = -0.055;
    camera2base.translation.y = 0.055;
    camera2base.translation.z = -0.13;
    camera2base.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14);
    try
    {
      odom2camera = _tf_buffer.lookupTransform("odom", "camera_t265_pose_frame", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    _data->debug->vio_z = odom2camera.transform.translation.z + camera2base.translation.z;
  }
}

void CMPCLocomotion_Cv::_idxMapChecking(Vec3<float>& pf,
                                        int x_idx,
                                        int y_idx,
                                        int& x_idx_selected,
                                        int& y_idx_selected,
                                        const int& leg)
{
  grid_map::Index center(x_idx, y_idx);
  // std::cout << " Leg position (x,y) " << pf[0] << " " << pf[1] << std::endl;
  double radius = 0.10;
  // std::cout << "Normal is " << _grid_map_raw.at("normal_z", Eigen::Array2i(x_idx, y_idx)) <<
  // std::endl;
  for (grid_map_utils::SpiralIterator iterator(_grid_map_raw, center, radius); !iterator.isPastEnd(); ++iterator)
  {
    // auto traversability = _grid_map_raw.at("traversability", *iterator);
    auto traversability = _grid_map_plane.at("plane_classification", *iterator);
    // auto uncertainty_r = height_map_filter.at("uncertainty_range", *iterator);
    // if (leg == 0)
    // std::cout << "traversability = " << traversability << std::endl;
    // If can step
    if (!std::isnan(traversability) && traversability > 0.8 /*&& !std::isnan(uncertainty_r) && uncertainty_r < 0.7*/)
    {
      x_idx_selected = (*iterator)(0);
      // if (!_doorstep_case)
      // y_idx_selected = (*iterator)(1);
      // if ((x_idx != x_idx_selected) && (y_idx != y_idx_selected))
      //   std::cout << "Edit footstep from ( " << x_idx << " " << y_idx << ") to ( " << x_idx_selected << " " << y_idx_selected
      //             << " )" << std::endl;
      return;
    }
  }
  //  std::cout << "Can`t find foothold from map!" << std::endl;
}

void CMPCLocomotion_Cv::_findPF(Vec3<float>& v_des_world, size_t foot)
{

  // auto swing_state = (_gait->getSwingState())[foot];

  // if (long_step_trigger && firstSwing[foot])
  // {
  //   long_step_run = true;
  // }

  // if (long_step_run)
  // {
  //   _longStep(_grid_map_filter, foot);
  //   if (swing_state <= 0) // stance
  //   {
  //     long_step_run = false;
  //     long_step_trigger = false;
  //   }
  //   return;
  // }

  // if (long_step_vel)
  // {
  //   _longStep(_grid_map_filter, foot);
  //   return;
  // }

  float side_sign[4] = { -1, 1, -1, 1 };
  float interleave_y[4] = { -0.08, 0.08, 0.02, -0.02 };
  float interleave_gain = -0.2;
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  float v_abs = std::fabs(v_des_robot[0]);
  auto seResult = _data->stateEstimator->getResult();

  if (firstSwing[foot])
  {
    swingTimeRemaining[foot] = swingTimes[foot];
  }
  else
  {
    swingTimeRemaining[foot] -= dt;
  }

  // footSwingTrajectories[foot].setHeight(_data->userParameters->Swing_traj_height);
  footSwingTrajectories[foot].setHeight(_updateTrajHeight(foot));

  Vec3<float> offset(0, side_sign[foot] * _data->quadruped->_abadLinkLength, 0);

  Vec3<float> pRobotFrame = (_data->quadruped->getHipLocation(foot) + offset);

  pRobotFrame[1] += interleave_y[foot] * v_abs * interleave_gain;
  float stance_time = _gait->getCurrentStanceTime(dtMPC, foot);

  Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

  Vec3<float> des_vel;
  des_vel[0] = _x_vel_des;
  des_vel[1] = _y_vel_des;
  des_vel[2] = 0.0;

  Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[foot]);

  float p_rel_max = 0.3f;

  float pfx_rel = seResult.vWorld[0] * stance_time / 2.0 + 0.03f * (v_des_world[0] - seResult.vWorld[0]) +
                  0.5f * seResult.position[2] / 9.81f * seResult.vWorld[1] * _yaw_turn_rate;
  float pfy_rel = seResult.vWorld[1] * stance_time / 2.0 + 0.03f * (v_des_world[1] - seResult.vWorld[1]) -
                  0.5f * seResult.position[2] / 9.81f * seResult.vWorld[0] * _yaw_turn_rate;

  pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
  pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
  Pf[0] += pfx_rel;
  Pf[1] += pfy_rel;
  Pf[2] = 0.0;

  _updateFoothold(Pf, seResult.position, foot);
  footSwingTrajectories[foot].setFinalPosition(Pf);
  // _data->debug->all_legs_info.leg[foot].swing_pf.x = Pf(0);
  // _data->debug->all_legs_info.leg[foot].swing_pf.y = Pf(1);
  // _data->debug->all_legs_info.leg[foot].swing_pf.z = Pf(2);
}

float CMPCLocomotion_Cv::_updateTrajHeight(size_t foot)
{
  double h = 0;
  double k = 0.05;
  double out;
  // cout << "MAXELL = " << _max_cell << endl;
  // double obst_h = _max_cell - footSwingTrajectories[foot].getInitialPosition()(2);
  double obst_h = std::abs(footSwingTrajectories[foot].getFinalPosition()(2));
  // double obst_h =
  //   std::abs(footSwingTrajectories[foot].getFinalPosition()(2) - footSwingTrajectories[foot].getInitialPosition()(2));
  h = obst_h + k;
  // Saturate h
  h = std::clamp(h, -_data->userParameters->Swing_traj_height, _data->userParameters->Swing_traj_height);
  // if (obst_h < 0.07)
  //   out = _data->userParameters->Swing_traj_height;
  // else
  //   out = h;
  // out = std::clamp(out, -MAX_STEP_HEIGHT, MAX_STEP_HEIGHT);
  return h;
}

double CMPCLocomotion_Cv::_findMaxInMapByLine(const grid_map::GridMap& map,
                                              grid_map::Index start,
                                              grid_map::Index end,
                                              const grid_map::Index* cell_idx)
{
  double max = 0.0;
  bool first_time = true;
  for (grid_map::LineIterator iterator(map, start, end); !iterator.isPastEnd(); ++iterator)
  {
    double cell = map.at("elevation", *iterator);
    if ((cell >= max) && !std::isnan(cell))
      max = cell;
    if (cell_idx != nullptr && cell > 0.05 && first_time)
    {
      first_time = false;
      cell_idx = &(*iterator);
    }
  }
  return max;
}

void CMPCLocomotion_Cv::_longStep(const grid_map::GridMap& map, int foot)
{
  Vec3<float> p0 = footSwingTrajectories[foot].getInitialPosition();
  Vec3<float> body_pos = _data->stateEstimator->getResult().position;
  Vec3<float> local_p0 = p0 - body_pos;

  int row_idx_body_com = map.getSize()(0) / 2;
  int col_idx_body_com = map.getSize()(1) / 2;
  int p0_x_idx = col_idx_body_com - ceil(local_p0[0] / map.getResolution());
  int p0_y_idx = row_idx_body_com - ceil(local_p0[1] / map.getResolution());
  grid_map::Index p0_index(p0_x_idx, p0_y_idx);

  grid_map::Index cell_search_coord(p0_index);
  cell_search_coord.x() += 10; // 20cm
  grid_map::Index obst_map_idx;
  // double h = _findMaxInMapByLine(map, p0_index, cell_search_coord, &obst_map_idx);
  double h = 0.1;
  cout << "[DEBUG]: Obstacle height = " << h << endl;

  grid_map::Index d(10, 0);
  // grid_map::Index d = obst_map_idx - p0_index;
  Eigen::Array2i dist = d * Eigen::Array2i(map.getResolution());
  footSwingTrajectories[foot].setHeight(h + 0.03);
  Vec3<float> pf = body_pos + local_p0 + Vec3<float>(-2 * dist.x(), -2 * dist.y(), 0);
  footSwingTrajectories[foot].setFinalPosition(pf);
}

std::vector<float> CMPCLocomotion_Cv::calcDesVel()
{
  std::vector<float> out = { 0, 0 };
  if (_gait == nullptr)
    return out;

  static std::vector<bool> progress(4, false);
  static std::vector<bool> swing_start(4, false);
  static std::vector<bool> stance_start(4, false);

  auto swing_state = _gait->getSwingState();
  for (size_t leg = 0; leg < 4; leg++)
  {
    if (firstSwing[leg])
      swing_start[leg] = true;
    if (swing_state[leg] <= 0 && swing_start[leg] == true) // stance
      stance_start[leg] = true;

    if (swing_start[leg] && stance_start[leg])
      progress[leg] = true;
  }

  size_t cnt = 0;
  for (auto it : progress)
  {
    if (it != false)
      cnt++;
  }
  if (cnt == 4)
  {
    long_step_vel = false;
    _data->userParameters->cmpc_use_sparse = false;
    progress = std::vector<bool>(4, false);
    swing_start = std::vector<bool>(4, false);
    stance_start = std::vector<bool>(4, false);
  }

  out[0] = 1.0;
  return out;
}

void CMPCLocomotion_Cv::_hipSefetyCircle(Vec3<float>& pDesLeg, Vec3<float>& pDesFootWorld, const int& foot)
{
  if (((pDesLeg[0] * pDesLeg[0]) + (pDesLeg[2] * pDesLeg[2])) / (_data->staticParams->r_circle * _data->staticParams->r_circle) <
      1)
  {
    pDesLeg[2] = -std::sqrt((_data->staticParams->r_circle * _data->staticParams->r_circle) - (pDesLeg[0] * pDesLeg[0]));
    pDesFootWorld = _data->stateEstimator->getResult().rBody.transpose() * (pDesLeg + _data->quadruped->getHipLocation(foot)) +
                    _data->stateEstimator->getResult().position;
    ROS_INFO_THROTTLE(1, "Cut foot swing trajectory by hip-centered circle");
  }
}
