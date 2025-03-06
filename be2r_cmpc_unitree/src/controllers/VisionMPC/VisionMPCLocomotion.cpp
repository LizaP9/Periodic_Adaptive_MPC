#include "VisionMPCLocomotion.h"

// #define SHOW_MPC_SOLVE_TIME

using namespace std;

////////////////////
// Controller
////////////////////

VisionMPCLocomotion::VisionMPCLocomotion(float _dt, int iterations_between_mpc, ControlFSMData<float>* data)
  : _data(data),
    _floor_plane_height(0.),
    _iterationsBetweenMPC(iterations_between_mpc),
    _dyn_params(_data->userParameters),
    _gait_period(_dyn_params->gait_period),
    horizonLength(_data->staticParams->horizon),
    dt(_dt),
    trotting(_gait_period,
             Vec4<int>(0, _gait_period / 2.0, _gait_period / 2.0, 0),
             Vec4<int>(_gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0, _gait_period / 2.0),
             "Trotting"),
    standing(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, _gait_period, _gait_period, _gait_period), "Standing"),
    walking(_gait_period,
            Vec4<int>(2 * _gait_period / 4., 0, _gait_period / 4., 3 * _gait_period / 4.),
            Vec4<int>(0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period, 0.75 * _gait_period),
            "Walking"), // for real
    two_leg_balance(_gait_period, Vec4<int>(0, 0, 0, 0), Vec4<int>(_gait_period, 0, _gait_period, 0), "Two legs balance"),
    _nh()
{
  dtMPC = dt * _iterationsBetweenMPC;
  default_iterations_between_mpc = _iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, _iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 150); // original
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;
  // _timer = _nh.createTimer(ros::Rate(2), &VisionMPCLocomotion::_locHeightClearance, this);

  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void VisionMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
  {
    firstSwing[i] = true;
  }

  firstRun = true;
}

void VisionMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  _iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void VisionMPCLocomotion::_setupCommand(ControlFSMData<float>& data)
{
  _data = &data;
  _body_height = _dyn_params->body_height;

  float x_vel_cmd, y_vel_cmd;
  float filter(0.1);

  // Original
  x_vel_cmd = data.gamepad_command->left_stick_analog[1];
  y_vel_cmd = data.gamepad_command->left_stick_analog[0];
  _yaw_turn_rate = data.gamepad_command->right_stick_analog[0];

  // auto curr_pos = (this->_data->stateEstimator->getResult()).position;
  // std::cout << "cur_pose = " << curr_pos(0) << std::endl;
  // if (curr_pos(0) <= 1.5)
  // {
  //   x_vel_cmd = 0.2;
  // }
  // else
  // {
  //   x_vel_cmd = 0.0;
  // }

  _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
  _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

  // _yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  if ((M_PI - abs(_yaw_des)) <= 0.1)
  {
    _yaw_des = data.stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
  }
  else
  {
    _yaw_des += dt * _yaw_turn_rate;
  }
  _roll_des = 0.;
  _pitch_des = 0.;

  // Update PD coefs
  Kp = Vec3<float>(_dyn_params->Kp_cartesian_0, _dyn_params->Kp_cartesian_1, _dyn_params->Kp_cartesian_2).asDiagonal();
  Kp_stance = Kp;

  Kd = Vec3<float>(_dyn_params->Kd_cartesian_0, _dyn_params->Kd_cartesian_1, _dyn_params->Kd_cartesian_2).asDiagonal();
  Kd_stance = Kd;
}

void VisionMPCLocomotion::run(const Vec3<float>& vel_cmd_world,
                              const grid_map::GridMap& height_map_filter,
                              const grid_map::GridMap& height_map_raw,
                              const grid_map::GridMap& map_plane)
{
  bool omniMode = false;

  // Command Setup
  _setupCommand(*_data);

  gaitNumber = _data->userParameters->cmpc_gait;

  auto& seResult = _data->stateEstimator->getResult();
  Vec3<float> v_robot = seResult.vWorld;

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

  // pick gait
  Gait* gait = &trotting;
  current_gait = gaitNumber;

  if (current_gait == 4)
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

  gait->updatePeriod(_dyn_params->gait_period);
  gait->restoreDefaults();
  gait->setIterations(_iterationsBetweenMPC, _iterationCounter);
  // gait->earlyContactHandle(seResult.contactSensor, _iterationsBetweenMPC, _iterationCounter);
  gait->earlyContactHandle(_data->stateEstimator->getContactSensorData(), _iterationsBetweenMPC, _iterationCounter);
  //  std::cout << "_iterationCounter " << _iterationCounter << std::endl;

  recompute_timing(default_iterations_between_mpc);

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;

  // std::cout << "sensor data: " << data.stateEstimator->getContactSensorData()(0) << std::endl;
  static Vec3<float> pDesFootWorldStance[4] = { pFoot[0], pFoot[1], pFoot[2], pFoot[3] };

  // p front mid, p back mid
  Vec3<float> p_fm = (pDesFootWorldStance[0] + pDesFootWorldStance[1]) / 2;
  Vec3<float> p_bm = (pDesFootWorldStance[2] + pDesFootWorldStance[3]) / 2;

  if (current_gait == 4 || current_gait == 13)
  {
    _pitch_des = 0.0;
  }
  else
  {
    _pitch_des =
      _data->stateEstimator->getResult().rpy[1] + _data->stateEstimator->getResult().est_pitch_plane - 0.07 * _x_vel_des;
  }

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

  for (int foot = 0; foot < 4; foot++)
  {
    pFoot[foot] = seResult.position +
                  seResult.rBody.transpose() * (_data->quadruped->getHipLocation(foot) + _data->legController->datas[foot].p);
    footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
  }
  static float z_des[4] = { 0 };

  if (gait != &standing)
  {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }
  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];
    _yaw_des = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {
      footSwingTrajectories[i].setHeight(_dyn_params->Swing_traj_height);

      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      _data->debug->all_legs_info.leg[i].swing_ps.x = pFoot[i](0);
      _data->debug->all_legs_info.leg[i].swing_ps.y = pFoot[i](1);
      _data->debug->all_legs_info.leg[i].swing_ps.z = pFoot[i](2);

      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
      _data->debug->all_legs_info.leg[i].swing_pf.x = pFoot[i](0);
      _data->debug->all_legs_info.leg[i].swing_pf.y = pFoot[i](1);
      _data->debug->all_legs_info.leg[i].swing_pf.z = pFoot[i](2);

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

  // cout << "iter: " << _iterationCounter << " first swing leg 0" << firstSwing[0] << endl;

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

    Vec3<float> offset(0, side_sign[i] * _data->quadruped->_abadLinkLength, 0);

    Vec3<float> pRobotFrame = (_data->quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);

    Vec3<float> pYawCorrected = coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    float pfx_rel = seResult.vWorld[0] * (.5 + _dyn_params->cmpc_bonus_swing) * stance_time +
                    .03f * (seResult.vWorld[0] - v_des_world[0]) +
                    (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC + .03f * (seResult.vWorld[1] - v_des_world[1]) +
                    (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);

    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    pf[0] += pfx_rel;
    pf[1] += pfy_rel;
    pf[2] = z_des[i];

    _updateFoothold(pf, seResult.position, height_map_filter, height_map_raw, map_plane, i);

    if (pf[2] > 0.02)
      std::cout << "PF = " << pf[2] << std::endl;

    footSwingTrajectories[i].setFinalPosition(pf);
    _data->debug->all_legs_info.leg[i].swing_pf.x = pf(0);
    _data->debug->all_legs_info.leg[i].swing_pf.y = pf(1);
    _data->debug->all_legs_info.leg[i].swing_pf.z = pf(2);
  }

  // calc gait
  _iterationCounter++;

  // gait
  // trot leg 0 starts in stance because offset is 0
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  int* mpcTable = gait->getMpcTable();

  for (size_t leg_num = 0; leg_num < 4; leg_num++)
  {
    _data->debug->all_legs_info.leg[leg_num].stance_time = contactStates[leg_num];
    _data->debug->all_legs_info.leg[leg_num].swing_time = swingStates[leg_num];
    _data->debug->all_legs_info.leg[leg_num].phase = gait->getCurrentGaitPhase();
    _data->debug->all_legs_info.leg[leg_num].is_contact = _data->stateEstimator->getContactSensorData()(leg_num);
  }

  updateMPCIfNeeded(mpcTable, *_data, omniMode);

  Vec4<float> se_contactState(0, 0, 0, 0);
  se_contactState = _data->stateEstimator->getContactSensorData().cast<float>();
  static bool is_stance[4] = { 0, 0, 0, 0 };
  static Vec3<float> p_fw[4] = {};
  static Vec3<float> p_fl[4] = {};
  static float delta_yaw[4] = {};
  static Vec3<float> delta_p_bw[4] = {};

  // ROS_INFO_STREAM("is contact: " << se_contactState(0));

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];

    // if first stance
    // if ((is_stance[foot] == 0) && (se_contactState[foot] == 1) && (swingState > 0.65))
    if ((is_stance[foot] == 0) && !(swingState > 0))
    {
      is_stance[foot] = 1;

      pDesFootWorldStance[foot] = pFoot[foot];
      _data->debug->last_p_stance[foot] = ros::toMsg(pFoot[foot]);
      p_fw[foot] = pFoot[foot];
      p_fl[foot] = _data->legController->datas[foot].p + _data->quadruped->getHipLocation(foot);
      delta_p_bw[foot] << 0, 0, 0;
      delta_yaw[foot] = 0;
    }

    delta_p_bw[foot] += seResult.vBody * dt;
    delta_yaw[foot] += seResult.omegaBody(2) * dt;
    _data->debug->last_p_local_stance[foot] =
      ros::toMsg(ori::rpyToRotMat(Vec3<float>(0, 0, delta_yaw[foot])) * (p_fl[foot] - delta_p_bw[foot]));

    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        is_stance[foot] = 0;
        _data->debug->all_legs_info.leg[foot].swing_ps.x = pFoot[foot](0);
        _data->debug->all_legs_info.leg[foot].swing_ps.y = pFoot[foot](1);
        _data->debug->all_legs_info.leg[foot].swing_ps.z = pFoot[foot](2);

        z_des[foot] = pFoot[foot][2];
      }
      // double swing_height = _updateTrajHeight(foot);
      // if (foot == 0)
      // std::cout << "step height = " << swing_height << std::endl;
      footSwingTrajectories[foot].setHeight(_data->userParameters->Swing_traj_height);
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      // footSwingTrajectories[foot].computeStairsSwingTrajectoryBezier(swingState,
      // swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _data->quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<float> pActFootWorld = seResult.rBody.inverse() * (_data->legController->_datas[foot].p
      // + _data->quadruped->getHipLocation(foot)) + seResult.position;
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (_data->legController->datas[foot].v) + seResult.vWorld;

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      _data->debug->all_legs_info.leg[foot].p_des.x = pDesLeg[0];
      _data->debug->all_legs_info.leg[foot].p_des.y = pDesLeg[1];
      _data->debug->all_legs_info.leg[foot].p_des.z = pDesLeg[2];

      _data->debug->all_legs_info.leg[foot].v_des.x = vDesLeg[0];
      _data->debug->all_legs_info.leg[foot].v_des.y = vDesLeg[1];
      _data->debug->all_legs_info.leg[foot].v_des.z = vDesLeg[2];

      _data->debug->all_legs_info.leg[foot].p_w_act.x = pFoot[foot][0];
      _data->debug->all_legs_info.leg[foot].p_w_act.y = pFoot[foot][1];
      _data->debug->all_legs_info.leg[foot].p_w_act.z = pFoot[foot][2];
      // _data->debug->all_legs_info.leg[foot].p_w_act.x = pActFootWorld[0];
      // _data->debug->all_legs_info.leg[foot].p_w_act.y = pActFootWorld[1];
      // _data->debug->all_legs_info.leg[foot].p_w_act.z = pActFootWorld[2];

      _data->debug->all_legs_info.leg[foot].v_w_act.x = vActFootWorld[0];
      _data->debug->all_legs_info.leg[foot].v_w_act.y = vActFootWorld[1];
      _data->debug->all_legs_info.leg[foot].v_w_act.z = vActFootWorld[2];

      _data->debug->all_legs_info.leg[foot].p_w_des.x = pDesFootWorld[0];
      _data->debug->all_legs_info.leg[foot].p_w_des.y = pDesFootWorld[1];
      _data->debug->all_legs_info.leg[foot].p_w_des.z = pDesFootWorld[2];

      _data->debug->all_legs_info.leg[foot].v_w_des.x = vDesFootWorld[0];
      _data->debug->all_legs_info.leg[foot].v_w_des.y = vDesFootWorld[1];
      _data->debug->all_legs_info.leg[foot].v_w_des.z = vDesFootWorld[2];

      _data->debug->body_info.euler_des.x = pBody_RPY_des[0];
      _data->debug->body_info.euler_des.y = pBody_RPY_des[1];
      _data->debug->body_info.euler_des.z = pBody_RPY_des[2];

      if (!_data->userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        _data->legController->commands[foot].pDes = pDesLeg;
        _data->legController->commands[foot].vDes = vDesLeg;
        _data->legController->commands[foot].kpCartesian = Kp;
        _data->legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      // Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> vDesFootWorld(0, 0, 0);
      // Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorldStance[foot] - seResult.position) -
      // _data->quadruped->getHipLocation(foot);
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - _data->quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      // Vec3<float> pActFootWorld = seResult.rBody.inverse() * (_data->legController->_datas[foot].p
      // + _data->quadruped->getHipLocation(foot)) + seResult.position;
      Vec3<float> vActFootWorld = seResult.rBody.inverse() * (_data->legController->datas[foot].v) + seResult.vWorld;

      if (!_data->userParameters->use_wbc) // wbc off
      {
        _data->legController->commands[foot].pDes = pDesLeg;
        _data->legController->commands[foot].vDes = vDesLeg;
        _data->legController->commands[foot].kpCartesian = Kp_stance;
        _data->legController->commands[foot].kdCartesian = Kd_stance;

        _data->legController->commands[foot].forceFeedForward = f_ff[foot];
        _data->legController->commands[foot].kdJoint =
          Vec3<float>(_dyn_params->Kd_joint_0, _dyn_params->Kd_joint_1, _dyn_params->Kd_joint_2).asDiagonal();
      }
      else
      { // Stance foot damping
        _data->legController->commands[foot].pDes = pDesLeg;
        _data->legController->commands[foot].vDes = vDesLeg;
        _data->legController->commands[foot].kpCartesian = 0. * Kp_stance;
        _data->legController->commands[foot].kdCartesian = Kd_stance;
      }

      se_contactState[foot] = contactState;

      _data->debug->all_legs_info.leg[foot].p_des.x = pDesLeg[0];
      _data->debug->all_legs_info.leg[foot].p_des.y = pDesLeg[1];
      _data->debug->all_legs_info.leg[foot].p_des.z = pDesLeg[2];

      _data->debug->all_legs_info.leg[foot].v_des.x = vDesLeg[0];
      _data->debug->all_legs_info.leg[foot].v_des.y = vDesLeg[1];
      _data->debug->all_legs_info.leg[foot].v_des.z = vDesLeg[2];

      _data->debug->all_legs_info.leg[foot].p_w_act.x = pFoot[foot][0];
      _data->debug->all_legs_info.leg[foot].p_w_act.y = pFoot[foot][1];
      _data->debug->all_legs_info.leg[foot].p_w_act.z = pFoot[foot][2];

      _data->debug->all_legs_info.leg[foot].v_w_act.x = vActFootWorld[0];
      _data->debug->all_legs_info.leg[foot].v_w_act.y = vActFootWorld[1];
      _data->debug->all_legs_info.leg[foot].v_w_act.z = vActFootWorld[2];

      // _data->debug->all_legs_info.leg[foot].p_w_des.x = pDesFootWorldStance[foot][0];
      // _data->debug->all_legs_info.leg[foot].p_w_des.y = pDesFootWorldStance[foot][1];
      // _data->debug->all_legs_info.leg[foot].p_w_des.z = pDesFootWorldStance[foot][2];
      _data->debug->all_legs_info.leg[foot].p_w_des.x = pDesFootWorld[0];
      _data->debug->all_legs_info.leg[foot].p_w_des.y = pDesFootWorld[1];
      _data->debug->all_legs_info.leg[foot].p_w_des.z = pDesFootWorld[2];

      _data->debug->all_legs_info.leg[foot].v_w_des.x = vDesFootWorld[0];
      _data->debug->all_legs_info.leg[foot].v_w_des.y = vDesFootWorld[1];
      _data->debug->all_legs_info.leg[foot].v_w_des.z = vDesFootWorld[2];
    }
  }

  _data->stateEstimator->setContactPhase(se_contactState);
  _data->stateEstimator->setSwingPhase(gait->getSwingState());

  // Update For WBC
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  // pBody_RPY_des[1] = 0.;
  pBody_RPY_des[1] = _pitch_des;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  _data->debug->body_info.pos_des.x = pBody_des[0];
  _data->debug->body_info.pos_des.y = pBody_des[1];
  _data->debug->body_info.pos_des.z = pBody_des[2];

  _data->debug->body_info.vel_des.linear.x = vBody_des[0];
  _data->debug->body_info.vel_des.linear.y = vBody_des[1];
  _data->debug->body_info.vel_des.linear.z = vBody_des[2];

  _data->debug->body_info.euler_des.x = pBody_RPY_des[0];
  _data->debug->body_info.euler_des.y = pBody_RPY_des[1];
  _data->debug->body_info.euler_des.z = pBody_RPY_des[2];

  _data->debug->body_info.vel_des.angular.x = vBody_Ori_des[0];
  _data->debug->body_info.vel_des.angular.y = vBody_Ori_des[1];
  _data->debug->body_info.vel_des.angular.z = vBody_Ori_des[2];

  contact_state = gait->getContactState();
}

void VisionMPCLocomotion::_updateFoothold(Vec3<float>& pf,
                                          const Vec3<float>& body_pos_arg,
                                          const grid_map::GridMap& height_map_filter,
                                          const grid_map::GridMap& height_map_raw,
                                          const grid_map::GridMap& map_plane,
                                          int leg)
{
  // Положение лапы в СК тела
  //  Vec3<float> scale(1.2, 1, 1);
  static bool first = true;
  static Vec3<float> freeze_pose;
  Vec3<float> body_pos;
  Vec3<float> p0 = footSwingTrajectories[leg].getInitialPosition();
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
  int row_idx_body_com = height_map_raw.getSize()(0) / 2;
  int col_idx_body_com = height_map_raw.getSize()(1) / 2;
  if (_data->debug->is_map_upd_stop)
  {
    col_idx_body_com -= floor((body_pos_arg[0] - freeze_pose[0]) / height_map_raw.getResolution());
    row_idx_body_com += floor((body_pos_arg[1] - freeze_pose[1]) / height_map_raw.getResolution());
    col_idx_body_com = std::clamp(col_idx_body_com, 0, height_map_raw.getSize()(1) - 1);
    row_idx_body_com = std::clamp(row_idx_body_com, 0, height_map_raw.getSize()(0) - 1);
    // TODO:Call enable update service when touch map edge
  }

  // Минус для преобразования координат
  int x_idx = col_idx_body_com - floor(local_pf[0] / height_map_raw.getResolution());
  int y_idx = row_idx_body_com - floor(local_pf[1] / height_map_raw.getResolution());
  x_idx = std::clamp(x_idx, 0, height_map_raw.getSize()(1) - 1);
  y_idx = std::clamp(y_idx, 0, height_map_raw.getSize()(0) - 1);

  int x_idx_selected = x_idx;
  int y_idx_selected = y_idx;

  _idxMapChecking(local_pf, x_idx, y_idx, x_idx_selected, y_idx_selected, height_map_raw, height_map_raw, leg);

  // Минус для преобразования координат
  pf[0] = -(x_idx_selected - col_idx_body_com) * height_map_raw.getResolution() + body_pos[0];
  pf[1] = -(y_idx_selected - row_idx_body_com) * height_map_raw.getResolution() + body_pos[1];
  auto pf_h = height_map_filter.at("elevation", checkBoundaries(height_map_filter, x_idx_selected, y_idx_selected));
  int p0_x_idx = col_idx_body_com - floor(local_p0[0] / height_map_raw.getResolution());
  int p0_y_idx = row_idx_body_com - floor(local_p0[1] / height_map_raw.getResolution());
  auto p0_h = height_map_filter.at("elevation", checkBoundaries(height_map_filter, p0_x_idx, p0_y_idx));

  //  int counter = 0;
  //  static double step_threshold = 0.07;
  //  double step_height = 0;
  //  for (size_t leg = 0; leg < 4; leg++)
  //  {
  //    Vec3<float> p0 = footSwingTrajectories[leg].getInitialPosition();
  //    Vec3<float> local_p0 = p0 - body_pos;
  //    int p0_x_idx = col_idx_body_com - floor(local_p0[0] / height_map_filter.getResolution());
  //    int p0_y_idx = row_idx_body_com - floor(local_p0[1] / height_map_filter.getResolution());
  //    auto p0_h = height_map_filter.at("elevation", Eigen::Array2i(p0_x_idx, p0_y_idx));

  //    if (p0_h >= step_threshold)
  //    {
  //      counter++;
  //      if (p0_h >= step_height)
  //        step_height = p0_h;
  //    }
  //  }
  //  std::cout << "po_h = " << p0_h << std::endl;
  //  std::cout << "From map " << h << std::endl;
  //  std::cout << "z_offset " << __data->debug->z_offset << std::endl;
  // ЕВРИСТИКА
  static double start_offset = 0.7;
  if (_data->stateEstimator->getResult().position(0) > start_offset)
  {
    static double step_length = 0.6;
    double x = _data->stateEstimator->getResult().position(0) - start_offset;
    double k = x / step_length;
    if (k >= 1.0)
      k = 1.0;
    _data->debug->z_offset = k * 0.40;
  }

  // double mean_p0_h = 0;
  // for (size_t leg = 0; leg < 4; leg++)
  // {
  //   Vec3<float> p0 = footSwingTrajectories[leg].getInitialPosition();
  //   Vec3<float> local_p0 = p0 - body_pos;
  //   int p0_x_idx = col_idx_body_com - floor(local_p0[0] / height_map_filter.getResolution());
  //   int p0_y_idx = row_idx_body_com - floor(local_p0[1] / height_map_filter.getResolution());
  //   auto p0_h = height_map_filter.at("elevation", Eigen::Array2i(p0_x_idx, p0_y_idx));
  //   p0_h = (std::isnan(p0_h)) ? 0. : p0_h;
  //   mean_p0_h += p0_h;
  // }
  // mean_p0_h = mean_p0_h / 4.;
  // _data->debug->z_offset = height_map_raw.at("elevation", Eigen::Array2i(0, 0));
  //  if (counter >= 2)

  //  pf_h -= step_height;

  //  std::cout << "z_offset = " << _data->debug->z_offset << std::endl;
  // In ODOM frame

  // double _floor_plane_height = map_plane.at("smooth_planar", checkBoundaries(map_plane, col_idx_body_com, row_idx_body_com));
  // // Каждые 0,5 секунды вызываем очистку
  // if ((_iterationCounter % 500) == 0)
  //   _locHeightClearance(map_plane, "smooth_planar", checkBoundaries(map_plane, col_idx_body_com, row_idx_body_com), 0.5, 0.1);

  // _data->debug->z_offset = _floor_plane_height;
  // std::cout << "pf_h = " << pf_h << std::endl;
  pf_h -= p0_h;
  // pf_h -= _floor_plane_height;
  pf[2] = (std::isnan(pf_h)) ? 0. : pf_h;
  if (pf[2] > 0.17)
  {
    pf[2] = 0.17;
    std::cout << "Leg height limit" << std::endl;
  }
  // if (leg == 0)
  //   std::cout << "PF_0 = " << pf[2] << std::endl;

  // in WORLD frame
  // pf_h -= floor_plane_height - _data->debug->body_info.pos_act.z;
  //  h =
  // pf_h = p0(2) + (pf_h - p0_h);
  //  std::cout << "After " << pf_h << std::endl;
  //  if (leg == 3 || leg == 2)
}

Eigen::Array2i checkBoundaries(const grid_map::GridMap& map, int col, int row)
{
  col = std::clamp(col, 0, map.getSize()(1));
  row = std::clamp(row, 0, map.getSize()(0));
  return Eigen::Array2i(col, row);
}

void VisionMPCLocomotion::_locHeightClearance(const grid_map::GridMap& map,
                                              std::string layer,
                                              grid_map::Index center,
                                              double rad,
                                              double threshold)
{
  const grid_map::Matrix& data = map[layer];
  double sum = 0;
  double min = 9999;
  double max = -9999;
  size_t cntr = 0;
  for (grid_map_utils::SpiralIterator iterator(map, center, rad); !iterator.isPastEnd(); ++iterator)
  {
    cntr++;
    const grid_map::Index index(*iterator);
    auto cell = data(index(0), index(1));
    cell = (std::isnan(cell)) ? 0. : cell;

    sum += cell;
    if (cell <= min)
      min = cell;
    if (cell >= max)
      max = cell;
  }
  double mean = sum / double(cntr);
  double variance = std::min(std::abs(max - mean), std::abs(mean - min));
  if (std::abs(mean - variance) < threshold)
  {
    // std::cout << "Height estimate CLEAN" << mean << std::endl;
    _floor_plane_height = 0;
  }
  // std::cout << "mean - variance = " << std::abs(mean - variance) << std::endl;
  // std::cout << "mean = " << mean << " variance = " << variance << std::endl;
}

void VisionMPCLocomotion::_idxMapChecking(Vec3<float>& pf,
                                          int x_idx,
                                          int y_idx,
                                          int& x_idx_selected,
                                          int& y_idx_selected,
                                          const grid_map::GridMap& height_map_filter,
                                          const grid_map::GridMap& height_map_raw,
                                          int leg)
{
  grid_map::Index center(x_idx, y_idx);
  // std::cout << " Leg position (x,y) " << pf[0] << " " << pf[1] << std::endl;
  double radius = 0.09;
  // std::cout << "Normal is " << height_map_raw.at("normal_z", Eigen::Array2i(x_idx, y_idx)) <<
  // std::endl;
  for (grid_map_utils::SpiralIterator iterator(height_map_filter, center, radius); !iterator.isPastEnd(); ++iterator)
  {
    auto traversability = height_map_filter.at("normal_vectors_z", *iterator);
    // auto uncertainty_r = height_map_filter.at("uncertainty_range", *iterator);
    // if (leg == 0)
    // std::cout << "traversability = " << traversability << std::endl;
    // If can step
    if (!std::isnan(traversability) && traversability > 0.98 /*&& !std::isnan(uncertainty_r) && uncertainty_r < 0.7*/)
    {
      x_idx_selected = (*iterator)(0);
      y_idx_selected = (*iterator)(1);
      // if ((x_idx != x_idx_selected) && (y_idx != y_idx_selected))
      //   std::cout << "Edit footstep from ( " << x_idx << " " << y_idx << ") to ( " << x_idx_selected << " " << y_idx_selected
      //             << " )" << std::endl;
      return;
    }
  }
  //  std::cout << "Can`t find foothold from map!" << std::endl;
}

void VisionMPCLocomotion::updateMPCIfNeeded(int* mpcTable, ControlFSMData<float>& data, bool omniMode)
{
  // _iterationsBetweenMPC = 30;
  if ((_iterationCounter % _iterationsBetweenMPC) == 0)
  {
    auto seResult = data.stateEstimator->getResult();
    float* p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;

    // Stand gait
    // if (current_gait == 4)
    if (current_gait == 30)
    {
      // float trajInitial[12] = {
      //     _roll_des,
      //     _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
      //     (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
      //     (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
      //     (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
      //     (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
      //     0,
      //     0,
      //     0,
      //     0,
      //     0,
      //     0};

      // for (int i = 0; i < horizonLength; i++)
      //   for (int j = 0; j < 12; j++)
      //     trajAll[12 * i + j] = trajInitial[j];
    }
    else
    {
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

      // float trajInitial[12] = {(float)rpy_comp[0], // 0 roll des
      //                          (float)rpy_comp[1], // 1 pitch des
      //                          _yaw_des,           // 2 yaw des
      //                          xStart,              // 3 x body des
      //                          yStart,              // 4 y body des
      //                          (float)_body_height, // 5 z body des
      //                          0,                   // 6 velocity roll des
      //                          0,                   // 7 velocity pitch des
      //                          _yaw_turn_rate,      // 8 velocity yaw des
      //                          v_des_world[0],      // 9 vx body des
      //                          v_des_world[1],      // 10 vy body des
      //                          0};                  // 11 vz body des

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

      for (int i = 0; i < horizonLength; i++)
      {
        for (int j = 0; j < 12; j++)
        {
          trajAll[12 * i + j] = trajInitial[j];
        }

        if (i == 0) // start at current position  TODO consider not doing this
        {
          // trajAll[3] = hw_i->state_estimator->se_pBody[0];
          // trajAll[4] = hw_i->state_estimator->se_pBody[1];
          // trajAll[2] = seResult.rpy[2];
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

void VisionMPCLocomotion::solveDenseMPC(int* mpcTable, ControlFSMData<float>& data)
{
  auto seResult = data.stateEstimator->getResult();

  // original
  // float Q[12] = { 0.25, 0.25, 10, 2, 2, 50, 0, 0, 0.3, 0.2, 0.2, 0.1 };
  // float Q[12] = {2.5, 2.5, 10, 50, 50, 100, 0, 0, 0.5, 0.2, 0.2, 0.1};
  // float Q[12] = { 2.5, 2.5, 10, 300, 300, 300, 0, 0, 0.5, 1.5, 1.5, 1 };

  float Q[12] = { 10, 10, 15, 3, 3, 30, 0.5, 0.5, 3, 0.4, 0.4, 0.2 };

  float roll = seResult.rpy[0];
  float pitch = seResult.rpy[1];
  float yaw = seResult.rpy[2];
  float* weights = Q;
  float alpha = 4e-5; // make setting eventually
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

  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    // x_comp_integral += _dyn_params->cmpc_x_drag * pxy_err[0] * dtMPC /
    // vxy[0];
    x_comp_integral += _dyn_params->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  // printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_dyn_params->jcqp_max_iter, _dyn_params->jcqp_rho, _dyn_params->jcqp_sigma, _dyn_params->jcqp_alpha,
                         _dyn_params->jcqp_terminate, _dyn_params->use_jcqp);
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
      f[axis] = get_solution(leg * 3 + axis);

    // printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void VisionMPCLocomotion::solveSparseMPC(int* mpcTable, ControlFSMData<float>& data)
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

void VisionMPCLocomotion::initSparseMPC()
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

float VisionMPCLocomotion::_updateTrajHeight(size_t foot)
{
  double h = 0;
  double k = 0.3;
  double obst_h = footSwingTrajectories[foot].getFinalPosition()(2) - footSwingTrajectories[foot].getInitialPosition()(2);
  h = obst_h * k;
  // Saturate h
  h = std::clamp(h, -_dyn_params->Swing_traj_height, _dyn_params->Swing_traj_height);
  double out = _dyn_params->Swing_traj_height + h;
  out = std::clamp(out, -MAX_STEP_HEIGHT, MAX_STEP_HEIGHT);
  return out;
}
