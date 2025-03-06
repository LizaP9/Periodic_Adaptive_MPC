/*============================= FSM State =============================*/
/**
 * FSM State base class
 */

#include "FSM_State.h"

/**
 * Constructor for the FSM State class.
 *
 * @param _controlFSMData holds all of the relevant control data
 * @param stateNameIn the enumerated state name
 * @param stateStringIn the string name of the current FSM state
 */
template<typename T>
FSM_State<T>::FSM_State(ControlFSMData<T>* _controlFSMData, FSM_StateName stateNameIn, std::string stateStringIn) : _data(_controlFSMData), stateName(stateNameIn), stateString(stateStringIn)
{
  transitionData.zero();
  std::cout << "[FSM_State] Initialized FSM state: " << stateStringIn << std::endl;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param qDes desired joint position
 * @param dqDes desired joint velocity
 */
template<typename T>
void FSM_State<T>::jointPDControl(int leg, Vec3<T> qDes, Vec3<T> qdDes)
{
  // MIT old params
  kpMat << 80, 0, 0, 0, 80, 0, 0, 0, 80;
  kdMat << 1, 0, 0, 0, 1, 0, 0, 0, 1;

  _data->legController->commands[leg].kpJoint = Eigen::DiagonalMatrix<T, 3>(_data->userParameters->Kp_joint_0, _data->userParameters->Kp_joint_1,
                                                                             _data->userParameters->Kp_joint_2);
  _data->legController->commands[leg].kdJoint = Eigen::DiagonalMatrix<T, 3>(_data->userParameters->Kd_joint_0, _data->userParameters->Kd_joint_1,
                                                                             _data->userParameters->Kd_joint_2);

  _data->legController->commands[leg].qDes = qDes;
  _data->legController->commands[leg].qdDes = qdDes;
}

/**
 * Cartesian impedance control for a given leg.
 *
 * @param leg the leg number to control
 * @param pDes desired foot position
 * @param vDes desired foot velocity
 * @param kp_cartesian P gains
 * @param kd_cartesian D gains
 */
template<typename T>
void FSM_State<T>::cartesianImpedanceControl(int leg, Vec3<T> pDes, Vec3<T> vDes,
                                             Vec3<double> kp_cartesian, Vec3<double> kd_cartesian)
{
  _data->legController->commands[leg].pDes = pDes;
  // Create the cartesian P gain matrix
  kpMat << kp_cartesian[0], 0, 0, 0, kp_cartesian[1], 0, 0, 0, kp_cartesian[2];
  _data->legController->commands[leg].kpCartesian = kpMat;

  _data->legController->commands[leg].vDes = vDes;
  // Create the cartesian D gain matrix
  kdMat << kd_cartesian[0], 0, 0, 0, kd_cartesian[1], 0, 0, 0, kd_cartesian[2];
  _data->legController->commands[leg].kdCartesian = kdMat;
}

/**
 *  Было изначально закомменчено
 */
// template <typename T>
// void FSM_State<T>::footstepHeuristicPlacement(int leg)
// {

//   // Create the projection matrix for the 2D foot placement components
//   Mat23<float> projectionMatrix;
//   projectionMatrix << 1, 0, 0, 0, 1, 0;

//   Vec3<float> velDes = _data->_desiredStateCommand->data.stateDes.block<3, 1>(6, 0);
//   Vec3<float> angVelDes = _data->_desiredStateCommand->data.stateDes.block<3, 1>(9, 0);
//   Mat3<float> rBody = _data->_stateEstimate.rBody;

//   // Find each of the footstep locations for the swing feet
//   for (int leg = 0; leg < 4; leg++)
//   {
//     if (_data->_gaitScheduler->gaitData.contactStateScheduled(leg))
//     {
//       // The leg is in contact so nothing to do here
//     }
//     else
//     {
//       if (_data->_gaitScheduler->gaitData._currentGait == GaitType::TRANSITION_TO_STAND)
//       {
//         // Position the legs under the hips to stand...
//         // Could also get rid of this and simply send 0 velocity ang vel
//         // from the CoM desired planner...
//         commands Vec3<float> posHip = _data->quadruped.getHipLocation(leg);
//         footstepLocations.col(leg) << projectionMatrix.transpose() * projectionMatrix *
//                                           (_data->_stateEstimate.position + // rBody *
//                                            posHip);
//       }
//       else
//       {
//         // Pull out the approximate yaw rate component of the robot in the
//         world.Vec3<float> yaw_rate;
//         yaw_rate << 0, 0, _stateEstimate.omegaWorld(3);

//         Vec3<float> posHip = _data->quadruped.getHipLocation(leg);

//         float timeStance = _data->_gaitScheduler->gaitData.timeStance(leg);

//         // Footstep heuristic composed of several parts in the world frame
//         footstepLocations.col(leg) << projectionMatrix.transpose() * projectionMatrix * // Ground
//         projection
//                                           (_stateEstimate.position +                    // rBody
//                                           * posHip
//                                            +                                            // Foot
//                                            under hips timeStance / 2 * velDes + // Raibert
//                                            Heuristic timeStance / 2 * (angVelDes.cross(rBody *
//                                            posHip)) +          // Turning Raibert Heuristic
//                                            (_stateEstimate.vBody - velDes));
//       }
//     }
//   }
// }

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template<typename T>
void FSM_State<T>::runControls()
{
  // This option should be set from the user interface or autonomously
  // eventually
  int CONTROLLER_OPTION = 1;

  // Reset the forces and steps to 0
  footFeedForwardForces = Mat34<T>::Zero();
  footstepLocations = Mat34<T>::Zero();

  // Choose the controller to run for picking step locations and balance forces
  if (CONTROLLER_OPTION == 0)
  {
    // Test to make sure we can control the robot these will be calculated by
    // the controllers
    for (int leg = 0; leg < 4; leg++)
    {
      footFeedForwardForces.col(leg) << 0.0, 0.0, 0; //-220.36;
      // footFeedForwardForces.col(leg) = stateEstimate.rBody *
      // footFeedForwardForces.col(leg);

      footstepLocations.col(leg) << 0.0, 0.0, -_data->quadruped->_maxLegLength / 2;
    }
  }
  else if (CONTROLLER_OPTION == 1)
  {
    // QP Balance Controller
    // runBalanceController();

    // Swing foot landing positions are calculated with heuristics
    for (int leg = 0; leg < 4; leg++)
    {
      footstepLocations.col(leg) << 0.0, 0.0, -_data->quadruped->_maxLegLength / 2;
    } // footstepHeuristicPlacement();
  }
  else if (CONTROLLER_OPTION == 2)
  {
    // WBC
    // runWholeBodyController();
  }
  else if (CONTROLLER_OPTION == 3)
  {
    // cMPC
    // runConvexModelPredictiveController();

    // Swing foot landing positions are calculated with heuristics
    // footstepHeuristicPlacement();
  }
  else if (CONTROLLER_OPTION == 4)
  {
    // RPC
    // runRegularizedPredictiveController();
  }
  else
  {
    // Zero out the commands if a controller was not selected
    jointFeedForwardTorques = Mat34<float>::Zero(); // feed forward joint torques
    jointPositions = Mat34<float>::Zero();          // joint angle positions
    jointVelocities = Mat34<float>::Zero();         // joint angular velocities
    footFeedForwardForces = Mat34<float>::Zero();   // feedforward forces at the feet
    footPositions = Mat34<float>::Zero();           // cartesian foot positions
    footVelocities = Mat34<float>::Zero();

    // Print an error message
    std::cout << "[FSM_State] ERROR: No known controller was selected: " << CONTROLLER_OPTION
              << std::endl;
  }
}

/**
 *
 */
// template <typename T>
// void FSM_State<T>::runBalanceController()
// {
// double minForce = 25;
// double maxForce = 500;
// double contactStateScheduled[4]; // = {1, 1, 1, 1};
// for (int i = 0; i < 4; i++)
// {
//   contactStateScheduled[i] = _data->_gaitScheduler->gaitData.contactStateScheduled(i);
// }

// double minForces[4]; // = {minForce, minForce, minForce, minForce};
// double maxForces[4]; // = {maxForce, maxForce, maxForce, maxForce};
// for (int leg = 0; leg < 4; leg++)
// {
//   minForces[leg] = contactStateScheduled[leg] * minForce;
//   maxForces[leg] = contactStateScheduled[leg] * maxForce;
// }

// double COM_weights_stance[3] = {1, 1, 10};
// double Base_weights_stance[3] = {20, 10, 10};
// double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3], omegaDes[3];
// double se_xfb[13];
// double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

// for (int i = 0; i < 4; i++)
// {
//   se_xfb[i] = (double)_data->stateEstimator->getResult().orientation(i);
// }
// // se_xfb[3] = 1.0;
// for (int i = 0; i < 3; i++)
// {
//   rpy[i] = 0; //(double)_data->stateEstimator->getResult().rpy(i);
//   p_des[i] = (double)_data->stateEstimator->getResult().position(i);
//   p_act[i] = (double)_data->stateEstimator->getResult().position(i);
//   omegaDes[i] = 0; //(double)_data->stateEstimator->getResult().omegaBody(i);
//   v_act[i] = (double)_data->stateEstimator->getResult().vBody(i);
//   v_des[i] = (double)_data->stateEstimator->getResult().vBody(i);

//   se_xfb[4 + i] = (double)_data->stateEstimator->getResult().position(i);
//   se_xfb[7 + i] = (double)_data->stateEstimator->getResult().omegaBody(i);
//   se_xfb[10 + i] = (double)_data->stateEstimator->getResult().vBody(i);

//   // Set the translational and orientation gains
//   kpCOM[i] = (double)_data->controlParameters->kpCOM(i);
//   kdCOM[i] = (double)_data->controlParameters->kdCOM(i);
//   kpBase[i] = (double)_data->controlParameters->kpBase(i);
//   kdBase[i] = (double)_data->controlParameters->kdBase(i);
// }

// Vec3<T> pFeetVec;
// Vec3<T> pFeetVecCOM;
// // Get the foot locations relative to COM
// for (int leg = 0; leg < 4; leg++)
// {
//   computeLegJacobianAndPosition(**&_data->quadruped, _data->legController->datas[leg].q, (Mat3<T>*)nullptr, &pFeetVec, 1);
//   //pFeetVecCOM = _data->stateEstimator->getResult().rBody.transpose() *
//   //(_data->quadruped->getHipLocation(leg) + pFeetVec);

//   pFeetVecCOM = _data->stateEstimator->getResult().rBody.transpose() * (_data->quadruped->getHipLocation(leg) + _data->legController->datas[leg].p);

//   pFeet[leg * 3] = (double)pFeetVecCOM[0];
//   pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
//   pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
// }

// balanceController.set_alpha_control(0.01);
// balanceController.set_friction(0.5);
// balanceController.set_mass(46.0);
// balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
// balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
// balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
// balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
// balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act, O_err, 0.0);

// double fOpt[12];
// balanceController.solveQP_nonThreaded(fOpt);

// // Publish the results over LCM
// // balanceController.publish_data_lcm();

// // Copy the results to the feed forward forces
// for (int leg = 0; leg < 4; leg++)
// {
//   footFeedForwardForces.col(leg) << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];
// }
// }

/**
 * Gait independent formulation for choosing appropriate GRF and step locations
 * as well as converting them to leg controller understandable values.
 */
template<typename T>
void FSM_State<T>::turnOnAllSafetyChecks()
{
  // Pre controls safety checks
  checkSafeOrientation = true; // check roll and pitch

  // Post control safety checks
  checkPDesFoot = true;         // do not command footsetps too far
  checkForceFeedForward = true; // do not command huge forces
  checkLegSingularity = true;   // do not let leg
  checkJointLimits = true;
}

/**
 *
 */
template<typename T>
void FSM_State<T>::turnOffAllSafetyChecks()
{
  // Pre controls safety checks
  checkSafeOrientation = false; // check roll and pitch

  // Post control safety checks
  checkPDesFoot = false;         // do not command footsetps too far
  checkForceFeedForward = false; // do not command huge forces
  checkLegSingularity = false;   // do not let leg
  checkJointLimits = false;
}

template<typename T>
Vec3<T> FSM_State<T>::findAngles(uint8_t leg_num, Vec3<T> p_act)
{
  Vec3<T> q_eval;
  q_eval.Zero();

  const float AB = 0.0838;
  const float BC = 0.2;
  const float CD = 0.2;

  float L = 0;
  float f = 0;
  float Qrad = 0;
  float Q0rad = 0;
  float L1 = 0;
  float a_rad = 0;
  float Arad = 0;
  float a2_rad = 0;
  float a1_rad = 0;

  float x = -p_act(1);
  float y = p_act(0);
  float z = p_act(2);

  L = sqrt(x * x + z * z);
  f = sqrt(L * L - AB * AB);
  Qrad = acos(AB / L);
  Q0rad = acos(x / L);

  if (leg_num == 1 || leg_num == 3)
  {
    Q0rad = acos(-x / L);
  }

  L1 = sqrt(f * f + y * y);
  a_rad = acos((L1 * L1 - BC * BC - CD * CD) / (-2 * BC * CD));
  Arad = acos((CD * CD - BC * BC - L1 * L1) / (-2 * BC * L1));
  a2_rad = acos(y / L1);
  a1_rad = M_PI / 2 - a2_rad;

  // std::cout << "L: " << L << " f: " << f << " Qrad: " << Qrad << " Q0rad: " << Q0rad << " L1: "
  // << L1 << " a_rad: " << a_rad << " Arad: " << Arad << " a2_rad: " << a2_rad << " a1_rad: " <<
  // a1_rad << std::endl;

  switch (leg_num)
  {
    // Front Right
    case 0:
      q_eval(0) = -(Qrad - Q0rad);
      q_eval(1) = Arad - a1_rad;
      q_eval(2) = -(M_PI - a_rad);
      break;

    // Front Left
    case 1:
      q_eval(0) = (Qrad - Q0rad);
      q_eval(1) = Arad - a1_rad;
      q_eval(2) = -(M_PI - a_rad);
      break;

    // Rear Right
    case 2:
      q_eval(0) = -(Qrad - Q0rad);
      q_eval(1) = Arad - a1_rad;
      q_eval(2) = -(M_PI - a_rad);
      break;

    // Rear Left
    case 3:
      q_eval(0) = (Qrad - Q0rad);
      q_eval(1) = Arad - a1_rad;
      q_eval(2) = -(M_PI - a_rad);
      break;
  }

  return q_eval;
}

// template class FSM_State<double>;
template class FSM_State<float>;
