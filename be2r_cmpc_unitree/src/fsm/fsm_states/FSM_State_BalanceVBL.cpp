/*============================= Testing ==============================*/
/**
 * State for be2r testing cases
 */

#include "FSM_State_BalanceVBL.h"

// using namespace std;
using std::cout;
using std::endl;

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template<typename T>
FSM_State_BalanceVBL<T>::FSM_State_BalanceVBL(ControlFSMData<T>* _controlFSMData) : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_VBL, "BALANCE_VBL")
{
  _data = _controlFSMData;
  balanceController = new BalanceController();
  balance_controller_vbl = new BalanceControllerVBL();
  reference_grf = new ReferenceGRF();
}

template<typename T>
void FSM_State_BalanceVBL<T>::onEnter()
{
  // Default is to not transition
  this->nextStateName = this->stateName;

  // Reset the transition data
  this->transitionData.zero();

  // Reset iteration counter
  iter = 0;
}

template<typename T>
void FSM_State_BalanceVBL<T>::runBalanceController()
{
  double minForce = 25;
  double maxForce = 500;
  // double contactStateScheduled[4] = {1, 1, 1, 1};
  double contactStateScheduled[4];

  for (int i = 0; i < 4; i++)
  {
    contactStateScheduled[i] = _data->gaitScheduler->gaitData.contactStateScheduled(i);
  }

  double minForces[4]; // = {minForce, minForce, minForce, minForce};
  double maxForces[4]; // = {maxForce, maxForce, maxForce, maxForce};
  for (int leg = 0; leg < 4; leg++)
  {
    minForces[leg] = contactStateScheduled[leg] * minForce;
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }

  double COM_weights_stance[3] = { 1, 1, 10 };
  double Base_weights_stance[3] = { 20, 10, 10 };
  double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3], omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

  for (int i = 0; i < 4; i++)
  {
    se_xfb[i] = (double)_data->stateEstimator->getResult().orientation(i);
  }

  for (int i = 0; i < 3; i++)
  {
    rpy[i] = 0.0;
    p_act[i] = (double)_data->stateEstimator->getResult().position(i);
    omegaDes[i] = 0.0;
    v_act[i] = (double)_data->stateEstimator->getResult().vBody(i);
    v_des[i] = 0.0;

    se_xfb[4 + i] = (double)_data->stateEstimator->getResult().position(i);
    se_xfb[7 + i] = (double)_data->stateEstimator->getResult().omegaBody(i);
    se_xfb[10 + i] = (double)_data->stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
    // kpCOM[i] = 50.0;
    // kdCOM[i] = 10.0;
    // kpBase[i] = 200;
    // kdBase[i] = 20;

    kpCOM[i] = 5.0e-1;
    kdCOM[i] = 5.0e-1;
    kpBase[i] = 1.0e2;
    kdBase[i] = 5.0e-1;
  }

  // kpBase[1] *= -1.0;
  // kdBase[1] *= -1.0;

  p_des[0] = 0.0;
  p_des[1] = 0.0;
  p_des[2] = 0.25;

  v_des[0] = 0.0;
  v_des[1] = 0.0;
  v_des[2] = 0.0;

  Vec3<T> pFeetVec;
  Vec3<T> pFeetVecCOM;
  // Get the foot locations relative to COM
  for (int leg = 0; leg < 4; leg++)
  {
    computeLegJacobianAndPosition(**&_data->quadruped, _data->legController->datas[leg].q, (Mat3<T>*)nullptr, &pFeetVec, leg);
    pFeetVecCOM = _data->stateEstimator->getResult().rBody.transpose() * (_data->quadruped->getHipLocation(leg) + _data->legController->datas[leg].p);

    pFeet[leg * 3] = (double)pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
  }

  balanceController->set_alpha_control(0.01);
  balanceController->set_friction(0.5);
  balanceController->set_mass(12.0);
  balanceController->set_wrench_weights(COM_weights_stance, Base_weights_stance);
  balanceController->set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  balanceController->set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceController->SetContactData(contactStateScheduled, minForces, maxForces);
  // cout << "O_err: " << O_err[0] << " " << O_err[1] << " " << O_err[2] << endl;
  // cout << rpy[0] - _data->stateEstimator->getResult().rpy[0] << " " << rpy[1] - _data->stateEstimator->getResult().rpy[1] << " " << _data->stateEstimator->getResult().rpy[2] << endl;
  balanceController->updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act, O_err, 0.0);

  double fOpt[12];
  balanceController->solveQP_nonThreaded(fOpt);

  footFeedForwardForces = Mat34<T>::Zero();

  // Copy the results to the feed forward forces
  for (int leg = 0; leg < 4; leg++)
  {
    footFeedForwardForces.col(leg) << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];

    Vec3<float> f_ff;
    f_ff << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];

    _data->legController->commands[leg].forceFeedForward = f_ff;
    cout << "f" << leg << ": " << f_ff.norm() << endl;
  }
}

template<typename T>
void FSM_State_BalanceVBL<T>::runBalanceControllerVBL()
{
  double minForce = 25;
  double maxForce = 500;
  double contactStateScheduled[4];
  float mass = 12.0;

  for (int i = 0; i < 4; i++)
  {
    contactStateScheduled[i] = _data->gaitScheduler->gaitData.contactStateScheduled(i);
  }

  double minForces[4] = { minForce, minForce, minForce, minForce };
  double maxForces[4] = { maxForce, maxForce, maxForce, maxForce };
  for (int leg = 0; leg < 4; leg++)
  {
    minForces[leg] = contactStateScheduled[leg] * minForce;
    maxForces[leg] = contactStateScheduled[leg] * maxForce;
  }

  double COM_weights_stance[3] = { 1, 1, 10 };
  double Base_weights_stance[3] = { 20, 10, 10 };
  double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy_act[3], rpy[3], omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

  for (int i = 0; i < 4; i++)
  {
    se_xfb[i] = (double)_data->stateEstimator->getResult().orientation(i);
  }

  for (int i = 0; i < 3; i++)
  {
    rpy[i] = 0.0;
    rpy_act[i] = _data->stateEstimator->getResult().rpy[i];
    p_act[i] = (double)_data->stateEstimator->getResult().position(i);
    omegaDes[i] = 0.0;
    v_act[i] = (double)_data->stateEstimator->getResult().vBody(i);
    v_des[i] = 0.0;

    se_xfb[4 + i] = (double)_data->stateEstimator->getResult().position(i);
    se_xfb[7 + i] = (double)_data->stateEstimator->getResult().omegaBody(i);
    se_xfb[10 + i] = (double)_data->stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
    kpCOM[i] = 50.0;
    kdCOM[i] = 10.0;
    kpBase[i] = 200;
    kdBase[i] = 20;
  }
  p_des[0] = 0.0;
  p_des[1] = 0.0;
  p_des[2] = 0.25;

  v_des[0] = 0.0;
  v_des[1] = 0.0;
  v_des[2] = 0.0;

  Vec3<T> pFeetVec;
  Vec3<T> pFeetVecCOM;
  // Get the foot locations relative to COM
  for (int leg = 0; leg < 4; leg++)
  {
    computeLegJacobianAndPosition(**&_data->quadruped, _data->legController->datas[leg].q, (Mat3<T>*)nullptr, &pFeetVec, leg);
    pFeetVecCOM = _data->stateEstimator->getResult().rBody.transpose() * (_data->quadruped->getHipLocation(leg) + _data->legController->datas[leg].p);

    pFeet[leg * 3] = (double)pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = (double)pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = (double)pFeetVecCOM[2];
  }
  double f_ref_in[12] = { 0 };
  double f = 10;
  f_ref_in[2] = f;
  f_ref_in[5] = f;
  f_ref_in[8] = f;
  f_ref_in[11] = f;

  // reference_grf->set_alpha_control(0.01);
  // reference_grf->set_mass(mass);
  // reference_grf->set_RobotLimits();
  // reference_grf->set_worldData();
  // reference_grf->SetContactData(contactStateScheduled, minForces, maxForces, 100, 4);
  // reference_grf->updateProblemData(pFeet, p_des);

  double f_opt_in[4];
  // reference_grf->solveQP_nonThreaded(f_opt_in);

  f_ref_in[2] = f_opt_in[0];
  f_ref_in[5] = f_opt_in[1];
  f_ref_in[8] = f_opt_in[2];
  f_ref_in[11] = f_opt_in[3];

  // cout << "f_ref_in: " << f_ref_in[2] << endl;

  double Q_x[3] = { 1 };
  double Q_dx[3] = { 1 };
  double Q_w[3] = { 1 };
  double Q_dw[3] = { 1 };

  balance_controller_vbl->set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balance_controller_vbl->SetContactData(contactStateScheduled, minForces, maxForces, 0, 4);
  balance_controller_vbl->set_worldData();
  balance_controller_vbl->set_LQR_weights(Q_x, Q_dx, Q_w, Q_dw, 1.0e-2, 1.0e-2);
  balance_controller_vbl->set_RobotLimits();
  balance_controller_vbl->updateProblemData(se_xfb, pFeet, pFeet, rpy, rpy_act);
  // balance_controller_vbl->set_reference_GRF(f_ref_in);

  double fOpt[12];
  Eigen::VectorXd f_unc = balance_controller_vbl->getFunc();

  for (uint8_t i = 0; i < 12; i++)
  {
    fOpt[i] = f_unc(i);
  }

  // balance_controller_vbl->solveQP_nonThreaded(fOpt);

  footFeedForwardForces = Mat34<T>::Zero();

  // Copy the results to the feed forward forces
  for (int leg = 0; leg < 4; leg++)
  {
    footFeedForwardForces.col(leg) << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];

    Vec3<float> f_ff;
    f_ff << (T)fOpt[leg * 3], (T)fOpt[leg * 3 + 1], (T)fOpt[leg * 3 + 2];

    _data->legController->commands[leg].forceFeedForward = f_ff;
  }

  cout << fOpt[0] << endl;
  cout << fOpt[1] << endl;
  cout << fOpt[2] << endl;
  cout << endl;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template<typename T>
void FSM_State_BalanceVBL<T>::run()
{
  // runBalanceController();
  runBalanceControllerVBL();
}

/**
 * Manages which states can be transitioned into either by the user
 * commands or state event triggers.
 *
 * @return the enumerated FSM state name to transition into
 */
template<typename T>
FSM_StateName FSM_State_BalanceVBL<T>::checkTransition()
{
  this->nextStateName = this->stateName;
  iter++;

  // Switch FSM control mode
  switch ((int)this->_data->userParameters->FSM_State)
  {
    case K_BALANCE_VBL:
      break;

    case K_STAND_UP:
      // Requested switch to Stand Up
      this->nextStateName = FSM_StateName::STAND_UP;
      break;

    case K_PASSIVE:
      this->nextStateName = FSM_StateName::PASSIVE;
      break;

    case K_BALANCE_STAND:
      this->nextStateName = FSM_StateName::BALANCE_STAND;
      break;

    default:
      std::cout << "[CONTROL FSM] Bad Request: Cannot transition from "
                << K_BALANCE_VBL << " to "
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
TransitionData<T> FSM_State_BalanceVBL<T>::transition()
{
  // Finish Transition
  switch (this->nextStateName)
  {
    case FSM_StateName::PASSIVE:
      this->transitionData.done = true;
      break;

    case FSM_StateName::STAND_UP:
      this->transitionData.done = true;
      break;

    case FSM_StateName::BALANCE_STAND:
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
void FSM_State_BalanceVBL<T>::onExit()
{
  // Nothing to clean up when exiting
  // this->_data->legController->zeroCommand();

  this->_data->legController->setEnabled(false);
}

template class FSM_State_BalanceVBL<float>;
