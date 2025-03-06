#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "Controllers/GaitScheduler.h"
#include "Controllers/LegController.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/Quadruped.h"
#include "SimUtilities/GamepadCommand.h"
#include "ros_read_param.h"
#include <be2r_cmpc_unitree/ros_dynamic_paramsConfig.h>
#include <debug.hpp>
#include <ros/ros.h>

/**
 *
 */
template<typename T>
struct ControlFSMData
{
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Quadruped<T>* quadruped;
  StateEstimatorContainer<T>* stateEstimator;
  LegController<T>* legController;
  GaitScheduler<T>* gaitScheduler;
  GamepadCommand* gamepad_command;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig* userParameters;
  StaticParams* staticParams;
  Debug* debug;
};

template struct ControlFSMData<float>;

#endif // CONTROLFSM_H
