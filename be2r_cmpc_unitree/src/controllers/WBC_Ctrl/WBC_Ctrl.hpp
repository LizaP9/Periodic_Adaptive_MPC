#ifndef WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include "cppTypes.h"
#include <ControlFSMData.h>
#include <Dynamics/FloatingBaseModel.h>
#include <Dynamics/Quadruped.h>
#include <WBC/WBIC/KinWBC.hpp>
#include <WBC/WBIC/WBIC.hpp>

#define WBCtrl WBC_Ctrl<T>

template<typename T>
class WBC_Ctrl
{
public:
  WBC_Ctrl(FloatingBaseModel<T> model);
  virtual ~WBC_Ctrl();

  void run(void* input, ControlFSMData<T>& data);
  void setFloatingBaseWeight(const T& weight)
  {
    _wbic_data->_W_floating = DVec<T>::Constant(6, weight);
  }

protected:
  virtual void _ContactTaskUpdate(void* input, ControlFSMData<T>& data) = 0;
  virtual void _ContactTaskUpdateTEST(void* input, ControlFSMData<T>& data)
  {
    (void)input;
    (void)data;
  }
  virtual void _LCM_PublishData() {}
  void _UpdateModel(const StateEstimate<T>& state_est, const LegControllerData<T>* leg_data);
  void _UpdateLegCMD(ControlFSMData<T>& data);
  void _ComputeWBC();

  KinWBC<T>* _kin_wbc;
  WBIC<T>* _wbic;
  WBIC_ExtraData<T>* _wbic_data;

  FloatingBaseModel<T> _model;
  std::vector<ContactSpec<T>*> _contact_list;
  std::vector<Task<T>*> _task_list;

  DMat<T> _A;
  DMat<T> _Ainv;
  DVec<T> _grav;
  DVec<T> _coriolis;

  FBModelState<T> _state;

  DVec<T> _full_config;
  DVec<T> _tau_ff;
  DVec<T> _des_jpos;
  DVec<T> _des_jvel;

  Vec3<T> _Kp_joint;
  Vec3<T> _Kd_joint;
  // std::vector<T> _Kp_joint_swing, _Kd_joint_swing;

  unsigned long long _iter;
};
#endif
