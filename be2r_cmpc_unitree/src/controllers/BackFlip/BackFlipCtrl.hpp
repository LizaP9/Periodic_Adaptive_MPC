#ifndef BACKFLIP_CTRL
#define BACKFLIP_CTRL

#include "DataReadCtrl.hpp"
#include "DataReader.hpp"
#include <Controllers/LegController.h>
#include <Dynamics/FloatingBaseModel.h>

template <typename T>
class BackFlipCtrl : public DataReadCtrl<T>
{
public:
  BackFlipCtrl(DataReader*, float _dt);
  virtual ~BackFlipCtrl();

  virtual void OneStep(float _curr_time, bool b_preparation, LegControllerCommand<T>* command);

protected:
  void _update_joint_command();
};

#endif
