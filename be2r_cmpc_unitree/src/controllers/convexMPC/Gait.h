#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include "cppTypes.h"
#include <iostream>
#include <queue>
#include <string>

class Gait
{
public:
  virtual ~Gait() = default;

  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual int getCurrentGaitPhase() = 0;
  virtual void debugPrint() {}
  virtual void earlyContactHandle(Vec4<uint8_t>, int, int) {}
  virtual void restoreDefaults() {}
  virtual void updatePeriod(int) {}

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait
{
public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();
  void earlyContactHandle(Vec4<uint8_t> footSensorState, int iterationsBetweenMPC, int currentIteration);
  void restoreDefaults();
  void updatePeriod(int);

private:
  int* _mpc_table;
  Array4i _offsets;        // offset in mpc segments
  Array4i _durations;      // duration of step in mpc segments
  Array4f _offsetsFloat;   // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  Array4i _durations_defaults;
  Array4f _durationsF_defaults;
  Array4i _offsets_defaults;
  Array4f _offsetsF_defaults;
  int _stance;
  int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};

class MixedFrequncyGait : public Gait
{
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  int getCurrentGaitPhase();
  void debugPrint();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

#endif // PROJECT_GAIT_H
