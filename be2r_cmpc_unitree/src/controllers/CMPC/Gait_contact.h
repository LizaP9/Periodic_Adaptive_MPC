#pragma once

#include "cppTypes.h"
#include <iostream>
#include <queue>
#include <string>

class Gait_contact
{
public:
  virtual ~Gait_contact() = default;

  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual float getCurrentGaitPhase() = 0;
  virtual int getCurrentGaitIteration() = 0;
  virtual void earlyContactHandle(Vec4<uint8_t>, int, int) {}
  virtual void restoreDefaults() {}
  virtual void updatePeriod(int) {}

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGaitContact : public Gait_contact
{
public:
  OffsetDurationGaitContact(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGaitContact();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getCurrentGaitIteration();
  void debugPrint();
  void earlyContactHandle(Vec4<uint8_t> footSensorState, int iterationsBetweenMPC, int currentIteration);
  void restoreDefaults();
  void updatePeriod(int);

  bool is_contact[4] = {};
  float delta_t[4] = {};

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
