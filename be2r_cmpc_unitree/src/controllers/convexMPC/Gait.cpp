#include "Gait.h"

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name)
  : _offsets(offsets.array()),
    _durations(durations.array()),
    _nIterations(nSegment) // == horizon
{

  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  // offset и duration в диапазоне горизонта (0 -- 1)
  _offsetsFloat = offsets.cast<float>() / (float)nSegment;
  _durationsFloat = durations.cast<float>() / (float)nSegment;
  _durationsF_defaults = _durationsFloat;
  _durations_defaults = _durations;
  _offsets_defaults = _offsets;
  _offsetsF_defaults = _offsetsFloat;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name)
{
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

OffsetDurationGait::~OffsetDurationGait()
{
  delete[] _mpc_table;
}

MixedFrequncyGait::~MixedFrequncyGait()
{
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGait::getContactState()
{
  Array4f offset = _offsetsFloat;
  for (int i = 0; i < 4; i++)
  {
    if (offset[i] < 0)
      offset[i] += 1.f;
  }
  Array4f progress = _phase - offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.f;
    if (progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //  printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2],
  //  progress[3]);
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getContactState()
{
  Array4f progress = _phase;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
    {
      progress[i] += 1.;
    }

    if (progress[i] > _duty_cycle)
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  // printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2],
  // progress[3]);
  return progress.matrix();
}

Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;

  for (int i = 0; i < 4; i++)
  {
    if (swing_offset[i] > 1)
    {
      swing_offset[i] -= 1.;
    }
  }

  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
      progress[i] += 1.f;
    if (progress[i] >= swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  //  printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2],
  //  progress[3]);
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getSwingState()
{

  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
    {
      progress[i] = 0;
    }
    else
    {
      progress[i] = progress[i] / swing_duration;
    }
  }

  // printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2],
  // progress[3]);
  return progress.matrix();
}

int* OffsetDurationGait::getMpcTable()
{
  // printf("MPC table:\n");
  for (int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;

    for (int j = 0; j < 4; j++)
    {
      if (progress[j] < 0)
      {
        progress[j] += _nIterations;
      }
      if (progress[j] < _durations[j])
      {
        _mpc_table[i * 4 + j] = 1;
      }
      else
      {
        _mpc_table[i * 4 + j] = 0;
      }

      // printf("%d ", _mpc_table[i*4 + j]);
    }
    // printf("\n");
  }

  return _mpc_table;
}

int* MixedFrequncyGait::getMpcTable()
{
  // printf("MPC table (%d):\n", _iteration);
  for (int i = 0; i < _nIterations; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      int progress = (i + _iteration + 1) % _periods[j]; // progress
      if (progress < (_periods[j] * _duty_cycle))
      {
        _mpc_table[i * 4 + j] = 1;
      }
      else
      {
        _mpc_table[i * 4 + j] = 0;
      }
      // printf("%d %d (%d %d) | ", _mpc_table[i*4 + j], progress, _periods[j], (int)(_periods[j] *
      // _duty_cycle));
    }

    // printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", _mpc_table[i*4], _mpc_table[i*4 + 1],
    // _mpc_table[i*4 + ]) printf("\n");
  }
  return _mpc_table;
}

// currentIteration  - общий счетчик итераций
// iterationsPerMPC - итерации между МПС
void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  // Текущая итерация диапазон (0 - (_nIterations-1))
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  // Фаза походки
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float)(iterationsPerMPC * _nIterations);
  //  printf("phase: %.3f\n", _phase);
  //  printf("iteration: %d\n", _iteration);
}

void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsBetweenMPC); // % _nIterations;
  for (int i = 0; i < 4; i++)
  {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float)iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float)
    //(iterationsBetweenMPC * _periods[i]);
  }

  // printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);
}

int OffsetDurationGait::getCurrentGaitPhase()
{
  return _iteration;
}

int MixedFrequncyGait::getCurrentGaitPhase()
{
  return 0;
}

float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _swing;
}

float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg)
{
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _stance;
}

float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg)
{
  return dtMPC * _duty_cycle * _periods[leg];
}

void OffsetDurationGait::debugPrint()
{
}

void MixedFrequncyGait::debugPrint()
{
}

void OffsetDurationGait::earlyContactHandle(Vec4<uint8_t> footSensorState, int iterationsBetweenMPC, int currentIteration)
{
  for (long leg = 0; leg < 4; leg++)
  {
    // Если ранний контакт обнаружен в заключительной части свинг фазы
    if ((getSwingState()[leg] > 0.75f) && (footSensorState(leg) == 1))
    {
      // Уменьшить оффсет, увеличить duration на ту же величину
      float difference = _offsetsFloat(leg) - _phase < -0.001f ? _offsetsFloat(leg) - _phase + 1.0f : _offsetsFloat(leg) - _phase;

      _offsetsFloat(leg) -= difference * 1.0001;
      _durationsFloat(leg) += difference;

      _durations(leg) = int(_durationsFloat(leg) * float(_nIterations));
      _offsets(leg) = int(_offsetsFloat(leg) * float(_nIterations));
    }
    if ((getSwingState()[leg] < 0.25f) && (footSensorState(leg) == 1))
    {
    }
  }
}

void OffsetDurationGait::restoreDefaults()
{
  _durationsFloat = _durationsF_defaults;
  _durations = _durations_defaults;
  _offsets = _offsets_defaults;
  _offsetsFloat = _offsetsF_defaults;
}

void OffsetDurationGait::updatePeriod(int n)
{
  if (n == this->_nIterations || _phase != 0)
    return;

  _offsets = (float(n) * _offsetsFloat).cast<int>();
  _durations = (float(n) * _durationsFloat).cast<int>();
  std::cout << "offsets is " << _offsets(1) << " " << _offsets(2) << std::endl;
  std::cout << "duration is " << _durations(1) << " " << _durations(2) << std::endl;
  _nIterations = n;

  delete[] _mpc_table;

  _mpc_table = new int[_nIterations * 4];
  // offset и duration в диапазоне горизонта (0 -- 1)
  _durations_defaults = _durations;
  _offsets_defaults = _offsets;

  _stance = _durations[0];
  _swing = _nIterations - _durations[0];
}
