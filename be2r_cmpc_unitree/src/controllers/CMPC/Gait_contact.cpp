#include "Gait_contact.h"

// Offset - Duration Gait
OffsetDurationGaitContact::OffsetDurationGaitContact(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name)
  : _offsets(offsets.array()),
    _durations(durations.array()),
    _nIterations(nSegment)
{
  _name = name;
  // allocate memory for MPC gait table
  _mpc_table = new int[nSegment * 4];

  // ROS_INFO_STREAM(nSegment << " gait period");
  std::cout << name << " gait period: " << nSegment << std::endl;

  // nSegment - количество сегментов мпс = gait period

  // offset и duration stance в диапазоне (0 -- 1)
  _offsetsFloat = offsets.cast<float>() / (float)nSegment;
  _durationsFloat = durations.cast<float>() / (float)nSegment;
  _durationsF_defaults = _durationsFloat;
  _durations_defaults = _durations;
  _offsets_defaults = _offsets;
  _offsetsF_defaults = _offsetsFloat;

  _stance = durations[0];
  _swing = nSegment - durations[0];
}

OffsetDurationGaitContact::~OffsetDurationGaitContact()
{
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGaitContact::getContactState()
{
  Array4f offset = _offsetsFloat;

  for (int i = 0; i < 4; i++)
  {
    if (offset[i] < 0)
    {
      offset[i] += 1.f;
    }
  }

  Array4f progress = _phase - offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
    {
      progress[i] += 1.f;
    }
    if (progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  return progress.matrix();
}

Vec4<float> OffsetDurationGaitContact::getSwingState()
{
  Array4f swing_offset;
  Array4f swing_duration;
  Array4f progress;

  swing_offset = _offsetsFloat + _durationsFloat;

  for (int i = 0; i < 4; i++)
  {
    if (swing_offset[i] > 1)
    {
      swing_offset[i] -= 1.;
    }
  }

  swing_duration = 1. - _durationsFloat;

  progress = _phase - swing_offset;

  for (int i = 0; i < 4; i++)
  {
    if (progress[i] < 0)
    {
      progress[i] += 1.f;
    }
    if (progress[i] >= swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / swing_duration[i];
    }
  }

  for (size_t i = 0; i < 4; i++)
  {
    if (is_contact[i])
    {
      delta_t[i] = swing_duration[i] - progress[i];
      progress[i] = 0;
    }
  }

  return progress.matrix();
}

int* OffsetDurationGaitContact::getMpcTable()
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

// currentIteration  - общий счетчик итераций
// iterationsPerMPC - итерации между МПС
void OffsetDurationGaitContact::setIterations(int iterationsPerMPC, int currentIteration)
{
  // Текущая итерация диапазон (0 - (_nIterations-1))
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  // Фаза походки
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float)(iterationsPerMPC * _nIterations);
}

int OffsetDurationGaitContact::getCurrentGaitIteration()
{
  return _iteration;
}

float OffsetDurationGaitContact::getCurrentGaitPhase()
{
  return _phase;
}

float OffsetDurationGaitContact::getCurrentSwingTime(float dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _swing;
}

float OffsetDurationGaitContact::getCurrentStanceTime(float dtMPC, int leg)
{
  (void)leg;
  return dtMPC * _stance;
}

void OffsetDurationGaitContact::earlyContactHandle(Vec4<uint8_t> footSensorState, int iterationsBetweenMPC, int currentIteration)
{
  // std::cout << "sensor 0: " << (int)footSensorState(0) << std::endl;

  for (long leg = 0; leg < 4; leg++)
  {
    // Если ранний контакт обнаружен в заключительной части свинг фазы
    if ((getSwingState()[leg] > 0.65f) && (footSensorState(leg) == 1))
    {
      // Уменьшить оффсет, увеличить duration на ту же величину
      // std::cout << "SWING STATE before" << getSwingState()[leg] << std::endl;
      float difference = _offsetsFloat(leg) - _phase < -0.001f ? _offsetsFloat(leg) - _phase + 1.0f : _offsetsFloat(leg) - _phase;
      // std::cout << "difference in phase " << difference << std::endl;
      // std::cout << "phase" << _phase << std::endl;
      // std::cout << "offset [ " << leg << "] " << _offsetsFloat(leg) << std::endl;
      // std::cout << "duration before [ " << leg << "] " << _durationsFloat(leg) << std::endl;
      _offsetsFloat(leg) -= difference * 1.0001;
      //      _offsets(leg) = int(_offsetsFloat(leg)) * _nIterations;
      //      if (_offsetsFloat(leg) == 0.5f)
      //        _durationsFloat(leg) += difference;
      //      if (_offsetsFloat(leg) <= 0.01f)
      _durationsFloat(leg) += difference;

      _durations(leg) = int(_durationsFloat(leg) * float(_nIterations));
      _offsets(leg) = int(_offsetsFloat(leg) * float(_nIterations));
      //      std::cout << "_durations(leg)" << _durations(leg) << std::endl;
      // std::cout << "offset after [ " << leg << "] " << _offsetsFloat(leg) << std::endl;
      // std::cout << "duration after [ " << leg << "] " << _durationsFloat(leg) << std::endl;
      // std::cout << "SWING STATE after" << getSwingState()[leg] << std::endl;
      // std::cout << "STANCE STATE after" << getContactState()[leg] << std::endl;
    }
    if ((getSwingState()[leg] < 0.25f) && (footSensorState(leg) == 1))
    {
    }
  }
}

void OffsetDurationGaitContact::restoreDefaults()
{
  _durationsFloat = _durationsF_defaults;
  _durations = _durations_defaults;
  _offsets = _offsets_defaults;
  _offsetsFloat = _offsetsF_defaults;
}

void OffsetDurationGaitContact::updatePeriod(int n)
{
  if (n == this->_nIterations || _phase != 0)
  {
    return;
  }

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
  // _durationsF_defaults
  // _offsetsF_defaults

  _stance = _durations[0];
  _swing = _nIterations - _durations[0];
}