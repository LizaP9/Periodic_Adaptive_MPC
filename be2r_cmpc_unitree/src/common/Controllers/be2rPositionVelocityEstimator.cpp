#include "Controllers/be2rPositionVelocityEstimator.h"

using std::cout;
using std::endl;

template<typename T>
PositionEstimator<T>::PositionEstimator()
{
  time_start = ros::Time::now();
  acc_buffer.set_capacity(MOVING_AVERAGE);
  vel_buffer.set_capacity(MOVING_AVERAGE);

  for (size_t i = 0; i < MOVING_AVERAGE; i++)
  {
    acc_buffer.push_back(Vec3<T>(0, 0, 0));
    vel_buffer.push_back(Vec3<T>(0, 0, 0));
  }
  acc_z = 0;
}

/*!
 * Initialize the state estimator
 */
template<typename T>
void PositionEstimator<T>::setup()
{
  a_filtered << 0, 0, 0;
  a_filtered2 << 0, 0, 0;
  p_body << 0, 0, 0.056;
  v_body << 0, 0, 0;
}

/*!
 * Run state estimator
 */
template<typename T>
void PositionEstimator<T>::run()
{
  T dt = 0.002;
  float filter = 0.001;
  static uint16_t counter = 0;
  static float z = 0.056;
  static Vec3<float> vz_filtered(0, 0, 0);
  static Vec3<float> vz_filtered2(0, 0, 0);
  // float phase[4];

  Vec3<T> acceleration = this->_stateEstimatorData.result->aWorld;
  // phase[0] = this->_stateEstimatorData.result->contactEstimate(0);
  // phase[1] = this->_stateEstimatorData.result->contactEstimate(1);
  // phase[2] = this->_stateEstimatorData.result->contactEstimate(2);
  // phase[3] = this->_stateEstimatorData.result->contactEstimate(3);

  // float trust_window = 0.1;
  // float trust = 0;

  // for (size_t i = 0; i < 4; i++)
  // {
  //   if (phase[i] < trust_window)
  //   {
  //     trust += phase[i] / trust_window / 5;
  //   }
  //   else
  //   {
  //     trust += 0.2;
  //   }
  // }

  if (counter <= 500)
  {
    counter++;
    _offset += this->_stateEstimatorData.result->aWorld;
    _offset_vel += this->_stateEstimatorData.result->vWorld[2];
  }
  else
  {
    // filter = trust;
    acceleration -= _offset / 500;
    //    std::cout << "offset = " << _offset / 500 << std::endl;
    static ros::Time time;
    static ros::Time prev_time;
    time = ros::Time::now();
    acc_z = simpleKalman(acceleration(2), abs(time.toSec() - prev_time.toSec()), 1.0);
    prev_time = time;
    //    a_filtered2 = a_filtered2 * (1 - filter) + a_filtered * filter;
    //    for (size_t i = 0; i < 3; i++)
    //    {
    //      a_filtered2(i) = a_filtered(i) + std::pow((acceleration(i) - a_filtered(i)), 3) /
    //                                         (0.1 + std::pow((acceleration(i) - a_filtered(i)),
    //                                         2));
    //    }
    //    a_filtered = a_filtered2;

    //    vz_filtered = _filter(Vec3<float>(0, 0, this->_stateEstimatorData.result->vWorld[2]), 0);
    //    vz_filtered2 = vz_filtered2 * (1 - filter) + vz_filtered * filter;

    // v_body += acceleration * dt;
    v_body(2) += acc_z * dt;
    // v_body += a_filtered * dt;
    p_body(2) += v_body(2) * dt;

    // cout << "trust: " << trust << endl;
    // cout << "ac x: " << a_filtered[0] << " y: " << a_filtered[1] << " z: " << a_filtered[2] <<
    // endl; cout << "vel x: " << v_body[0] << " y: " << v_body[1] << " z: " << v_body[2] << endl;
    //    cout << "x: " << p_body[0] << " y: " << p_body[1] << " z: " << p_body[2] << endl;
    // cout << (ros::Time::now() - time_start).toSec() << endl;
    double vel = this->_stateEstimatorData.result->vWorld[2] - _offset_vel / 500;
    vel = simpleKalman(vel, dt, 0.1);
    z += vel * dt + (acc_z * dt * dt) * 0.5;
    // z += vz_filtered(2) * dt;
    //    std::cout << "z: " << z << std::endl;
    this->_stateEstimatorData.result->heightBody = z;
  }
}

template<typename T>
Vec3<T> PositionEstimator<T>::_filter(Vec3<T> acc, bool sw)
{
  Vec3<T> result(0, 0, 0);
  Vec3<T> sum(0, 0, 0);

  if (sw)
  {
    acc_buffer.push_back(acc);

    for (size_t i = 0; i < MOVING_AVERAGE; i++)
    {
      sum += acc_buffer.at(i);
    }
  }
  else
  {
    vel_buffer.push_back(acc);

    for (size_t i = 0; i < MOVING_AVERAGE; i++)
    {
      sum += vel_buffer.at(i);
    }
  }

  result = sum / (float)MOVING_AVERAGE;

  return result;
}

float simpleKalman(float newVal, double dt, double k)
{
  //  std::cout << " dt = " << dt << std::endl;
  float _err_measure = k; // примерный шум измерений
  float _q = dt; // скорость изменения значений 0.001-1, варьировать самому

  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;

  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =
    (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

template class PositionEstimator<float>;
