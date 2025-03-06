#pragma once

#include "Controllers/StateEstimatorContainer.h"
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <unitree_legged_msgs/BodyInfo.h>

#define MOVING_AVERAGE 4

template<typename T>
class PositionEstimator : public GenericEstimator<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PositionEstimator();
  virtual void run();
  virtual void setup();
  Vec3<T> _filter(Vec3<T> acc, bool sw);

  boost::circular_buffer<Vec3<T>> acc_buffer;
  boost::circular_buffer<Vec3<T>> vel_buffer;

private:
  Eigen::Matrix<T, 18, 1> _xhat;
  Eigen::Matrix<T, 12, 1> _ps;
  Eigen::Matrix<T, 12, 1> _vs;
  Eigen::Matrix<T, 18, 18> _A;
  Eigen::Matrix<T, 18, 18> _Q0;
  Eigen::Matrix<T, 18, 18> _P;
  Eigen::Matrix<T, 28, 28> _R0;
  Eigen::Matrix<T, 18, 3> _B;
  Eigen::Matrix<T, 28, 18> _C;
  Vec3<float> a_filtered;
  Vec3<float> a_filtered2;
  Vec3<float> p_body;
  Vec3<float> v_body;
  ros::Time time_start;
  float acc_z;
  Vec3<float> _offset;
  double _offset_vel;
};

float simpleKalman(float newVal, double dt, double k);
