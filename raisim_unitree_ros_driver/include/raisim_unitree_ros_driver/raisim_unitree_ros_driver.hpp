#pragma once

// ROS
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <tf/tf.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>

// CPP
#include <math.h>

#include <iostream>

// RaiSim
#include "raisim/RaisimServer.hpp"

#include "path_list.h"

#define SIM_FREQ 500
#define RTF 1.0f //real time factor
#define SIM_DT 1.0f / (float)SIM_FREQ* RTF
#define INIT_TIME 1.0

#define TORQUE_LIMIT 50.0

#define MOTOR_BREAK 0x00
#define MOTOR_ON 0x0A

enum SPAWN_POSITION
{
  LAY_POSITION,
  SIT_POSITION,
  STAND_POSITION
};

#define START_POSITION LAY_POSITION
// #define START_POSITION STAND_POSITIONC

class Unitree
{
public:
  Unitree(double dt);
  ~Unitree();

  bool init();
  void updateWorld();
  void updateWorldOnly();
  void forceOn();
  void checkContact();

  Eigen::VectorXd jointTargetPosition;
  Eigen::VectorXd jointTargetVelocity;
  Eigen::VectorXd jointTargetEffort;

  Eigen::VectorXd actPos;
  Eigen::VectorXd actVel;
  Eigen::VectorXd prevVel;
  Eigen::VectorXd actAcc;

  double motor_data[24];

private:
  //шаблон для чтения ROS параметров
  template <typename T>
  bool readParam(std::string param_name, T& param_var)
  {
    if (!_nh.getParam(param_name, param_var))
    {
      ROS_WARN_STREAM("Can't read param " << param_name);
      return false;
    }

    std::cout << param_name << ": " << param_var << std::endl;

    return true;
  }


  raisim::ArticulatedSystem* _robot;
  raisim::World* _world;
  raisim::RaisimServer* _server;
  bool _is_air_fixed = false;
  std::string _robot_type = "a1";

  bool _spawnRobot();
  void _setSpawnPosition(uint8_t spawn_pos);
  void _zero();

  void _lowCmdCallback(unitree_legged_msgs::LowCmd msg);

  void _computeBodyAcceleration();
  void _updateFeedBack();
  void _computeImu();
  void _createStairs(uint8_t num_of_steps, float width, float depth, float height, Eigen::Vector3f p_spawn);

  ros::NodeHandle _nh;
  ros::Time _time_start;
  const ros::Time _zero_time;

  ros::Subscriber _sub_low_cmd;
  ros::Publisher _pub_low_state;
  ros::Publisher _pub_ground_truth;
  ros::Publisher _pub_rpy;
  nav_msgs::Odometry _ground_truth_odom;
  // ros::ServiceServer _srv_restart;
  // bool _srvRestart(std_srvs::Trigger::Request& reqest, std_srvs::Trigger::Response& response);

  unitree_legged_msgs::LowCmd _low_cmd;
  unitree_legged_msgs::LowState _low_state;

  ros::Publisher _pub_external_force;
  ros::Publisher _pub_sim_time;

  double _dt;

  // Параметры внешней силы
  float d_s = -10.;
  float d_n = 15.0; // Амплитуда силы (Н)
  float externalForceFrequency = 0.33;  // Частота (Гц)
  float externalForcePhase = 0.0;      // Начальная фаза
  Eigen::Vector3d forceApplicationPoint = Eigen::Vector3d(0.0, 0.0, 0.0); // Точка на спине робота

  void applyExternalForce();


};

Unitree* unitree;
