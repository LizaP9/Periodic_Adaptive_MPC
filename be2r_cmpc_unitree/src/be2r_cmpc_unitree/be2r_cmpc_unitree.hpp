#pragma once

// C++
#include <cstdint>
#include <iostream>

// ROS
#include <Utilities/ros_read_param.h>
#include <be2r_cmpc_unitree/ros_dynamic_paramsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <std_msgs/Float32.h>

// MIT
#include "Configuration.h"
#include "ControlFSM.h"
#include "Controllers/StateEstimatorContainer.h"
#include "LegController.h"
#include "MiniCheetah.h"
// #include "Controllers/ContactEstimator.h"
#include "Controllers/ContactEstimator.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"

// BE2R
#include "Controllers/be2rPositionVelocityEstimator.h"
#include "debug.hpp"

// Unitree sdk
// namespace USDK
// {
#include "unitree_legged_sdk/unitree_legged_sdk.h"
// }
// using namespace USDK;

#define MAIN_LOOP_RATE 500

// #define FSM 3
// #define FSM_AUTO

#define MOTOR_BREAK 0x00
#define MOTOR_ON 0x0A

const float max_max_torque[3] = { 170.f, 170.f, 260.f }; // TODO CHECK WITH BEN
const float wimp_torque[3] = { 6.f, 6.f, 6.f };          // TODO CHECK WITH BEN
const float disabled_torque[3] = { 0.f, 0.f, 0.f };

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Body_Manager
{
public:
  Body_Manager();
  ~Body_Manager();

  void init();
  void run();
  void setupStep();
  void finalizeStep();
  void initializeStateEstimator();

  VectorNavData vectorNavData;
  SpiData spiData;
  SpiCommand spiCommand;
  u64 _iterations = 0;
  Vec4<uint8_t> footContactState;

  bool is_stand = false;

  Timer t1;

  void UDPRecv();
  void UDPSend();
  bool is_udp_connection = false;

private:
  ros::NodeHandle _nh;

  ros::Publisher _pub_low_cmd;
  ros::Publisher _pub_low_state;
  ros::Subscriber _sub_ground_truth;
  ros::Subscriber _sub_low_state;
  ros::Subscriber _sub_cmd_vel;
  ros::ServiceServer _srv_do_step;
  ros::ServiceServer _srv_stop_map;
  ros::ServiceServer _srv_start_map;
  ros::Time _time_start;
  const ros::Time _zero_time;
  bool _is_param_updated = false;
  bool _is_do_step = false;
  float _do_step_vel = 0;

  dynamic_reconfigure::Server<be2r_cmpc_unitree::ros_dynamic_paramsConfig> server;
  dynamic_reconfigure::Server<be2r_cmpc_unitree::ros_dynamic_paramsConfig>::CallbackType f;

  void _initSubscribers();
  void _initPublishers();
  void _filterInput();
  void _initParameters();
  void _odomPublish();

  void _lowStateCallback(unitree_legged_msgs::LowState msg);
  void _cmdVelCallback(geometry_msgs::Twist msg);
  void _torqueCalculator(SpiCommand* cmd, SpiData* data, int leg_num);
  void _callbackDynamicROSParam(be2r_cmpc_unitree::ros_dynamic_paramsConfig& config, uint32_t level);
  void _groundTruthCallback(nav_msgs::Odometry ground_truth_msg);
  bool _srvDoStep(std_srvs::Trigger::Request& reqest, std_srvs::Trigger::Response& response);
  bool _srvStopMap(std_srvs::Empty::Request& reqest, std_srvs::Empty::Response& response);
  bool _srvStartMap(std_srvs::Empty::Request& reqest, std_srvs::Empty::Response& response);

  // Unitree sdk
  UNITREE_LEGGED_SDK::Safety safe;
  UNITREE_LEGGED_SDK::UDP* udp;
  void _readRobotData();
  UNITREE_LEGGED_SDK::LowCmd _udp_low_cmd = {};
  UNITREE_LEGGED_SDK::LowState _udp_low_state = {};
  nav_msgs::Odometry ground_truth = {};
  UNITREE_LEGGED_SDK::LowCmd _rosCmdToUdp(unitree_legged_msgs::LowCmd ros_low_cmd);
  unitree_legged_msgs::LowState _udpStateToRos(UNITREE_LEGGED_SDK::LowState udp_low_state);

  ControlFSM<float>* _controlFSM;
  Quadruped<float> _quadruped;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float> _stateEstimate;
  CheaterState<float> _cheater_state;
  Debug* _debug = nullptr;
  GamepadCommand* _gamepad_command = nullptr;
  unitree_legged_msgs::LowState _low_state;
  unitree_legged_msgs::LowCmd _low_cmd;
  bool _is_low_level = false;
  int _power_limit = 0;
  bool _is_torque_safe = true;
  string _robot_type = "a1";

  // Gait Scheduler controls the nominal contact schedule for the feet
  GaitScheduler<float>* _gaitScheduler;
  be2r_cmpc_unitree::ros_dynamic_paramsConfig _rosParameters;
  StaticParams _rosStaticParams;
};

extern float simulation_time;  // Объявляем глобальную переменную
void simulationTimeCallback(const std_msgs::Float32::ConstPtr& msg);  // Прототип функции

