#include "raisim_unitree_ros_driver.hpp"
#include "ros/time.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include <Eigen/src/Core/Matrix.h>
#include "std_msgs/Float32.h"

using namespace std;

Unitree::Unitree(double dt)
  : jointTargetPosition(19),
    jointTargetVelocity(18),
    jointTargetEffort(18),
    actPos(19),
    actVel(18),
    prevVel(18),
    actAcc(18),
    _zero_time(0)
{
  _dt = dt;

  _sub_low_cmd = _nh.subscribe("/low_cmd", 1, &Unitree::_lowCmdCallback, this, ros::TransportHints().tcpNoDelay(true));
  _pub_low_state = _nh.advertise<unitree_legged_msgs::LowState>("/low_state", 1);
  _pub_rpy = _nh.advertise<geometry_msgs::PointStamped>("/ground_truth_rpy", 1);
  _pub_ground_truth = _nh.advertise<nav_msgs::Odometry>("/ground_truth_odom", 1);
  // _srv_restart = _nh.advertiseService("restart_sim", &Unitree::_srvRestart, this);

  _pub_external_force = _nh.advertise<std_msgs::Float32>("/external_forces", 1);

  _pub_sim_time = _nh.advertise<std_msgs::Float32>("/sim_time", 1);


}

bool Unitree::init()
{
  _world = new raisim::World;
  _server = new raisim::RaisimServer(_world);

  _zero();
  jointTargetPosition(3) = 1.0;

  _time_start = ros::Time::now();

  readParam(ros::this_node::getName() + "/air_fixed", _is_air_fixed);
  readParam(ros::this_node::getName() + "/robot_type", _robot_type);

  return _spawnRobot();
}

void Unitree::_zero()
{
  jointTargetPosition.setZero();
  jointTargetVelocity.setZero();
  jointTargetEffort.setZero();

  actPos.setZero();
  actVel.setZero();
  prevVel.setZero();
  actAcc.setZero();
}

// bool Unitree::_srvRestart(std_srvs::Trigger::Request& reqest,
//                           std_srvs::Trigger::Response& response)
// {
//   ROS_WARN("RESTART!");

//   _server->killServer();
//   _server->~RaisimServer();
//   _world->~World();

//   // world = new raisim::World;
//   // server = new raisim::RaisimServer(world);

//   // _world = world;
//   // _server = server;

//   _init();

//   double start_time = 0;
//   double deltaT = 0;
//   start_time = ros::Time::now().toSec();

//   deltaT = ros::Time::now().toSec() - start_time;

//   if (deltaT < INIT_TIME)
//   {
//     ROS_INFO_STREAM_ONCE("Wait: " << INIT_TIME << " seconds");
//     unitree->updateWorldOnly();
//   }

//   return true;
// }

void Unitree::_computeBodyAcceleration()
{
  if (_is_air_fixed)
  {
    for (size_t i = 0; i < 3; i++)
    {
      actAcc(i) = 0;
      prevVel(i) = actVel(i);
    }
  }
  else
  {
    for (size_t i = 0; i < 3; i++)
    {
      actAcc(i) = (actVel(i) - prevVel(i)) / _dt;
      prevVel(i) = actVel(i);
    }
  }
}

void Unitree::_lowCmdCallback(unitree_legged_msgs::LowCmd msg)
{
  _low_cmd = msg;

  for (uint8_t joint = 0; joint < 12; joint++)
  {
    if (_low_cmd.motorCmd[joint].tau > TORQUE_LIMIT)
    {
      _low_cmd.motorCmd[joint].tau = TORQUE_LIMIT;
    }

    if (_low_cmd.motorCmd[joint].tau < -TORQUE_LIMIT)
    {
      _low_cmd.motorCmd[joint].tau = -TORQUE_LIMIT;
    }
  }
}

void Unitree::_setSpawnPosition(uint8_t spawn_pos)
{
  switch (spawn_pos)
  {
    case LAY_POSITION:
      jointTargetPosition.tail(12)[0] = -0.53;
      jointTargetPosition.tail(12)[1] = 1.07;
      jointTargetPosition.tail(12)[2] = -2.63;

      jointTargetPosition.tail(12)[3] = 0.53;
      jointTargetPosition.tail(12)[4] = 1.07;
      jointTargetPosition.tail(12)[5] = -2.63;

      jointTargetPosition.tail(12)[6] = -0.53;
      jointTargetPosition.tail(12)[7] = 1.07;
      jointTargetPosition.tail(12)[8] = -2.63;

      jointTargetPosition.tail(12)[9] = 0.53;
      jointTargetPosition.tail(12)[10] = 1.07;
      jointTargetPosition.tail(12)[11] = -2.63;
      break;

    case SIT_POSITION:
      jointTargetPosition.tail(12)[0] = 0;
      jointTargetPosition.tail(12)[1] = 0.785;
      jointTargetPosition.tail(12)[2] = -0.785;

      jointTargetPosition.tail(12)[3] = 0;
      jointTargetPosition.tail(12)[4] = 0.785;
      jointTargetPosition.tail(12)[5] = -0.785;

      jointTargetPosition.tail(12)[6] = 0;
      jointTargetPosition.tail(12)[7] = 0.785;
      jointTargetPosition.tail(12)[8] = -0.785;

      jointTargetPosition.tail(12)[9] = 0;
      jointTargetPosition.tail(12)[10] = 0.785;
      jointTargetPosition.tail(12)[18] = -0.785;
      break;

    case STAND_POSITION:
      jointTargetPosition.tail(12)[0] = 0.06;
      jointTargetPosition.tail(12)[1] = 0.6;
      jointTargetPosition.tail(12)[2] = -1.2;

      jointTargetPosition.tail(12)[3] = -0.06;
      jointTargetPosition.tail(12)[4] = 0.6;
      jointTargetPosition.tail(12)[5] = -1.2;

      jointTargetPosition.tail(12)[6] = 0.06;
      jointTargetPosition.tail(12)[7] = 0.6;
      jointTargetPosition.tail(12)[8] = -1.2;

      jointTargetPosition.tail(12)[9] = -0.06;
      jointTargetPosition.tail(12)[10] = 0.6;
      jointTargetPosition.tail(12)[11] = -1.2;
      break;
  }
}

bool Unitree::_spawnRobot()
{
  _world->setTimeStep(_dt);
  _world->addGround();

  if (_is_air_fixed)
  {
    _robot = _world->addArticulatedSystem(PAKAGE_PATH "../a1_description/urdf/a1_edited_air_fixed.urdf");

    jointTargetPosition.resize(_robot->getDOF());
    jointTargetVelocity.resize(_robot->getDOF());
    jointTargetEffort.resize(_robot->getDOF());
    actPos.resize(_robot->getDOF());
    actVel.resize(_robot->getDOF());
    prevVel.resize(_robot->getDOF());
    actAcc.resize(_robot->getDOF());
    _zero();

    _setSpawnPosition(STAND_POSITION);
  }
  else
  {
    if (_robot_type == "a1")
    {
      _robot = _world->addArticulatedSystem(PAKAGE_PATH "../a1_description/urdf/a1_edited.urdf");
      _robot->setName("unitree_a1");
    }
    else if (_robot_type == "go1")
    {
      _robot = _world->addArticulatedSystem(PAKAGE_PATH "../go1_description/urdf/go1_edited.urdf");
      _robot->setName("unitree_go1");
    }
    else
    {
      ROS_ERROR("Wrong robot type!");
      return 1;
    }

    // body spawn coordinates
    jointTargetPosition[0] = 0.0;  // root x
    jointTargetPosition[1] = 0.0;  // root y
    jointTargetPosition[2] = 0.35; // root z
    jointTargetPosition[3] = 1.0;  // W of quaternion

    _setSpawnPosition(START_POSITION);
  }

  cout << "Robot DOF: " << _robot->getDOF() << endl;

  Eigen::VectorXd jointPgain(_robot->getDOF());
  Eigen::VectorXd jointDgain(_robot->getDOF());
  jointPgain.tail(12).setConstant(100.0);
  jointDgain.tail(12).setConstant(10.0);

  _robot->setGeneralizedCoordinate(jointTargetPosition);
  _robot->setGeneralizedVelocity(jointTargetVelocity);
  _robot->setGeneralizedForce(Eigen::VectorXd::Zero(_robot->getDOF()));
  _robot->setPdGains(jointPgain, jointDgain);
  _robot->setPdTarget(jointTargetPosition, jointTargetVelocity);
  _robot->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

  // depth width height
  // Eigen::Vector3f stairs_params(0.2, 2, 0.14);
  // Eigen::Vector3f stairs_params(0.2, 2, 0.1);
  // Eigen::Vector3f stairs_pos(1, 0, stairs_params(2) / 2);

  // _createStairs(2, stairs_params(1), stairs_params(0), stairs_params(2), stairs_pos);

  // Eigen::Vector3f box_size(1, 2, 0.07);
  // Eigen::Vector3f box_pos(1, 0, box_size(2) / 2);
  // auto block_2 = _world->addBox(box_size(0), box_size(1), box_size(2), 0);
  // block_2->setName("block_2");
  // block_2->setBodyType(raisim::BodyType::STATIC);
  // // block_2->setPosition(0, -2, box_size(1) * sin(M_PI_2 / 3) / 2 - 0.05);
  // // block_2->setOrientation(0.966, -0.259, 0.0, 0.0); //30 deg, slipping
  // block_2->setPosition(0, -2, box_size(1) * sin(M_PI_2 / 6) / 2 - 0.05);
  // block_2->setOrientation(0.991, -0.131, 0.0, 0.0); // 15 deg 0.26 rad, slipping
  // block_2->setAppearance("1.0, 1.0, 1.0, 1.0");

  _server->launchServer();
  //_server->focusOn(_robot);

  return 0;
}

void Unitree::forceOn() { _robot->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE); }

void Unitree::updateWorldOnly()
{
  _server->integrateWorldThreadSafe();
  _robot->setPdTarget(jointTargetPosition, jointTargetVelocity);

  _robot->getState(actPos, actVel);

  for (uint8_t servo_num = 0; servo_num < 12; servo_num++)
  {
    _low_state.motorState[servo_num].q = actPos.tail(12)[servo_num];
    _low_state.motorState[servo_num].dq = actVel.tail(12)[servo_num];
  }

  // compute body acceleration in world frame
  _computeBodyAcceleration();

  _computeImu();

  _pub_low_state.publish(_low_state);
}

float randomFloat()
{
  float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);

  return r;
}

void Unitree::updateWorld()
{
  //// МЕСТО ВЫЗОВА ИЗДАТЕЛЯ И ПОДАЧА СООБЩЕНИЙ В ТОПИК /external_forces

  // update world
  _server->integrateWorldThreadSafe();

  float sim_time = (float) _world->getWorldTime(); // Время симуляции
  std_msgs::Float32 msg;
  msg.data = sim_time;
  _pub_sim_time.publish(msg);
  //std::cout << "Real:\n" << sim_time << "\n";

  // Применяем внешнюю силу
  applyExternalForce();

  // get pos and vel, compute acc from vel and prev vel
  _robot->getState(actPos, actVel);

//_server->getVisualObject("camera")->setPosition(actPos[0] + 1.5, actPos[1], actPos[2] + 1.0);

  Eigen::Vector3d camera_offset(0.0, -2.0, 0.5);
  Eigen::Vector3d camera_position = actPos.head(3) + camera_offset;

  // Камера всегда смотрит в точку робота
  Eigen::Vector3d look_at_position = actPos.head(3);

  // Фиксируем камеру относительно робота
  _server->setCameraPositionAndLookAt(camera_position, look_at_position);



  float noize_q = (randomFloat() + randomFloat() + randomFloat()) / 3.0 - 0.5;
  noize_q *= 0.01 * 2.0;
  float noize_dq = (randomFloat() + randomFloat() + randomFloat()) / 3.0 - 0.5;
  noize_dq *= 0.1 * 2.0;
  // cout << "noize: " << noize << endl;

  for (uint8_t servo_num = 0; servo_num < 12; servo_num++)
  {
    _low_state.motorState[servo_num].q = actPos.tail(12)(servo_num);
    _low_state.motorState[servo_num].dq = actVel.tail(12)(servo_num);
    // _low_state.motorState[servo_num].q = actPos.tail(12)(servo_num) + noize_q;
    // _low_state.motorState[servo_num].dq = actVel.tail(12)(servo_num) + noize_dq;
  }

  _ground_truth_odom.header.stamp = ros::Time::now();
  _ground_truth_odom.pose.pose.position.x = actPos(0);
  _ground_truth_odom.pose.pose.position.y = actPos(1);
  _ground_truth_odom.pose.pose.position.z = actPos(2);
  _ground_truth_odom.pose.pose.orientation.w = actPos(3);
  _ground_truth_odom.pose.pose.orientation.x = actPos(4);
  _ground_truth_odom.pose.pose.orientation.y = actPos(5);
  _ground_truth_odom.pose.pose.orientation.z = actPos(6);

  _ground_truth_odom.twist.twist.linear.x = actVel(0);
  _ground_truth_odom.twist.twist.linear.y = actVel(1);
  _ground_truth_odom.twist.twist.linear.z = actVel(2);
  _ground_truth_odom.twist.twist.angular.x = actVel(3);
  _ground_truth_odom.twist.twist.angular.y = actVel(4);
  _ground_truth_odom.twist.twist.angular.z = actVel(5);

  geometry_msgs::PointStamped _rpy;

  geometry_msgs::Quaternion tf_quat;
  tf_quat.x = _ground_truth_odom.pose.pose.orientation.x;
  tf_quat.y = _ground_truth_odom.pose.pose.orientation.y;
  tf_quat.z = _ground_truth_odom.pose.pose.orientation.z;
  tf_quat.w = _ground_truth_odom.pose.pose.orientation.w;

  tfScalar pitch, roll, yaw;
  tfScalar x = _ground_truth_odom.pose.pose.orientation.x;
  tfScalar y = _ground_truth_odom.pose.pose.orientation.y;
  tfScalar z = _ground_truth_odom.pose.pose.orientation.z;
  tfScalar w = _ground_truth_odom.pose.pose.orientation.w;

  tf::Quaternion quat;
  quat.setX(x);
  quat.setY(y);
  quat.setZ(z);
  quat.setW(w);

  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  _rpy.point.x = roll;
  _rpy.point.y = pitch;
  _rpy.point.z = yaw;

  _rpy.header.stamp = ros::Time::now();

  _pub_rpy.publish(_rpy);

  _pub_ground_truth.publish(_ground_truth_odom);

  // compute body acceleration in world frame
  _computeBodyAcceleration();

  _computeImu();

  _low_state.header.stamp = ros::Time::now();

  _pub_low_state.publish(_low_state);

  jointTargetEffort.setZero();

  // Eigen::VectorXd jointPgain(_robot->getDOF());
  // Eigen::VectorXd jointDgain(_robot->getDOF());

  for (size_t i = 0; i < 12; i++)
  {
    if (_low_cmd.motorCmd[i].mode == MOTOR_ON)
    {
      jointTargetEffort.tail(12)(i) = _low_cmd.motorCmd[i].tau;
      // jointTargetPosition.tail(12)(i) = _low_cmd.motorCmd[i].q;
      // jointTargetVelocity.tail(12)(i) = _low_cmd.motorCmd[i].dq;
      // jointPgain.tail(12)(i) = _low_cmd.motorCmd[i].Kp;
      // jointDgain.tail(12)(i) = _low_cmd.motorCmd[i].Kd;
    }
    else if (_low_cmd.motorCmd[i].mode == MOTOR_BREAK)
    {
      jointTargetEffort.tail(12)(i) = 0;
      // jointDgain.tail(12).setConstant(0);
      // jointDgain.tail(12).setConstant(0);
    }
    else
    {
      ROS_ERROR("Wrong motor mode!");
    }

    // _robot->setPdGains(jointPgain, jointDgain);
  }

  _robot->setGeneralizedForce(jointTargetEffort);

  // cout << "COM x: " << _robot->getBodyCOM_B().at(0) << endl;
  // cout << "COM y: " << _robot->getBodyCOM_B().at(1) << endl;
  // cout << "COM z: " << _robot->getBodyCOM_B().at(2) << endl;

  checkContact();

  auto M1 = _robot->getMassMatrix();
  auto CG1 = _robot->getNonlinearities(_world->getGravity());

  static Eigen::Vector3d q(0, 0, 0);
  static Eigen::Vector3d dq(0, 0, 0);
  static Eigen::Vector3d dq_true(0, 0, 0);
  static Eigen::Vector3d ddq(0, 0, 0);
  static double dt = 0.002;

  uint8_t i_start = 0;
  Eigen::Matrix3d M = M1.e().block(i_start, i_start, 3, 3);
  Eigen::Vector3d CG = CG1.e().block(i_start, 0, 3, 1);
  Eigen::Vector3d tau(0, 0, 0);
  tau << _low_cmd.motorCmd[0].tau, _low_cmd.motorCmd[1].tau, _low_cmd.motorCmd[2].tau;
  dq_true << _low_state.motorState[0].dq, _low_state.motorState[1].dq, _low_state.motorState[2].dq;

  // // cout << "C: " << C << endl;
  // cout << "G: " << G << endl;
  if (abs(tau[0]) > 0)
  {
    ddq = M.inverse() * (tau - CG);
    dq = dq + ddq * dt;

    // cout << "tau: " << tau << endl;
    // cout << "Mass Matrix: " << M << endl;
    // cout << "Mass Matrix1: " << M1 << endl;
    // cout << "C+G: " << CG << endl;
    // cout << "C+G1: " << CG1 << endl;
    // cout << "ddq: " << ddq << endl;
    // cout << "dq: " << dq << endl;
    // cout << "dq true: " << dq_true << endl;
  }
}

void Unitree::_computeImu()
{
  if (_is_air_fixed)
  {
    actAcc(0) = 0;
    actAcc(1) = 0;
    actAcc(2) = 9.81;

    _low_state.imu.accelerometer[0] = actAcc(0);
    _low_state.imu.accelerometer[1] = actAcc(1);
    _low_state.imu.accelerometer[2] = actAcc(2);

    _low_state.imu.quaternion[0] = 0;
    _low_state.imu.quaternion[1] = 0;
    _low_state.imu.quaternion[2] = 0;
    _low_state.imu.quaternion[3] = 0;

    _low_state.imu.gyroscope[0] = 0;
    _low_state.imu.gyroscope[1] = 0;
    _low_state.imu.gyroscope[2] = 0;
  }
  else
  {
    // compute imu in body frame from world frame
    tf::Matrix3x3 rot_mat(tf::Quaternion(actPos(4), actPos(5), actPos(6), actPos(3)));
    actAcc(2) += 9.81;

    // TODO Matrix multiplication with Eigen
    _low_state.imu.accelerometer[0] = rot_mat[0][0] * actAcc(0) + rot_mat[1][0] * actAcc(1) + rot_mat[2][0] * actAcc(2);
    _low_state.imu.accelerometer[1] = rot_mat[0][1] * actAcc(0) + rot_mat[1][1] * actAcc(1) + rot_mat[2][1] * actAcc(2);
    _low_state.imu.accelerometer[2] = rot_mat[0][2] * actAcc(0) + rot_mat[1][2] * actAcc(1) + rot_mat[2][2] * actAcc(2);
    _low_state.imu.quaternion[0] = actPos(3); // w
    _low_state.imu.quaternion[1] = actPos(4); // x
    _low_state.imu.quaternion[2] = actPos(5); // y
    _low_state.imu.quaternion[3] = actPos(6); // z
    _low_state.imu.gyroscope[0] = rot_mat[0][0] * actVel(3) + rot_mat[1][0] * actVel(4) + rot_mat[2][0] * actVel(5);
    _low_state.imu.gyroscope[1] = rot_mat[0][1] * actVel(3) + rot_mat[1][1] * actVel(4) + rot_mat[2][1] * actVel(5);
    _low_state.imu.gyroscope[2] = rot_mat[0][2] * actVel(3) + rot_mat[1][2] * actVel(4) + rot_mat[2][2] * actVel(5);
  }
}

void Unitree::checkContact()
{
  auto fr_calf = _robot->getBodyIdx("FR_calf");
  auto fl_calf = _robot->getBodyIdx("FL_calf");
  auto rr_calf = _robot->getBodyIdx("RR_calf");
  auto rl_calf = _robot->getBodyIdx("RL_calf");

  bool is_contact[4] = { false, false, false, false };

  static uint8_t contact_ticks[4] = { 0 };
  static uint8_t max_ticks = 10;

  // for all contacts on the robot, check ...
  for (auto& contact : _robot->getContacts())
  {
    if (contact.skip())
    {
      continue; // if the contact is internal, one contact point is set to 'skip'
    }

    if (fr_calf == contact.getlocalBodyIndex())
    {
      is_contact[0] = true;
    }

    if (fl_calf == contact.getlocalBodyIndex())
    {
      is_contact[1] = true;
    }

    if (rr_calf == contact.getlocalBodyIndex())
    {
      is_contact[2] = true;
    }

    if (rl_calf == contact.getlocalBodyIndex())
    {
      is_contact[3] = true;
    }
  }

  _low_state.footForce[0] = is_contact[0];
  _low_state.footForce[1] = is_contact[1];
  _low_state.footForce[2] = is_contact[2];
  _low_state.footForce[3] = is_contact[3];
}

void Unitree::_createStairs(uint8_t num_of_steps, float width, float depth, float height, Eigen::Vector3f p_spawn)
{
  vector<raisim::Box*> stair_step;

  for (size_t i = 0; i < num_of_steps; i++)
  {
    stair_step.push_back(_world->addBox(depth, width, height, 0));
    stair_step.at(i)->setName("step_" + std::to_string(i));
    stair_step.at(i)->setBodyType(raisim::BodyType::STATIC);
    stair_step.at(i)->setPosition(p_spawn(0) + depth * i, p_spawn(1), p_spawn(2) + height * i);
    stair_step.at(i)->setOrientation(1.0, 0.0, 0.0, 0.0);
    stair_step.at(i)->setAppearance("1.0, 1.0, 1.0, 1.0");
  }

  double k = 6;
  stair_step.push_back(_world->addBox(depth * k, width, height, 0));
  stair_step.at(num_of_steps)->setName("step_" + std::to_string(num_of_steps));
  stair_step.at(num_of_steps)->setBodyType(raisim::BodyType::STATIC);
  stair_step.at(num_of_steps)->setPosition(p_spawn(0) + depth * (num_of_steps - 1) + depth * k * 0.5 + depth / 2, p_spawn(1), p_spawn(2) + height * num_of_steps);
  stair_step.at(num_of_steps)->setOrientation(1.0, 0.0, 0.0, 0.0);
  stair_step.at(num_of_steps)->setAppearance("1.0, 1.0, 1.0, 1.0");
}

Unitree::~Unitree()
{
  _server->killServer();
  _server->~RaisimServer();
  _world->~World();
}

void Unitree::applyExternalForce()
{
    double time = _world->getWorldTime(); // Время симуляции

    // Вычисляем силу по синусоидальному закону
    double forceValue = d_s + d_n * sin(2 * M_PI * externalForceFrequency * time + externalForcePhase);
    Eigen::Vector3d externalForce(forceValue, 0.0, 0.0); // Сила действует вдоль оси X

    //Eigen::Vector3d externalForce(15.0, 0.0, 0.0); 
    //Eigen::Vector3d ExternalTorque(5.0, 0.0, 0.0); 
    // Определяем индекс тела, к которому будем прикладывать силу (спина робота)
    int baseIdx = _robot->getBodyIdx("base");
    //std::cout << "forceValue:\n" << forceValue << "\n";

    // Применяем силу к указанной точке на корпусе
    _robot->setExternalForce(baseIdx, forceApplicationPoint, externalForce);

    std_msgs::Float32 externalForceMsg;
    externalForceMsg.data = forceValue;
    _pub_external_force.publish(externalForceMsg);
    //_robot->setExternalTorque(baseIdx, ExternalTorque);
}

