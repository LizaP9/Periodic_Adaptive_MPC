#pragma once

/*! @file GamepadCommand.h
 *  @brief The GamepadCommand type containing joystick information
 */

/* * * BUTTONS * * * * * * * * * *
0 - X
1 - O
2 - triangle
3 - square
4 - L1
5 - R1
8 - share
9 - options
10 - ps
11 - L3
12 - R3
* * * Arrows in "axes" array * * *
left - axes[6] = 1
right - axes[6] = -1
down - axes[7] = -1
up - axes[7] = 1
 * * * * * * * * * * * * * * * * */

/* * * AXES (JOYSTICKS) * * *
 *                    left (down) / neutral / right (up)
0 - Left Horizontal           1.0 /       0 / -1.0
1 - Left Vertical            -1.0 /       0 /  1.0
2 - L2                       -1.0 /     1.0 /  1.0
3 - Right Horizontal          1.0 /     0.0 / -1.0
4 - Right Vertical           -1.0 /     0.0 /  1.0
5 - R2                       -1.0 /     1.0 /  1.0
 * * * * * * * * * * * * * * */

#include "cppTypes.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

enum StairsMode
{
  UP = 0,
  DOWN = 1
};

/*!
 * The state of the gamepad
 */
class GamepadCommand
{
public:
  GamepadCommand();

  bool up, down, left, right;
  bool triangle, circle, cross, rectangle;
  bool L1, R1, L3, R3;
  bool share;
  bool options;
  bool PS;
  bool chord_op_l1;

  float L2, R2;

  Vec2<float> left_stick_analog, right_stick_analog;

  void zeroButtons();

  float max_roll = 0.8;
  float min_roll = -0.8;
  float max_pitch = 0.4;
  float min_pitch = -0.4;
  float max_vel_x = 0.7;
  float min_vel_x = -0.7;
  float max_vel_y = 0.5;
  float min_vel_y = -0.5;
  float max_turn_rate = 2.5;
  float min_turn_rate = -2.5;

  uint8_t stairs_mode = 0;

private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub_joy;

  void _initSubscribers();
  void _joyCallback(sensor_msgs::Joy msg);
};