#include "GamepadCommand.h"

GamepadCommand::GamepadCommand()
{
  zeroButtons();

  _initSubscribers();
}

void GamepadCommand::_joyCallback(sensor_msgs::Joy msg)
{
  zeroButtons();

  if (msg.axes.size() >= 7)
  {
    if (msg.axes[6] > 0)
    {
      left = true;
    }

    if (msg.axes[6] < 0)
    {
      right = true;
    }

    if (msg.axes[7] > 0)
    {
      up = true;
    }

    if (msg.axes[7] < 0)
    {
      down = true;
    }
  }
  if (msg.buttons[4] && msg.buttons[9])
  {
    chord_op_l1 = true;
  }

  cross = msg.buttons[0];
  circle = msg.buttons[1];
  triangle = msg.buttons[2];
  rectangle = msg.buttons[3];
  L1 = msg.buttons[4];
  R1 = msg.buttons[5];
  share = msg.buttons[8];
  options = msg.buttons[9];
  PS = msg.buttons[10];
  L3 = msg.buttons[11];
  R3 = msg.buttons[12];

  left_stick_analog[1] = msg.axes[1];
  left_stick_analog[0] = msg.axes[0];

  right_stick_analog[1] = msg.axes[4];
  right_stick_analog[0] = msg.axes[3];
}

void GamepadCommand::_initSubscribers()
{
  _sub_joy = _nh.subscribe("/joy", 1, &GamepadCommand::_joyCallback, this, ros::TransportHints().tcpNoDelay(true));
}

void GamepadCommand::zeroButtons()
{
  up = false;
  down = false;
  left = false;
  right = false;
  triangle = false;
  circle = false;
  cross = false;
  rectangle = false;
  PS = false;
  L1 = false;
  R1 = false;
  L3 = false;
  R3 = false;
  chord_op_l1 = false;

  L2 = 0.0;
  R2 = 0.0;

  left_stick_analog = Vec2<float>::Zero();
  right_stick_analog = Vec2<float>::Zero();
}
