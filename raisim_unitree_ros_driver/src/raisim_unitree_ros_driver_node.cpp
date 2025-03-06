#include "raisim_unitree_ros_driver.hpp"
#include "ros/init.h"

using namespace std;

ros::ServiceServer srv_restart;

bool srvRestart(std_srvs::Trigger::Request& reqest,
                std_srvs::Trigger::Response& response)
{
  ROS_WARN("RESTART!");

  unitree->~Unitree();

  unitree = new Unitree(SIM_DT);

  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "raisim_unitree_ros_driver_node");
  ros::NodeHandle n;
  ros::Rate rate(SIM_FREQ);

  ROS_INFO("Init");

  srv_restart = n.advertiseService("restart_sim", srvRestart);

  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  raisim::World::setActivationKey("~/.raisim/activation.raisim");

  unitree = new Unitree(SIM_DT);

  if (unitree->init())
  {
    ROS_ERROR("Init failed!");
    ros::shutdown();
  }

  double start_time = 0;
  double deltaT = 0;
  start_time = ros::Time::now().toSec();

  ROS_INFO("Enter loop");

  uint8_t counter = 0;

  while (ros::ok())
  {
    deltaT = ros::Time::now().toSec() - start_time;

    if (deltaT < INIT_TIME)
    {
      ROS_INFO_STREAM_ONCE("Wait: " << INIT_TIME << " seconds");
      unitree->updateWorldOnly();
    }
    else
    {
      ROS_INFO_STREAM_ONCE("Start");

      unitree->forceOn();
      unitree->updateWorld();
    }

    ros::spinOnce();
    rate.sleep();
  }
}
