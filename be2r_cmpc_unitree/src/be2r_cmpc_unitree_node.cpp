#include "be2r_cmpc_unitree.hpp"

using namespace std;
// using namespace USDK;

float simulation_time = 0.0f;

// Функция обратного вызова для обновления времени симуляции
void simulationTimeCallback(const std_msgs::Float32::ConstPtr& msg)
{
    simulation_time = msg->data;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "unitree_ctrl");
  ros::NodeHandle n;
  ros::Rate rate(MAIN_LOOP_RATE);

  ros::Subscriber sim_time_sub = n.subscribe("/sim_time", 1, simulationTimeCallback);


  ROS_INFO("Initialization...");

  Body_Manager unitree;

  unsigned long tick = 0;

  while (tick < MAIN_LOOP_RATE / 3)
  {
    tick++;
    ROS_INFO_ONCE("WAIT ROS Init");
    rate.sleep();
    ros::spinOnce();
  }

  unitree.init();

  UNITREE_LEGGED_SDK::LoopFunc* loop_udpSend = nullptr;
  UNITREE_LEGGED_SDK::LoopFunc* loop_udpRecv = nullptr;

  if (unitree.is_udp_connection)
  {
    ROS_WARN("UDP CONNECTION");
    loop_udpSend = new UNITREE_LEGGED_SDK::LoopFunc("udp_send", 0.002, 3, boost::bind(&Body_Manager::UDPSend, &unitree));
    loop_udpRecv = new UNITREE_LEGGED_SDK::LoopFunc("udp_recv", 0.002, 3, boost::bind(&Body_Manager::UDPRecv, &unitree));
    loop_udpSend->start();
    loop_udpRecv->start();
  }

  ROS_INFO("Initialization Done!");

  Timer t1;

  while (ros::ok())
  {
    // t1.start();

    unitree.run();

    // cout << t1.getMs() << endl;

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
