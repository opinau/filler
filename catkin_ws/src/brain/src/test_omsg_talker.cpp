#include "ros/ros.h"  //does not affect running
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "opinau_msgs/relay.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_omsg_talker");

  ros::NodeHandle n;

  ros::Publisher relay_instructions_pub = n.advertise<opinau_msgs::relay>("relay_instructions", 1000);

  ros::Rate loop_rate(10);

  int counter = 0;

  while (ros::ok())
  {
    opinau_msgs::relay msg;

    msg.index = 0;
    msg.enabled = 0;

    std::cout << "\n Instruction # " << counter << "; Relay: " << (int)msg.index
              << "; ON/OFF: " << (bool)msg.enabled;

    relay_instructions_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++counter;
  }

  ros::spin();

  return 0;
}
