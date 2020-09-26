#include "ros/ros.h"  //does not affect running
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "brain/opinau_relay.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_omsg_talker");

  ros::NodeHandle n;

  ros::Publisher relay_instructions_pub = n.advertise<brain::opinau_relay>("relay_instructions", 1000);

  ros::Rate loop_rate(10);

  int counter = 0;

  while (ros::ok())
  {
    brain::opinau_relay msg;

    msg.relay_number = 0;
    msg.relay_on_off = 0;

    std::cout << "\n Instruction # " << counter << "; Relay: " << (int)msg.relay_number
              << "; ON/OFF: " << (bool)msg.relay_on_off;

    relay_instructions_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++counter;
  }

  ros::spin();

  return 0;
}
