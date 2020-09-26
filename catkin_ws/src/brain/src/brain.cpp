#include "ros/ros.h"  //does not affect running
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <sstream>
#include "messages/opinau_inkshield.h"

// Callback function ADDED

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char** argv)
{
  /**
    * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
  40    * For programmatic remappings you can use a different version of init() which takes
  41    * remappings directly, but for most command-line programs, passing argc and argv is
  42    * the easiest way to do it.  The third argument to init() is the name of the node.
  43    *
  44    * You must call one of the versions of ros::init() before using any other
  45    * part of the ROS system.
  46    */
  ros::init(argc, argv, "brain");

  /**
  50    * NodeHandle is the main access point to communications with the ROS system.
  51    * The first NodeHandle constructed will fully initialize this node, and the last
  52    * NodeHandle destructed will close down the node.
  53    */

  ros::NodeHandle n;

  /**
  57    * The advertise() function is how you tell ROS that you want to
  58    * publish on a given topic name. This invokes a call to the ROS
  59    * master node, which keeps a registry of who is publishing and who
  60    * is subscribing. After this advertise() call is made, the master
  61    * node will notify anyone who is trying to subscribe to this topic name,
  62    * and they will in turn negotiate a peer-to-peer connection with this
  63    * node.  advertise() returns a Publisher object which allows you to
  64    * publish messages on that topic through a call to publish().  Once
  65    * all copies of the returned Publisher object are destroyed, the topic
  66    * will be automatically unadvertised.
  67    *
  68    * The second parameter to advertise() is the size of the message queue
  69    * used for publishing messages.  If messages are published more quickly
  70    * than we can send them, the number here specifies how many messages to
  71    * buffer up before throwing some away.
  72    */

  ros::Publisher starwheel1_instructions_pub = n.advertise<std_msgs::String>("starwheel1_instructions", 1000);
  ros::Publisher starwheel2_instructions_pub = n.advertise<std_msgs::String>("starwheel2_instructions", 1000);
  ros::Publisher conveyor_instructions_pub = n.advertise<std_msgs::String>("conveyor_instructions", 1000);

  ros::Publisher labelprint_instructions_pub =
      n.advertise<messages::opinau_inkshield>("labelprint_instructions", 1000);

  ros::Publisher labelapply_instructions_pub = n.advertise<std_msgs::String>("labelapply_instructions", 1000);
  ros::Publisher labelpull_instructions_pub = n.advertise<std_msgs::String>("labelpull_instructions", 1000);

  ros::Subscriber keyboard_instructions = n.subscribe("keyboard_instructions", 1000, chatterCallback);

  ros::Rate loop_rate(10);

  /**
  78    * A count of how many messages we have sent. This is used to create
  79    * a unique string for each message.
  80    */
  int count = 0;
  while (ros::ok())
  {
    /**
    85      * This is a message object. You stuff it with data, and then publish it.
    86      */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Start / Stop" << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
    96      * The publish() function is how you send messages. The parameter
    97      * is the message object. The type of this object must agree with the type
    98      * given as a template parameter to the advertise<>() call, as was done
    99      * in the constructor above.
    100      */

    messages::opinau_inkshield ink_msg;
    ink_msg.inkshield_on_off = 0;
    ink_msg.inkshield_date_lot = "NEW DATE, NEW DATE, NEW DATE";

    starwheel1_instructions_pub.publish(msg);
    starwheel2_instructions_pub.publish(msg);
    conveyor_instructions_pub.publish(msg);
    labelprint_instructions_pub.publish(ink_msg);
    labelapply_instructions_pub.publish(msg);
    labelpull_instructions_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  ros::spin();  // added

  return 0;
}
