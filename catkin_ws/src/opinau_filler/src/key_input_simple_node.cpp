#include <ros/ros.h>
#include <termios.h>
#include <std_msgs/String.h>

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt); // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);               // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

  int c = getchar(); // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
  return c;
}

int main(int argc, char **argv)
{
  /*initialize*/
  ros::init(argc, argv, "key_input_simple_node");

  /*node handle*/

  ros::NodeHandle n;

  //Use node name to determine the instructions that it publishes to the brain
  std::string thisnodename = ros::this_node::getName();
  thisnodename.erase(0, 1); // removes the default leading backslash that is output by getName
  ROS_INFO(" NODE NAME -> %s", thisnodename.c_str());
  thisnodename.append("_instructions");
  ROS_INFO(" %s", thisnodename.c_str());

  /* advertise publishing */
  ros::Publisher keyboard_instructions_pub = n.advertise<std_msgs::String>(thisnodename, 1000);

  ros::Rate loop_rate(10);

  /**
  78    * A count of how many messages we have sent. This is used to create
  79    * a unique string for each message.
  80    */

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    int c = getch(); // call your non-blocking input function
    if (c == '1')
      msg.data = '1'; //"starwheel1_go_message";
    if (c == 'q')
      msg.data = 'q'; //"starwheel1_stop_message";
    if (c == '2')
      msg.data = '2'; //"starwheel2_go_message";
    if (c == 'w')
      msg.data = 'w'; // "starwheel2_stop_message";
    if (c == '3')
      msg.data = '3'; //"conveyor_go_message";
    if (c == 'e')
      msg.data = 'e'; //"conveyor_stop_message";
    if (c == '4')
      msg.data = '4'; //"labelink_go_message";
    if (c == 'r')
      msg.data = 'r'; //"labelink_stop_message";
    if (c == '5')
      msg.data = '5'; // "labelapply_go_message";
    if (c == 't')
      msg.data = 't'; //"labelapply_stop_message";
    if (c == '6')
      msg.data = '6'; //"labelpull_go_message";
    if (c == 'z')
      msg.data = 'z'; //"labelpull_stop_message";
    if (c == '0')
      msg.data = '0'; //"emergency_stop_message";

    ROS_INFO("%s", msg.data.c_str());

    keyboard_instructions_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}