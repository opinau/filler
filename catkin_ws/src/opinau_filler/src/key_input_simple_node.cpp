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

    /* advertise publishing */
    ros::Publisher conveyor_instructions_pub = n.advertise<std_msgs::String>("conveyor_instructions", 1000);
    ros::Publisher keyboard_input_pub = n.advertise<std_msgs::String>("keyboard_input", 1000);//added

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
        if (c == 'y')
            msg.data = "stop";
        if (c == 'x')
            msg.data = "go";

        ROS_INFO("%s", msg.data.c_str());

        conveyor_instructions_pub.publish(msg);
        keyboard_input_pub.publish(msg);//added

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
}