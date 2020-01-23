#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 labeler_revolver_control_data;

void messageCb(const std_msgs::String &toggle_msg)
{
    digitalWrite(13, HIGH - digitalRead(13)); // blink the led
}

ros::Subscriber<std_msgs::String> sub("conveyor_instructions", &messageCb);
ros::Publisher labeler_revolver_control("/filler_robot/revolver_labeler_velocity_controller/command", &labeler_revolver_control_data);


void setup()
{
    pinMode(13, OUTPUT);
    nh.getHardware()->setBaud(9600);
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(labeler_revolver_control);
}

void loop()
{
    nh.spinOnce();
    delay(1);
}
