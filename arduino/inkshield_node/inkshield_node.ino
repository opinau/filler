#include <InkShieldMega.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

//initialize shield on pin 2 (valid options are 2-12 or 26-29)
InkShieldA0A3 MyInkShield(2);

ros::NodeHandle nh;

int mode = 0;

void keyboardCallback(const std_msgs::String &command)
{
    String mystring = command.data;
    char mychar = mystring.charAt(0);

    if (mychar == 'r')
    {
        mode = 0;
    }
    else if (mychar == '4')
    {
        mode = 1;
    }
}

ros::Subscriber<std_msgs::String> sub("/keyboard_instructions", &keyboardCallback);

void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{

    nh.spinOnce();
    delay(10); //was 10

    if (mode == 1)
    {
        //spray all 12 nozzles as fast as possible
        //(blackout pattern 0x0FFF = 0000111111111111)
        MyInkShield.spray_ink(0x0FFF);
    }
}