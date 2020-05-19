#include "ros.h" //does not affect running
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "/opinau_filler/opinau_relay.h"

ros::NodeHandle nh;
ros::Subscriber<opinau_filler::opinau_relay> sub("/relay_instructions", &keyboardCallback);

int relayArray[] = {2, 3, 4, 5, 6, 7, 8, 9};

void keyboardCallback(const opinau_filler::opinau_relay &command)
{
    int8 my_relay_number = command.relay_number;
    bool my_relay_on_off = command.relay_on_off;

    if (my_relay_on_off == 1)
    {
        relayArray[my_relay_number] = HIGH;
    }
    else if (my_relay_on_off == 0)
    {
        relayArray[my_relay_number] = LOW;
    }
}

void setup()
{
    for (int i = 0; i < 8; i++)
    {
        pinMode(relayArray[i], OUTPUT); //initialise releyArray as outputs
    }
}

void loop()
{

    nh.spinOnce(); //update ralay states based on heard instructions
    delay(10);
}