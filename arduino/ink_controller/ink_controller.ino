#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <opinau_msgs/ink.h>
#include <opinau_msgs/ink_status.h>

#include "InkShieldMega/InkShieldMega.h"
#include "InkShieldMega/InkShieldMega.cpp"
#include "charset.h"

//initialize shield on pin 2 (valid options are 2-12 or 26-29)
InkShieldA0A3 MyInkShield(2);

ros::NodeHandle nh;

bool enabled = false;
bool printing = false;
const int label_sensor = A0;                    // label sensor on pin A0
const int led_test = 11;                        // led indicator for testing on pin 11
String bbd_lot = "DATE: 01/01/2020 LOT: 00001"; // use if no lot string on topic

opinau_msgs::ink_status status;

void ink_message_cb(const opinau_msgs::ink &msg)
{
    enabled = msg.enabled;
    bbd_lot = msg.bbd_lot;
}

ros::Subscriber<opinau_msgs::ink> ink_sub("/labeller_ink", &ink_message_cb);
ros::Publisher ink_status_pub("/ink_status", &status);

void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(ink_sub);
    nh.advertise(ink_status_pub);

    pinMode(label_sensor, INPUT);
    pinMode(led_test, OUTPUT);
}

void loop()
{
    nh.spinOnce();
    delay(10);
    if (enabled)
    {
        bool label_present = analogRead(label_sensor) == 0;
        if (label_present)
        {
            printing = true;
            //digitalWrite(led_test, HIGH);
            delay(1000);
            spray_lot(bbd_lot);
            status.label_present = true;
        }
        else
        {
            status.label_present = true;

            if (printing)
            {
                printing = false;
            }

            // TODO: Tell the labeller motor 1 to stop
            // or let's do that from brain / GUI?
        }
    }

    ink_status_pub.publish(&status);
}

void spray_lot(String lot)
{
    for (int i = 0; i < lot.length(); i++)
    {
        spray_letter(lot.charAt(i));
    }
}

void spray_letter(int letter)
{
    if (letter >= minChar && letter <= maxChar)
    {
        //loop through the rows of the letter
        for (int row = 0; row < rowsPerChar; row++)
        {
            //retrive the row
            word strip = font[((letter - minChar) * rowsPerChar) + row];
            //print the row
            MyInkShield.spray_ink(strip);
        }
    }
}
