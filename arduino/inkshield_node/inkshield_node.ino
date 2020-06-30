#include <InkShieldMega.h>
#include "charset.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

//initialize shield on pin 2 (valid options are 2-12 or 26-29)
InkShieldA0A3 MyInkShield(2);

ros::NodeHandle nh;

int mode = 3;                                    //mode of operation of the inkshield
const int label_sensor = A0;                     // label sensor on pin 8
const int led_test = 11;                         // led indicator for testing on pin 11
String lot_test = "DATE: 01/01/2020 LOT: 00001"; // use if no lot string on topic

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
    else if (mychar == 'f')
    {
        mode = 2;
    }
}

void lotStringCallback(const std_msgs::String &lot)
{
    spray_lot(lot.data);
}

ros::Subscriber<std_msgs::String> sub("/keyboard_instructions", &keyboardCallback);

void setup()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(sub);

    pinMode(label_sensor, INPUT);
    pinMode(led_test, OUTPUT);
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
    else if (mode == 2)
    {
        // spray the lot String

        // read lot string on designated topic
        // for each char get equivalent ascii value
        // int char_ascii_value = char_value
        // spray each char_ascii_value
    }

    else if (mode == 3)
    {
        // read label sensor, if voltage not zero react with LED

        if (analogRead(label_sensor) == 0)
        {
            digitalWrite(led_test, HIGH);
            delay(1000);
            digitalWrite(led_test, LOW);
            delay(1000);
        }
    }
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