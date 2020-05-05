#include <InkShieldMega.h>
#include "charset.h"
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
        //spray the lot String
           
    }
}

void spray_lot(String lot) {
    for (int i = 0; i < lot.length(); i++) {
        spray_letter(lot.charAt(i));
    }
}

void spray_letter(int letter)
{
  if(letter>=minChar && letter<=maxChar)
  {
    //loop through the rows of the letter
    for(int row=0;row<rowsPerChar;row++){
      //retrive the row
      word strip = font[((letter-minChar)*rowsPerChar)+row];
      //print the row
      MyInkShield.spray_ink(strip);
    }
  }
}