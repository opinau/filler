#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>


const byte stepPin = 2;
const byte enablePin = 4;
const byte directionPin = 3; // direction
//const int proximityPin = 5;

int stepdelay = 3;           //rotation speed of starwheel
int stepnumber = 0;
int timetolabel = 1000;       //delay time needed to apply label

ros::NodeHandle  nh;

ros::Subscriber<std_msgs::Int32> stepper_motor_sub("stepper_motor", &stepper_messageCb );



void setup() 
{
 nh.getHardware()->setBaud(115200); // needs to be synchronised with value in launch file
  nh.initNode();
  nh.subscribe(actuator_control_sub);

  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
 // pinMode(proximityPin, INPUT);

  digitalWrite(directionPin, HIGH); 

  Serial.println("enabled");
  digitalWrite(enablePin, HIGH); // HIGH is on for DM332T driver,  LOW is on for HY driver 


}

void loop() {
  // put your main code here, to run repeatedly:

}
