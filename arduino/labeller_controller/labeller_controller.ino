#include <ros.h>
#include <opinau_msg/motor.h>


const byte stepPin = 2;
const byte directionPin = 3;
const byte enablePin = 4;
//const int proximityPin = 5;

int stepdelay = 3;           //rotation speed of starwheel
int stepnumber = 0;
int timetolabel = 1000;       //delay time needed to apply label

ros::NodeHandle  nh;

ros::Subscriber<std_msgs::Int32> motor_sub("labeller_motors", &motor_messageCb );



void motor_messageCb( const std_msgs::Int32& stepper_motor){          //calculates the time required to make RPM requested by BusinessLogic, then resets the timer
  int spin = period / (STEPS_PER_REVOLUTION * stepper_motor.data);
  currentMillis = millis();
  if(stepper_motor.data)
  { 
    if (currentMillis - startMillis >= period){
      myStepper.step(1);
      startMillis = currentMillis; 
    }
}
}


void setup() 
{
 nh.getHardware()->setBaud(115200); // needs to be synchronised with value in launch file
  nh.initNode();
  nh.subscribe(motor_sub);

  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  //pinMode(proximityPin, INPUT);

  digitalWrite(directionPin, HIGH); 

  Serial.println("enabled");
  digitalWrite(enablePin, HIGH); // HIGH is on for DM332T driver,  LOW is on for HY driver
}

void loop() {
  // put your main code here, to run repeatedly:

}
