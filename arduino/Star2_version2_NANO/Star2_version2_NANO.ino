#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>



ros::NodeHandle nh;

std_msgs::Float64 labeler_revolver_control_data;
std_msgs::Float64 test_float;



const byte stepPin = 2;
const byte enablePin = 4;
const byte directionPin = 3; // direction
double stepdelay = 0.5;
int mode = 1;



void keyboardCallback( const std_msgs::String& command)
{
  String mystring = command.data;
  char mychar = mystring.charAt(0);

  if ( mychar == 'w'){
      mode = 0;
    } else if ( mychar == '2') {
      mode = 1;
    }
}

ros::Publisher labeler_revolver_control("/testy", &labeler_revolver_control_data);
ros::Subscriber<std_msgs::String> sub("/keyboard_instructions", &keyboardCallback);


void setup() {

  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);

  digitalWrite(directionPin, HIGH); 
  digitalWrite(enablePin, LOW); //  LOW is ON

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(labeler_revolver_control);
  test_float.data = 0.1;
}

void loop() {

  nh.spinOnce();
  delay(10); //was 10

  //labeler_revolver_control.publish(&test_float);


  if (mode == 1)
  {
    digitalWrite(stepPin, HIGH);
    delay(stepdelay);
    digitalWrite(stepPin, LOW);
    delay(stepdelay);
  }
}