#include <ros.h>
#include <opinau_msgs/motor.h>

const int STEPS_PER_REVOLUTION = 200;
const byte STEP_PIN = 2;
const byte DIRECTION_PIN = 3;
const byte ENABLE_PIN = 4;
//const int proximityPin = 5;

int stepdelay = 3; //rotation speed of starwheel
int stepnumber = 0;
int timetolabel = 1000; //delay time needed to apply label

bool running = false;
bool enabled = false;

ros::NodeHandle nh;

void motor_message_cb(const opinau_msgs::motor &msg)
{ 
  digitalWrite(LED_BUILTIN, HIGH);

  if (msg.enabled != enabled) {
    if (msg.enabled) {
      digitalWrite(ENABLE_PIN, LOW);
    }
    else {
      digitalWrite(ENABLE_PIN, HIGH);
    }
    enabled = msg.enabled;
  }


  //int spin = period / (STEPS_PER_REVOLUTION * msg.data.speed);
  //currentMillis = millis();
  //   if(stepper_motor.data)
  //   {
  //     if (currentMillis - startMillis >= period){
  //       myStepper.step(1);
  //       startMillis = currentMillis;
  //     }
  // }
}

ros::Subscriber<opinau_msgs::motor> motor_sub("labeller_motors", &motor_message_cb);

void setup()
{
  nh.getHardware()->setBaud(115200); // needs to be synchronised with value in launch file
  nh.initNode();

  nh.subscribe(motor_sub);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  //pinMode(proximityPin, INPUT);

  digitalWrite(DIRECTION_PIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);

  digitalWrite(ENABLE_PIN, HIGH); // HIGH is enabled on for DM332T, LOW is on for HY & DM860T driver
}

void loop()
{
  nh.spinOnce();
  delay(stepdelay);
}
