#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <Stepper.h>


ros::NodeHandle  nh;
std_msgs::Bool sensor_one_reading;
std_msgs::Bool sensor_two_reading;

ros::Publisher pub_sensor_one("Sensor1", &sensor_one_reading);
ros::Publisher pub_sensor_two("Sensor2", &sensor_two_reading);

const int SENSOR_ONE_PIN = 13;
const int SENSOR_TWO_PIN = 12;
const int  RELAY_PIN_ONE = 32; //co2 valve
const int  RELAY_PIN_TWO= 30; //beer valve
const int  RELAY_PIN_THREE = 28;  //actuator
const int  RELAY_PIN_FOUR = 26;   //actuator

const int STEPS_PER_REVOLUTION = 200;   //default property of used motor, change to fit you model

Stepper myStepper(STEPS_PER_REVOLUTION, 8, 9, 10, 11);   // initialize the stepper library on pins 8 through 11

unsigned long startMillis;
unsigned long currentMillis;
const unsigned long period = 60000;

bool labeler_sensorState = true;
bool platforms_sensorState = true;

void valves_messageCb( const std_msgs::Bool& valves_status){             //open co2 valve, waits 10 seconds, closes co2 valve, opens beer valve and wait for sensor to                           
  if(valves_status.data)                                                // detect level of liquid
  {
       digitalWrite(RELAY_PIN_ONE,LOW);
       delay(10000);
       digitalWrite(RELAY_PIN_ONE,HIGH);
       digitalWrite(RELAY_PIN_TWO,LOW);
  }  
  else {
    digitalWrite(RELAY_PIN_TWO,HIGH);
  }
}

void actuator_messageCb( const std_msgs::Bool& actuator_control){
  if(actuator_control.data)
  {
       digitalWrite(RELAY_PIN_THREE,HIGH); 
        digitalWrite(RELAY_PIN_FOUR,LOW);
  }
  else {
    
   digitalWrite(RELAY_PIN_THREE,LOW);
        digitalWrite(RELAY_PIN_FOUR,HIGH);
  }
}


void stepper_messageCb( const std_msgs::Int32& stepper_motor){          //calculates the time required to make RPM requested by BusinessLogic, then resets the timer
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

ros::Subscriber<std_msgs::Bool> valves_status_sub("valves_status", &valves_messageCb );
ros::Subscriber<std_msgs::Bool> actuator_control_sub("actuator_control", &actuator_messageCb );          //subscribes to topics maintained by BusinessLogic, then uses callbacks 
ros::Subscriber<std_msgs::Int32> stepper_motor_sub("stepper_motor", &stepper_messageCb );                // to control the relays and stepper



void setup()
{
  nh.getHardware()->setBaud(115200); // needs to be synchronised with value in launch file
  nh.initNode();
  nh.advertise(pub_sensor_one);
  nh.advertise(pub_sensor_two);
  nh.subscribe(valves_status_sub);
  nh.subscribe(actuator_control_sub);
  
  pinMode(SENSOR_ONE_PIN, INPUT);
  pinMode(SENSOR_TWO_PIN, INPUT);
  
  pinMode(RELAY_PIN_ONE, OUTPUT);
  pinMode(RELAY_PIN_TWO, OUTPUT);
  pinMode(RELAY_PIN_THREE, OUTPUT);
  pinMode(RELAY_PIN_FOUR, OUTPUT);
  
  
  myStepper.setSpeed(60);
}
  
  
void loop()
  {  
     bool sensor_one_data = labeler_sensorState;
     bool sensor_two_data = platforms_sensorState;
     
     sensor_one_reading.data= sensor_one_data;
     sensor_two_reading.data= sensor_two_data;
     
     labeler_sensorState = digitalRead(SENSOR_ONE_PIN);
     platforms_sensorState = digitalRead(SENSOR_TWO_PIN);
     
     pub_sensor_one.publish(&sensor_one_reading);
     pub_sensor_two.publish(&sensor_two_reading);
     
       
    nh.spinOnce();
    
  }
      



