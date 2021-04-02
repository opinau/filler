#include <Arduino.h>

#include <Stepper.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

std_msgs::Float64 labeler_revolver_control_data;
std_msgs::Float64 main_revolver_control_data;
std_msgs::Float64 filling_platform_control_data;
std_msgs::Float64 capping_platform_control_data;

bool labeler_revolver_state;
bool main_revolver_state;
//bool platform_state; #TODO


ros::NodeHandle  nh;

ros::Publisher labeler_revolver_control("/filler_robot/revolver_labeler_velocity_controller/command", &labeler_revolver_control_data);
ros::Publisher main_revolver_control("/filler_robot/revolver_main_velocity_controller/command", &main_revolver_control_data);
ros::Publisher filling_platform_control("/filler_robot/filler_platform_effort_controller/command", &filling_platform_control_data);
ros::Publisher capping_platform_control("/filler_robot/capper_platform_effort_controller/command", &capping_platform_control_data);

const int STEPS_PER_REVOLUTION = 64 * 32;

Stepper myStepper(STEPS_PER_REVOLUTION, 3, 4, 5 ,6);


const int DIRECTION_PIN = 24;
const int STEP_PIN = 26;

const int SLEEP_PIN = 23;
const int RESET_PIN = 25;

int labeler_sensorState = 0;
int filler_sensorState = 0;

const int RELAY_PIN_1 = 34;
const int RELAY_PIN_2 = 36;

const int LABELER_SENSOR_PIN = 13;
const int FILLER_SENSOR_PIN = 12;


void setup(){

  myStepper.setSpeed(60);

  Serial.begin(9600);
  pinMode(DIRECTION_PIN,OUTPUT); 
  pinMode(STEP_PIN,OUTPUT);

  pinMode(SLEEP_PIN,OUTPUT); 
  pinMode(RESET_PIN,OUTPUT); 

  digitalWrite(SLEEP_PIN, HIGH); 
  digitalWrite(RESET_PIN, HIGH);

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT); 

  nh.getHardware()->setBaud(4800);
  nh.initNode();

  nh.advertise(labeler_revolver_control);
  nh.advertise(main_revolver_control);
  nh.advertise(filling_platform_control);
  nh.advertise(capping_platform_control);

}

void loop() {
  labeler_sensorState = digitalRead(LABELER_SENSOR_PIN);
  filler_sensorState = digitalRead(FILLER_SENSOR_PIN);
  myStepper.step(STEPS_PER_REVOLUTION);

  if ( labeler_sensorState == HIGH)           
  {

    Serial.println("Labelling");
    bool labeler_revolver_state = true;
    labeler_revolver_control_data.data= labeler_revolver_state;
    labeler_revolver_control.publish(&labeler_revolver_control_data);
    //delay(100);
    //nh.spinOnce();

  }               
  if (labeler_sensorState == LOW)
  {
    myStepper.step(4);
    Serial.println("Done");
    bool labeler_revolver_state = false;
    labeler_revolver_control_data.data = labeler_revolver_state;
    labeler_revolver_control.publish(&labeler_revolver_control_data);
    //delay(100);
    //nh.spinOnce();

  }

  if ( filler_sensorState == HIGH)           
  {

    Serial.println("Filling");
    bool main_revolver_state = true;
    main_revolver_control_data.data = main_revolver_state;
    main_revolver_control.publish(&main_revolver_control_data);
    
    // bool platform_state = true;
    //filling_platform_control_data=   #TODO
    //filling_platform_control.publish(& #TODO)
    //capping_platform_control_data=   #TODO

    digitalWrite(RELAY_PIN_1,HIGH); //lowering the filler
    digitalWrite(RELAY_PIN_2,LOW);
    //delay(100);
    //nh.spinOnce();

  }               
  if (filler_sensorState == LOW )
  {
    myStepper.step(4);
    Serial.println("Done");               
    bool main_revolver_state = false;

    main_revolver_control_data.data= main_revolver_state;
    main_revolver_control.publish(&main_revolver_control_data);
    
    //// bool platform_state = false;
    //filling_platform_control_data=   #TODO
    //filling_platform_control.publish(& #TODO)
    //capping_platform_control_data=   #TODO

    digitalWrite(RELAY_PIN_1,LOW);  //filler goes up
    digitalWrite(RELAY_PIN_2,HIGH);   
    //delay(100);
    //nh.spinOnce();

  }



  nh.spinOnce();

}









