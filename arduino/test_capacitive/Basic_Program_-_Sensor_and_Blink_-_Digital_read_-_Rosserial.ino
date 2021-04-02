#include <Arduino.h>

#include <ros.h>
#include <std_msgs/Bool.h>

const int KEY_DETECTOR_SENSOR_PIN = 52;

int last_read_sensor_value = 1;

ros::NodeHandle key_detector_node_handle;
std_msgs::Bool key_detector_reading;
ros::Publisher key_detector_publisher("key_detector", &key_detector_reading);


// the setup routine runs once when you press reset:
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(KEY_DETECTOR_SENSOR_PIN, INPUT);

  key_detector_node_handle.getHardware()->setBaud(115200);
  key_detector_node_handle.initNode();
  key_detector_node_handle.advertise(key_detector_publisher);
}

// the loop routine runs over and over again forever:
void loop() {
  int sensorValue = digitalRead(KEY_DETECTOR_SENSOR_PIN);  

  if (sensorValue != last_read_sensor_value) {
    if (sensorValue == 1)            
    {
       digitalWrite(LED_BUILTIN, HIGH);
       key_detector_reading.data = false;
    }
    else 
    {
       digitalWrite(LED_BUILTIN, LOW);
       key_detector_reading.data = true;
    }
    key_detector_publisher.publish(&key_detector_reading);
  }
  
  last_read_sensor_value = sensorValue;
  
  key_detector_node_handle.spinOnce();
}
