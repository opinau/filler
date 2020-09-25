#include <SoftwareSerial.h>
#include <String.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

const byte numChars = 64;
char receivedChars[numChars];
int resetPin = 2;

boolean newData = false;

const byte rxPin = 10;
SoftwareSerial myInputSerial (rxPin, 0);

ros::NodeHandle nh;

std_msgs::String hardware_msg;
ros::Publisher bottlePositionTopic("bottlePosition", &hardware_msg);
ros::Subscriber<std_msgs::Empty> sub("bottlePositionReset", &resetFunction);



void setup() {
    Serial.begin(9600);

    pinMode(resetPin,OUTPUT);
    digitalWrite(resetPin,HIGH);
    pinMode(13,OUTPUT);
    myInputSerial.begin(9600);
    
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.advertise(bottlePositionTopic);
    nh.subscribe(sub);
}



void loop() {
    recvWithStartEndMarkers();

    if (newData == true) {
        
        hardware_msg.data = receivedChars;
        bottlePositionTopic.publish( &hardware_msg );
          
        newData = false;
        
    }
     nh.spinOnce();
}

//============

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (myInputSerial.available() > 0 && newData == false) {
        rc = myInputSerial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; 
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }   
}

void resetFunction(const std_msgs::Empty& reset_msg) {
  digitalWrite(resetPin, LOW);
  digitalWrite(13, HIGH);
  delay(100);
  delay(500);
  myInputSerial.flush();
  digitalWrite(resetPin,HIGH);
  digitalWrite(13, LOW);
}
