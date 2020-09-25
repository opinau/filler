#include <SoftwareSerial.h>
#include <String.h>
#include <ros.h>
#include <std_msgs/String.h>

const byte numChars = 64;
char receivedChars[numChars];

boolean newData = false;

const byte rxPin = 10;
SoftwareSerial myInputSerial (rxPin, 0);

ros::NodeHandle nh;

std_msgs::String hardware_msg;
ros::Publisher bottlePrecisePositionTopic("bottlePrecisePosition", &hardware_msg);



void setup() {
    Serial.begin(9600);

    myInputSerial.begin(9600);
    
    nh.initNode();
    nh.getHardware()->setBaud(115200);
    nh.advertise(bottlePrecisePositionTopic);
}


void loop() {
    recvWithStartEndMarkers();

    if (newData == true) {
        
        hardware_msg.data = receivedChars;
        bottlePrecisePositionTopic.publish( &hardware_msg );
          
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
