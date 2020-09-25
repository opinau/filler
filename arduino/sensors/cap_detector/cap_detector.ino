#include <SoftwareSerial.h>
#include <String.h>
#include <ros.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

const byte numChars = 32;
char receivedChars[numChars];
char ROSChars[numChars];
char tempChars[numChars];        // temporary array for parsing

int bottleNum = 0;
int detectionNum = 0;
int capStatus = -1;
float capAngle = -1.0;
char messageFromCamera[numChars] = {0};

boolean newData = false;

const byte rxPin = 10;
SoftwareSerial myInputSerial (rxPin, 0);

ros::NodeHandle nh;
diagnostic_msgs::DiagnosticStatus diag_msg;
ros::Publisher capStatusTopic("capStatus", &diag_msg);



void setup() {
    myInputSerial.begin(9600);
    
    nh.initNode();
    nh.advertise(capStatusTopic);
}



void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars); // this protects the original data
        parseData();

        String message = "B: ";
        message += bottleNum;
        message += "; D: ";
        message += detectionNum;
        message += "; Ang: ";
        message += capAngle;
        message += "; Misc: ";
        message += messageFromCamera;
        message.toCharArray(ROSChars,64);
         
        diag_msg.level = capStatus;
        diag_msg.name = "Cap Detector #1";
        diag_msg.message = ROSChars;
        diag_msg.hardware_id = "1-1";

        capStatusTopic.publish( &diag_msg );
        
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



void parseData() {      

    char * strtokIndx; 

    
    strtokIndx = strtok(tempChars,",");      
    bottleNum = atoi(strtokIndx);
 
    strtokIndx = strtok(NULL, ","); 
    detectionNum = atoi(strtokIndx); 

    strtokIndx = strtok(NULL, ","); 
    capStatus = atoi(strtokIndx); 

    strtokIndx = strtok(NULL, ",");
    capAngle = atof(strtokIndx);    

    strtokIndx = strtok(NULL,",");      
    strcpy(messageFromCamera, strtokIndx); 
}
