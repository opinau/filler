const byte stepPin = 2;
const byte enablePin = 4;
const byte directionPin = 3; // direction
const int proximityPin = 5;

int stepdelay = 3;           //rotation speed of starwheel
int stepnumber = 0;
int timetolabel = 1000;       //delay time needed to apply label


void setup() {
  Serial.begin(9600);    
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(directionPin, OUTPUT);
 // pinMode(proximityPin, INPUT);

  digitalWrite(directionPin, HIGH); 

  Serial.println("enabled");
  digitalWrite(enablePin, HIGH); // LOW is on  



}

void loop() {
  
//check sensor  
//int sensorValue = digitalRead(proximityPin);
int sensorValue = 8888;
  
//run starwheel
  digitalWrite(stepPin, HIGH);
  delay(stepdelay);
  digitalWrite(stepPin, LOW);
  delay(stepdelay);

//count steps and read sensor
  stepnumber++;
  Serial.print("step : ");
  Serial.print(stepnumber);
  Serial.print("   proximity : ");
  Serial.println(sensorValue);
  

//pause starwheel at a given angle
  if (stepnumber == 200){
    stepnumber = 0;
    delay(timetolabel);
  }

//pause starwheel when proximity sensor reads 0
  //if (sensorValue == 0){
   // stepnumber = 0;
   // delay(timetolabel);
  //}

  
//  Serial.println("Beep!!");


//  Serial.println("lowhigh22");
 
  
//  if (Serial.available() > 0) {
//    Serial.println("whoosh");
//    delay(10);
//    String readString;
//    while (Serial.available() > 0) {
//      
//      
//      char c = Serial.read();  //gets one byte from serial buffer
//      readString += c; //makes the string readString
//    }
//          int steps = 0;
//          steps = readString.toInt();
//          Serial.println(steps);
//          digitalWrite(stepPin, LOW);
//          digitalWrite(stepPin, HIGH);
//          Serial.println("lowhigh22");
//  }
}
