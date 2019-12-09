const byte stepPin = 7;
const byte enablePin = 6;
const byte directionPin = 5; // direction

void setup() {
  Serial.begin(9600);    
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
    pinMode(directionPin, OUTPUT);

  digitalWrite(directionPin, LOW); 

  Serial.println("enabled");
  digitalWrite(enablePin, HIGH); // LOW is on  

}

void loop() {

  digitalWrite(stepPin, HIGH);
  delay(5);
  digitalWrite(stepPin, LOW);
  delay(5);
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
