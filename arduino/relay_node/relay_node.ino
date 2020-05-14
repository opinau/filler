int relayArray[] = {2,3,4,5,6,7,8,9};

int delayTime = 100;

void setup() {
  //initialise releyArray as outputs
  for(int i = 0; i<10; i++)
  {
    pinMode(relayArray[i], OUTPUT);
  }
}

void loop() {
  //turn on from 0-7
  for(int i = 0; i <= 7; i++)
  {
    digitalWrite(relayArray[i], LOW);
    delay(delayTime*(i+1));
  }

  //turn off from 7-0
  for(int i = 7; i >= 0; i--)
  {
    digitalWrite(relayArray[i], HIGH);
    delay(delayTime*(i+1));
  }
}
