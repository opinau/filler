/*
*****************************************
Object detection and tracking sensor 
Project OPINAU-FILLER-CV
*****************************************

created 7. 2020.

Sensor array consist of 10 proximity sensors WSR-04459 , digital output from sensors is conencted to D3-D12 pins

Aray is based on Arduino Nano microcontroller board (ATMEGA328P)

External 5V power supply is required for optimal operation (please use jumper for selecting microcontroller power source, external vs USB)

Sensor array is operating when HIGH state is read on D2 pin, which is usefull for synconisation when multiple arrays are present in order to prevent interferetions.
For continious use please place jumper in correct location (HIGH state on D2)

Sensor array tracks objects in 20 discrete locations [pos0-pos19], an reports over UART if any change is detected

Message type:
<object_number,object_position,timestamp,timer_last_change_,extra_string>

example:
<23,11,45000,1000,STATUS_OK>  for correct track
or
<24,9,47000,3000,STATUS_ERROR_OVERFLOW> for detected error in object tracking

Use TX and RX pin for seril communication (only TX is required)
System is only reporting when HIGH state is present on D2 (enable) pin
This should enable easy repogramming of microcontrller board and use of ingertated UART hardware (sofserial is not required)

Please consult provided documentation for more details:

*/


// defining sensor pins, please note reverse order (device only operates / tracks objects in one direction)
byte sensor_1= 12;
byte sensor_2= 11;
byte sensor_3= 10;
byte sensor_4= 9;
byte sensor_5= 8;
byte sensor_6= 7;
byte sensor_7= 6;
byte sensor_8= 5;
byte sensor_9= 4;
byte sensor_10= 3;

#define refresh_period 10 // please use refres time between 5 (200Hz) and 20 ms (50Hz), lower number may result in false readings

long refresh_timer=0; //for remembering time of last reading event

//definig external enable pin
byte ext_enable= 2; //for continious use place jumper on correct location
                    //for sync with another sensor array use provided input cable instead of jumper


byte sensor_enable= 13; //enable pin which controls power supply for sensors (MOSFET)

long bottle_counter=0; //object counter

byte status_sensor[10]={0,0,0,0,0,0,0,0,0,0};
byte last_status_sensor[10]={0,0,0,0,0,0,0,0,0,0};
long object_pos[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


//structure for tracking object present in array
struct Objects {
   byte  enable=0;  // object is enabled
   byte  error_descr=0;  // if error id detected
   byte  pos=0; //current object position
   long  obj_counter;  //current object counter
   long  time_last_change=0;  //time from last change
   long   time_current=0;  //current time millis()
};

 struct Objects object_array[20];  //reserve space for 20 object (max number in line)
 // please note that dynamc allocation is not used, due to RAM stability

 byte global_error_flag=0;
 byte global_warning_flag=0;
 byte global_warning_pos=0;

// setup rutine, run only once
void setup() {
  // initialize serial communication at 115200 bits per second, change if requred
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.flush();
  delay(10);

  digitalWrite(sensor_enable,LOW); //power off senosos (MOSFET)


  //define input signals , from porximity sensors
  pinMode(sensor_1, INPUT_PULLUP);
  pinMode(sensor_2, INPUT_PULLUP);
  pinMode(sensor_3, INPUT_PULLUP);
  pinMode(sensor_4, INPUT_PULLUP);
  pinMode(sensor_5, INPUT_PULLUP);
  pinMode(sensor_6, INPUT_PULLUP);
  pinMode(sensor_7, INPUT_PULLUP);
  pinMode(sensor_8, INPUT_PULLUP);
  pinMode(sensor_9, INPUT_PULLUP);
  pinMode(sensor_10, INPUT_PULLUP);

  pinMode(ext_enable, INPUT); //external enable pin

  //wait for enable HIGH signal before loop rutine
  while(digitalRead(ext_enable)==LOW)
  {
    ; //empty loop
  }

  // small/short delay before first operation
  delay(500);

  refresh_timer=millis();   ///for last reading event
  
}


//loop routine
void loop() {

   //check for external enable signal (pin2)
     while(digitalRead(ext_enable)==LOW)
  {
    digitalWrite(sensor_enable,LOW);
  }
  //set pin13 to high
  // this powers sensors over MOSFET and light onboard LED
  digitalWrite(sensor_enable,HIGH);

  while (millis()-refresh_timer<refresh_period)
    ; //empty loop, let it wait until refresh_period elapses
 refresh_timer=millis(); //store new timer
  
    
  // read the input signals, note the reverse logic!
  status_sensor[0] =  !digitalRead(sensor_1);
  status_sensor[1]  = !digitalRead(sensor_2);
  status_sensor[2]  = !digitalRead(sensor_3);
  status_sensor[3]  = !digitalRead(sensor_4);
  status_sensor[4]  = !digitalRead(sensor_5);
  status_sensor[5]  = !digitalRead(sensor_6);
  status_sensor[6]  = !digitalRead(sensor_7);
  status_sensor[7]  = !digitalRead(sensor_8);
  status_sensor[8]  = !digitalRead(sensor_9);
  status_sensor[9]  = !digitalRead(sensor_10);

  int flag_change=0; //check if there is any change in input signals!
  
  for (int i=0;i<=9;i++)
  {
    if (status_sensor[i]!=last_status_sensor[i])
    //Serial.println("change detected!");
    flag_change++;
  }
  /*please note that change detection is not based on interrupts!
   * Arduino nano has only 2 ext interrupt pins, while sensor array monitors 10 signals!
   * As loop is quite fast (few ms), and objects are not extremly fast, this should work fine!
   */
  
   //check if first sensor detect new object!
   if ((status_sensor[0]!=last_status_sensor[0])& (status_sensor[0]==1)) //first sensor detects change and HIGH state
    {
      bottle_counter++; //object counter (bottle)
      object_pos[0]=bottle_counter;  //placing current counter in array of possible positions

      byte object_placeholder_counter=0;  //find first object struture not use (enable==false)

      while ((object_array[object_placeholder_counter].enable!=0) && object_placeholder_counter<=20) //check if more than 20 objects tracked.. 
        {
        object_placeholder_counter++;
        }
        if (object_placeholder_counter<20) // if less than 20 objects
        //initialize (enable) new object, reset all
          {
          object_array[object_placeholder_counter].enable=1;    
          object_array[object_placeholder_counter].obj_counter=bottle_counter;
          object_array[object_placeholder_counter].pos=0;
          object_array[object_placeholder_counter].time_current=millis();
          object_array[object_placeholder_counter].error_descr=0;
          }
          else
          //error! please disable production line, more than 20 object present!
          {
            global_error_flag=1;
            Serial.println("<0,0,0,0,CRITICAL_ERROR>"); 
          }
    }

   //track object change in production line
   for (int i=0;i<=9;i++)
   {
    if ((status_sensor[i]!=last_status_sensor[i])& (status_sensor[i]==0))
        {object_pos[i*2+1]=object_pos[i*2];
        object_pos[i*2]=0;
        if (object_pos[i*2+1]==0) //change detected in unusual pos
          global_warning_flag=1;//warinig, unknown object detected!
          global_warning_pos=i*2; //record warning pos
        }
   }

   for (int i=1;i<=9;i++)
   {
    if ((status_sensor[i]!=last_status_sensor[i])& (status_sensor[i]==1))
      {object_pos[i*2]=object_pos[i*2-1];
      object_pos[i*2-1]=0;
        if (object_pos[i*2]==0) //change detected in unusual pos
          global_warning_flag=1; //warinig, unknown object detected!
          global_warning_pos=i*2-1; //record warning pos
      }
   }

    //execute only if change is detected
    if (flag_change!=0)     //if flag_change counter is larger than 0, execute!
    {

      //update objects!
     for (int i=0;i<=19;i++) //all 20 objects
       {
            if (object_array[i].enable==1) //check if current object is enabled
            {
              for (int j=0;j<=19;j++) //inner loop
              {
                if (object_array[i].obj_counter==object_pos[j]) //object found (counter == number in object_pos array)
                      {
                        object_array[i].pos=j;  //update pos
                        object_array[i].time_last_change=millis()-object_array[i].time_current; //update last change timer
                        object_array[i].time_current=millis(); //update current time
                      }
              } //of for (int j=0;j<=19;j++)
          } //of if (object_array[i].enable==1)

    } // of for (int i=0;i<=19;i++)
    
    // note that object cannot be update past postion 19, last object hold pos19 until new one arrives
        
        
     //owerflow check, detect if more than one object in same postion (this is basically error!)
     for (int i=0;i<=19;i++) // for all 20 objects
     {
       for (int j=0;j<=19;j++)  // inner loop
       //check if more than one object exist in same pos
       if ((object_array[i].pos==object_array[j].pos) && (object_array[i].enable==1) && (object_array[j].enable==1) && (i!=j) && (object_array[i].pos<19))
         
         {//overflow detected!
          //kill object with lower counter!
          if (object_array[i].obj_counter<object_array[j].obj_counter)
            {
              object_array[i].enable=0; //disabling object
              object_array[j].error_descr=1;
            }
            else
             {
              object_array[j].enable=0; //disabling object
              object_array[i].error_descr=1;
            }
         }
     }

//kil /disable all object in pos 19, leave only one with highest .obj_counter 
// this version keeps latest (freshest) object in pos 19, while disabling others

//     long top_lastpos_counter=0;  
//     byte index_top_lastpos_counter=-1;
//     
//     for (int i=0;i<=19;i++)  //search for latest counter, find index
//     {
//      if ((object_array[i].enable==1) && (object_array[i].obj_counter>top_lastpos_counter) && (object_array[i].pos==19) )
//      //check if this object has largest obj_counter
//        {
//          top_lastpos_counter=object_array[i].obj_counter;
//          index_top_lastpos_counter=i;
//        }
//     }
//     for (int i=0;i<=19;i++) //kill or disable all others
//     {
//      if ((object_array[i].enable==1) && (object_array[i].pos==19) && (i!=index_top_lastpos_counter) )
//        {
//          object_array[i].enable=0;  //disable object
//        }
//     }

     // repair global waring flag, if object returns one pos
     // during production line object can return a little, ad be detected in prevous pos
     if (global_warning_flag==1)
       {
        for (int i=0;i<=19;i++)
        {
          if ((object_array[i].enable==1) && (object_array[i].pos-2==global_warning_pos)  )  //check if returned one pos
          {
            object_array[i].pos=global_warning_pos+1;
            global_warning_flag=0;
          }

          if ((object_array[i].enable==1) && (object_array[i].pos==global_warning_pos)  ) //check if obj is back on track
          {
            object_array[i].pos=global_warning_pos+1;
            global_warning_flag=0;
          }
          
        }
       }  //(global_warning_flag==1)


//uncomment for debugging, array containing counter of object and position in sensor array      
//     for (int i=0;i<=19;i++)
//       {
//        Serial.print(object_pos[i]);
//       Serial.print(",") ;
//        }
//     Serial.println("");

     //report over UART , all 20 objects
     for (int i=0;i<=19;i++)
     {
       if (object_array[i].enable==1) //report only enabled objects
         {
         Serial.print("<");
         Serial.print(object_array[i].obj_counter);
         Serial.print(",");
         Serial.print(object_array[i].pos);
         Serial.print(",");
         Serial.print(object_array[i].time_last_change);
         Serial.print(",");
         Serial.print(object_array[i].time_current);

         //report status, all other than STATUS_OK should stop production line!
         switch (object_array[i].error_descr) 
          {
            case 0:
                  Serial.print(",STATUS_OK");
                  break;
            case 1:
                  Serial.print(",ERROR_OWERFLOW"); //object /bottle dropped this is most usual
                  break;   
                  //add more type of error status if required
          }    
         Serial.println(">");
         
         }
     }

     //disabling all object in pos 19 (this version do not keep lates object in pos19)
     for (int i=0;i<=19;i++)  //search for latest counter, find index
     {
      if ((object_array[i].enable==1) && (object_array[i].pos==19) ) //check if object is enabled, and in pos 19
        object_array[i].enable=0;  //disable object
     }

    //check for global warninig, if not repaired
    if (global_warning_flag==1)  
      {
        Serial.print("<0,");        
        Serial.print(global_warning_pos);  //write pos of global warning. 
        if (global_warning_pos<=17) //possible object returning, ROS should take care of it
          Serial.println(",0,0,UNKNOWN_OBJECT>");
          else
          Serial.println(",0,0,RETURNING_OBJECT?>");
           
      }
 
     
    } // if (flag_change!=0)  

    //if global error is detected
    if  (global_error_flag==1)  //more than 20 object in line
          Serial.println("<0,0,0,0,CRITICAL_ERROR>"); //this error should stop production line!


  //update last status on sensors (change detection)
  for (int i=0;i<=9;i++)
  {
    last_status_sensor[i]=status_sensor[i];
  }
    //reset global errors and warinigs!
    global_error_flag=0;
    global_warning_flag=0;
  
  delay(1);        //extra delay for stability
}
