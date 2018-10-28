/*
Fall Detection, Alert and Action monitoring system
for the elderly-- An Intel IoT project by Sudharshan
Suresh
December 2014
The circuit:
ADXL335 (Accelerometer)
-----------------------
analog 0: accelerometer self test
analog 1: z-axis
analog 2: y-axis
analog 3: x-axis
GND: ground
+5V: vcc
HC-05 (Bluetooth Module)
------------------------
+3.3V: Vcc
GND: ground
Digital Pin 10(RX): TX
Digital Pin 11(TX): RX
Piezobuzzer
-----------
Digital Pin 9(PWM): +ve
GND: -ve
3-color LED
-----------
Digital Pin 2: RED
Digital Pin 3(GND): Ground
Digital Pin 4:GREEN
Digital Pin 5:BLUE
Push Button
-----------
Connected between Digital Pin 12 and GND
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Arduino Uno @ COM18---- via Arduino IDE
Standard Serial over bluetooth link @ COM21
(Incoming)---- via Processing
Standard Serial over bluetooth link @ COM23
(Outgoing)---- via TeraTerm
*/
#include <SoftwareSerial.h>
// Accelerometer pin description

 // acceleration in g's
along each axis
 // Net acceleration ( 1when stationary), pitch and roll convey orientationof accelerometer


SoftwareSerial mySerial(10, 11); //RX, TX
void setup()
{
  mySerial.begin(9600); // initialize the software serial communication
  pinMode(12, INPUT_PULLUP); // Pull-up switch
 connection
  pinMode(9, OUTPUT); // PWM output for
  piezobuzzer
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  //RGB LED Pins
  pinMode(2, OUTPUT); //Red
  pinMode(3, OUTPUT); // Ground
  pinMode(4, OUTPUT); //Green
  pinMode(5, OUTPUT); //Blue
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
 
}
void loop()
{
    
    // Serial transfer of required variables 
    pedometer(); // Function to sense steps taken bywearer
    distress(); // Manually operated distresssignalcan be sent via pushbutton hold
    fall_detect(); // FALL DETECTION FUNCTION
    delay(20); // Delay between passing values
}
void pedometer()
{
  stepcheck = !stepcheck; // To sense every alternate iteration
  if((Anet>1.2)&&(stepcheck==true)&&(state==0)&&(firstfoot==false)) // Sense initial spike in acc. reading
  {
  firstfoot = true;
  steptime = millis();
  }
  if((Anet<0.9)&&(((millis()-
  steptime)/1000)<2)&&(firstfoot == true)) // Sense subsequent drop in acc. reading
  {
  steps++;
  firstfoot = false;
  }
  else if((((millis()-
  steptime)/1000)>2)&&(firstfoot == true)) //
  maximum permissible time between above 2 conditions
  firstfoot = false;
}
void distress()
{
  if ((state==0)&&(digitalRead(12) == LOW))
  // detects button PRESS
  {
      button++;
      if(button>70)
        // detects button HOLD
        {
          tone(9,4000);
          // Generate piezotone for distress
          state = 42;
        }
  }
  else if((state==0)&&(digitalRead(12) == HIGH))
  // detects button release
  {
      buttonrel++;
      if(buttonrel>10){ 
          button = 0;
          buttonrel = 0;
      }
  }
}

void fall_detect()
{
    if(state ==0){
      digitalWrite(2, LOW);
      digitalWrite(5, LOW);
      digitalWrite(4, HIGH);
    }
    if((Anet<0.8)&&(state==0)&&((abs(pitch)<60)&&(abs(roll)<60))) // Initial drop in acc. due to free-fall like condition
    {
        state = 1;
        time = millis();
    }
    if(state == 1)
    {
        if(Anet>1) // Increase in acceleration due to impact of fall
          {
              state = 2;
              time = millis();
          }
        if(((millis()-time)/1000)>2) // max. permissible time between states 1 and 2
          state = 0;
    }
    if(state == 2){
        if((abs(pitch)>60)||(abs(roll)>60)) //Senses a fall in any orientation by factoring pitch and roll
        {
          state = 3;
          time = millis();
        }
        if((millis()-time)>100) // max.permissible time between states 2 and 3
          state = 0;
    }
    if(state == 3){
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH );
        if((abs(pitch)<60)&&(abs(roll)<60)) //Detects getting-up and prevents false alarm
        {
          count++;
          if(count>20)
          {
            state = 0;
            count = 0;
          }
        }
        if(((millis()-time)/1000)>3) // 3 second window after fall between states 3 and 4
        {
          state = 4;
          ALERT = 1;
          tone(9,250); // Fall, perhaps?
        }
   }
  if((state ==4)&&(ALERT==1))
    {
      digitalWrite(5, (blinkstate) ? LOW: HIGH);
      digitalWrite(2, (blinkstate) ? HIGH: LOW);
      blinkstate = !blinkstate;
      if(digitalRead(12) == LOW) //Checks for button press to indicate false alarm
      {
          ALERT = 0;
          state = 0;
      }
      if(((millis()-time)/1000)>20) // if patient is unresponsive for more than 20 secs, fall is confirmed-- necessary action taken via processing
      {
          state = 5;
          tone(9,4000); // Generatepiezotone for distress
          digitalWrite(5, LOW);
      }
    }
}
}
