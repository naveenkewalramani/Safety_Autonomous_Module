//library included
#include <SoftwareSerial.h>
#include<dht.h>
#include <MQ2.h>
dht DHT;
SoftwareSerial GPRS(7, 8);

//pin declaration'
const int zpin = A0; // z-axis
const int ypin = A1; // y-axis
const int xpin = A2; // x-axis
const int Hallpin = A3; //Hall  sensor
#define DHT11_PIN A4
const int mq2pin = A5; // gas sensor

MQ2 mq2(mq2pin);
float Xval, Yval, Zval,Hallval;//for accelerometer
float Anet, pitch, roll;
unsigned long Time = 0; // variable to compute Timedifferences
unsigned long steps = 0; // number of steps taken
unsigned long stepTime; // variable to compute Time between steps
boolean stepcheck = false;
boolean firstfoot = false;
int state = 0; // condition of subject ( 0 indicates stable and 5 indicates fallen )
int count =0;
int buttonrel = 0;
int button = 0;
int ALERT = 0; // fall, perhaps?
int blinkstate = 0;

//below are the calibration values for the 3 axes, such that in stable condition the values are (1g, 0g, 0g)
float xZero = 505;
float yZero = 510;
float zZero = 525;

//for gas sensor
int LPG,CO,SMOKE;
float sensor_volt;
float RS_air; //get value of Rs in GAS
float R0;
float ratio; //Get ratio RS_GAS/RS_air

void setup() {
  
  pinMode(4,OUTPUT);
  mq2.begin();
  
  //RGB led work(PENDING)
  pinMode(9, OUTPUT); //Red
  pinMode(10, OUTPUT); // Ground
  pinMode(11, OUTPUT); //Green
  pinMode(12, OUTPUT); //Blue
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, LOW);

  GPRS.begin(9600);
  Serial.begin(9600);
  GPRS.println("AT+CMGF=1");
}

void loop() {
    while(GPRS.available()) {
    Serial.write(GPRS.read());
  }
    //Taking input from sensors
    Xval = (analogRead(xpin)- xZero)/102;
    Yval = (analogRead(ypin) - yZero)/102;
    Zval = (analogRead(zpin) - zZero)/102;
    Hallval = analogRead(Hallpin);
    int chk = DHT.read11(DHT11_PIN);

    //Calculations for Accelerometer
    Anet = sqrt(sq(Xval) + sq(Yval) + sq(Zval));
    pitch = atan(Xval/sqrt(pow(Yval,2) +pow(Zval,2)));
    pitch = pitch * (180.0/PI) - 90;
    roll = atan(Yval/sqrt(pow(Xval,2) +
    pow(Zval,2)));
    roll = roll * (180.0/PI);

    //Calculations for DHT11 temperature
    float temp=DHT.temperature;
    float humid = DHT.humidity;
    float Kelvin = temp+273.15;
    float Fahrenheit = (temp*9)/5 + 32;
    float Rankine = (Kelvin*9)/5;  
    
    //Calculations for Mq2
    LPG = mq2.readLPG();
    CO = mq2.readCO();
    SMOKE = mq2.readSmoke();
    float sensorValue=0.0;
    for(int x=0;x<100;x++){
      sensorValue = sensorValue + mq2pin;
    }
    sensorValue = sensorValue/100.0;
    sensor_volt = (float)sensorValue/1024*5.0;
    RS_air = (5.0 -sensor_volt)/sensor_volt;
    R0 = RS_air/9.8;
    ratio = RS_air/R0;
    
    //Calculation for hall Effect
    
    //Threshold
    if(temp>=30.00 or humid<=40.00){
      digitalWrite(4,HIGH);
       sendSMS();
    }
    
    else
      digitalWrite(4,LOW);
    //Printing 
    Serial.print(" Temperature = ");
    Serial.print(temp);
    Serial.print('\t');
    Serial.print(" Humidity = ");
    Serial.println(humid);
    Serial.print("Xval = ");
    Serial.print(Xval);
    Serial.print('\t');
    Serial.print("Yval = ");
    Serial.print(Yval);
    Serial.print('\t');
    Serial.print("Zval = ");
    Serial.print(Zval);
    Serial.print('\t');
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print('\t');
    Serial.print(" Roll = ");
    Serial.println(roll);
    Serial.print(" State = ");
    Serial.print(state);
    Serial.print('\t');
    Serial.print(" HallEffect = ");
    Serial.println(Hallval);
    Serial.print(" LPG = ");
    Serial.print(LPG);
    Serial.print('\t');
    Serial.print(" CO =");
    Serial.print(CO);
    Serial.print('\t');
    Serial.print(" SMOKE = ");
    Serial.println(SMOKE);
    fall_detect(); // FALL DETECTION FUNCTION
    pedometer(); // Function to sense steps taken bywearer
    distress(); // Manually operated distresssignalcan be sent via pushbutton hold
    delay(2000);
}
void pedometer()
{
  stepcheck = !stepcheck; // To sense every alternate iteration
  if((Anet>1.2)&&(stepcheck==true)&&(state==0)&&(firstfoot==false)) // Sense initial spike in acc. reading
  {
  firstfoot = true;
  stepTime = millis();
  }
  if((Anet<0.9)&&(((millis()-
  stepTime)/1000)<2)&&(firstfoot == true)) // Sense subsequent drop in acc. reading
  {
  steps++;
  firstfoot = false;
  }
  else if((((millis()-
  stepTime)/1000)>2)&&(firstfoot == true)) //maximum permissible Time between above 2 conditions
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
          //tone(9,4000);
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
      digitalWrite(9, LOW);
      digitalWrite(12, LOW);
      digitalWrite(11, HIGH);
    }
    if((Anet<0.8)&&(state==0)&&((abs(pitch)<60)&&(abs(roll)<60))) // Initial drop in acc. due to free-fall like condition
    {
        state = 1;
        Time = millis();
    }
    if(state == 1)
    {
        if(Anet>1) // Increase in acceleration due to impact of fall
          {
              state = 2;
              Time = millis();
          }
        if(((millis()-Time)/1000)>2) // max. permissible Time between states 1 and 2
          state = 0;
    }
    if(state == 2){
        if((abs(pitch)>60)||(abs(roll)>60)) //Senses a fall in any orientation by factoring pitch and roll
        {
          state = 3;
          Time = millis();
        }
        if((millis()-Time)>100) // max.permissible Time between states 2 and 3
          state = 0;
    }
    if(state == 3){
        digitalWrite(11, LOW);
        digitalWrite(12, HIGH );
        if((abs(pitch)<60)&&(abs(roll)<60)) //Detects getting-up and prevents false alarm
        {
          count++;
          if(count>20)
          {
            state = 0;
            count = 0;
          }
        }
        if(((millis()-Time)/1000)>3) // 3 second window after fall between states 3 and 4
        {
          state = 4;
          ALERT = 1;
          //tone(9,250); // Fall, perhaps?
        }
   }
  if(state ==4)
    {
      digitalWrite(12, (blinkstate) ? LOW: HIGH);
      digitalWrite(9, (blinkstate) ? HIGH: LOW);
      blinkstate = !blinkstate;
      if(((millis()-Time)/1000)>20) // if patient is unresponsive for more than 20 secs, fall is confirmed-- necessary action taken via processing
      {
          state = 5;
          //tone(9,4000); // Generatepiezotone for distress
          digitalWrite(12, LOW);
      }
    }
}
void sendSMS() {
  Serial.println("ALERT ");
  GPRS.println("AT+CMGS=\"+918052896807\"");
  
  delay(500);
  
  GPRS.print("ALERT");
  GPRS.write( 0x1a ); // ctrl+Z character
  
  delay(500);
}
