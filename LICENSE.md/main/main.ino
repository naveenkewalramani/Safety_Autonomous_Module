//#include <SoftwareSerial.h>
#include<dht.h>
#include <MQ2.h>
dht DHT;
#define DHT11_PIN A5
const int xpin = A3; // x-axis
const int ypin = A2; // y-axis
const int zpin = A1; // z-axis
const int Hallpin = A4; //Hall  sensor
const int mq2pin = A6; // gas sensor
MQ2 mq2(mq2pin);
float Xval, Yval, Zval,Hallval;//for accelerometer
float Anet, pitch, roll;
//unsigned long time = 0; // variable to compute timedifferences
unsigned long steps = 0; // number of steps taken
unsigned long steptime; // variable to compute time between steps
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
  pinMode(A7,OUTPUT);
  Serial.begin(9600);
  mq2.begin();
}

void loop() {
    
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
    if(temp>=30.00 or humid<=40.00)
      digitalWrite(A7,HIGH);
    else
      digitalWrite(A7,LOW);
    //Printing 
    Serial.print(" Temperature = ");
    Serial.print(temp);
    Serial.print(" Humidity = ");
    Serial.print(humid);
    Serial.print("Xval = ");
    Serial.print(Xval);
    Serial.print("Yval = ");
    Serial.print(Yval);
    Serial.print("Zval = ");
    Serial.print(Zval);
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print(" Roll = ");
    Serial.print(roll);
    Serial.print(" State = ");
    Serial.print(state);
    Serial.print(" HallEffect = ");
    Serial.print(Hallval);
    Serial.print(" LPG = ");
    Serial.print(LPG);
    Serial.print(" CO =");
    Serial.print(CO);
    Serial.print(" SMOKE = ");
    Serial.println(SMOKE);
    //Serial.print(" sensor_volt = ");
    //Serial.print(sensor_volt);
    //Serial.print(" RS_air = ");
    //Serial.print(RS_air); 
    //Serial.print(" R0 = ");
    //Serial.print(R0);
    //Serial.print(" Ratio = ");
    //Serial.println(ratio);
    
    delay(1000);
}
