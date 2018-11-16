//library included
#include <SoftwareSerial.h>
#include<dht.h>
#include <MQ2.h>
//#include <LiquidCrystal.h>
dht DHT;
SoftwareSerial GPRS(6,7);
//LiquidCrystal lcd(8,9,10,11,12,13);
//pin declaration'
const int zpin = A0; // z-axis
const int ypin = A1; // y-axis
const int xpin = A2; // x-axis
const int Hallpin = A3; //Hall  sensor
#define DHT11_PIN A4
const int mq2pin = A5; // gas sensor

MQ2 mq2(mq2pin);
float Xval=0.0, Yval=0.0, Zval=0.0,Hallval=0.0;//for accelerometer
float Anet=0.0, pitch=0.0, roll=0.0;

int TYPE = 0;

//below are the calibration values for the 3 axes, such that in stable condition the values are (1g, 0g, 0g)
float xZero = 0;
float yZero = 0;
float zZero = 0;

//for gas sensor
int LPG,CO,SMOKE;
float sensor_volt;
float RS_air; //get value of Rs in GAS
float R0;
float ratio; //Get ratio RS_GAS/RS_air

void setup() {
  //lcd.begin(16, 2);
  pinMode(13,OUTPUT);
  mq2.begin();
  
  //RGB led work(PENDING)
  pinMode(2, OUTPUT); //Red
  pinMode(3, OUTPUT); // Ground
  pinMode(4, OUTPUT); //Green
  pinMode(5, OUTPUT); //Blue
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  digitalWrite(4,LOW);
  GPRS.begin(9600);
  Serial.begin(9600);
  GPRS.println("AT+CMGF=1");
}

void loop() {
    while(GPRS.available()) {
    Serial.write(GPRS.read());
  }
  //lcd.setCursor(0,0);    
  //lcd.print("dfggg");
  delay(2000);
    //Taking input from sensors
    Xval = (analogRead(xpin)- xZero)/1024;
    Yval = (analogRead(ypin) - yZero)/1024;
    Zval = (analogRead(zpin) - zZero)/1024;
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
    float temp=0.0;
    temp=DHT.temperature;
    float humid = 0.0;
    humid=DHT.humidity;
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
    
    //Threshold
    if(temp>=30.00 or(humid<=40.00 and humid>=0.0)){
      TYPE=1;
      digitalWrite(13,HIGH);
      digitalWrite(2,HIGH);
      sendSMS();
    }else if(Anet>0.8){
      TYPE=2;
      digitalWrite(13,HIGH);
      digitalWrite(3,HIGH);
      sendSMS();
    }else if(SMOKE>=1 or LPG>=1 or CO>=1){
      TYPE=3;
      digitalWrite(13,HIGH);
      digitalWrite(4,HIGH);
      sendSMS();
    }else if(Hallval<=100){
      TYPE=4;
      digitalWrite(13,HIGH);
      digitalWrite(5,HIGH);
      sendSMS();
    }else{
      digitalWrite(13,LOW);
      digitalWrite(2,LOW);
      digitalWrite(3,LOW);
      digitalWrite(4,LOW);
      digitalWrite(5,LOW);
      TYPE=0;
    }
    
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
    Serial.print("Anet = ");
    Serial.print(Anet);
    Serial.print('\t');
    Serial.print(" Pitch = ");
    Serial.print(pitch);
    Serial.print('\t');
    Serial.print(" Roll = ");
    Serial.println(roll);
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
    
    delay(2500);
}

void sendSMS() {
  Serial.println("ALERT ");
  GPRS.println("AT+CMGS=\"+917897602880\""); //Akash
  
  delay(500);
  Serial.println("ALERT ");
  if(TYPE==1){
    GPRS.println("TEMPERATURE ALERT");
  }else if(TYPE==2){
    GPRS.println("FALL DETECTED");
  }else if(TYPE==3){
    GPRS.println("HARMFUL GASES NEARBY");
  }else if(TYPE==4){
    GPRS.println("HIGH MAGNETIC FIELD DETECTED");
  }
  GPRS.write( 0x1a ); // ctrl+Z character
  
  delay(500);
}
