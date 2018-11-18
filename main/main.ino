//library included
#include <SoftwareSerial.h>
#include<dht.h>
dht DHT;
SoftwareSerial GPRS(6,7);

//pin declaration'
const int zpin = A0; // z-axis
const int ypin = A1; // y-axis
const int xpin = A2; // x-axis
const int Hallpin = A3; //Hall  sensor
#define DHT11_PIN A4
const int mq_pin=A5;  // gas sensor

float Xval=0.0, Yval=0.0, Zval=0.0,Hallval=0.0;//for accelerometer
float Anet=0.0, pitch=0.0, roll=0.0;

int TYPE = 0;

//below are the calibration values for the 3 axes, such that in stable condition the values are (1g, 0g, 0g)
float xZero = 0;
float yZero = 0;
float zZero = 0;
                           
int RL_VALUE=5;                                     //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR=9.83;                     //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,//which is derived from the chart in datasheet
int CALIBARAION_SAMPLE_TIMES=10;                    //define how many samples you are going to take in the calibration phase
int CALIBRATION_SAMPLE_INTERVAL=200;                //define the time interal(in milisecond) between each samples in the//cablibration phase
int READ_SAMPLE_INTERVAL=10;                        //define how many samples you are going to take in normal operation
int READ_SAMPLE_TIMES=5;                            //define the time interal(in milisecond) between each samples in //normal operation
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohm

void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  delay(1000);
  Ro = MQCalibration(mq_pin);                         //Calibrating the sensor. Please make sure the sensor is in clean air
  
  //RGB led work(PENDING)
  pinMode(2, OUTPUT); //Red
  pinMode(3, OUTPUT); // Ground
  pinMode(4, OUTPUT); //Green
  pinMode(5, OUTPUT); //Blue
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  GPRS.begin(9600);
  Serial.println("CALIBRATING");
  GPRS.println("AT+CMGF=1");
}

void loop() {
    while(GPRS.available()) {
    Serial.write(GPRS.read());
  }
  delay(1000);
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
    long LPG = 0;
    long CO = 0;
    long SMOKE = 0;
    LPG = MQGetGasPercentage(MQRead(mq_pin)/Ro,GAS_LPG);
    CO = MQGetGasPercentage(MQRead(mq_pin)/Ro,GAS_CO);
    SMOKE = MQGetGasPercentage(MQRead(mq_pin)/Ro,GAS_SMOKE);
    
    //Threshold
    if(temp>=30.00 or(humid<=40.00 and humid>=0.0)){
      TYPE=1;
      digitalWrite(13,HIGH);
      digitalWrite(2,HIGH);
      sendSMS();
    }else if(Anet>0.78){
      TYPE=2;
      digitalWrite(13,HIGH);
      digitalWrite(3,HIGH);
      sendSMS();
    }else if(SMOKE>=20 or LPG>=50 or CO>=15){
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
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro                                        
  return val;                                                      //according to the chart in the datasheet 

} 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
long MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}
