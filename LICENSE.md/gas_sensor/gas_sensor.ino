void setup() {
  Serial.begin(9600);
}

void loop() {
  float sensor_volt;
  float RS_air; //get value of Rs in GAS
  float R0;
  float ratio; //Get ratio RS_GAS/RS_air
  float sensorValue=0.0;
  for(int x=0;x<100;x++){
    sensorValue = sensorValue + analogRead(A0);
  }
  sensorValue = sensorValue/100.0;
  sensor_volt = (float)sensorValue/1024*5.0;
  RS_air = (5.0 -sensor_volt)/sensor_volt;
  R0 = RS_air/9.8;
  ratio = RS_air/R0;
  
  Serial.print("sensor_volt = ");
  Serial.println(sensor_volt);
  //Serial.println("V");
  
  Serial.print("RS_air = ");
  Serial.println(RS_air);
 
  Serial.print("R0 = ");
  Serial.println(R0);

  Serial.print("Ratio = ");
  Serial.println(ratio);
  
  delay(1000);

}
