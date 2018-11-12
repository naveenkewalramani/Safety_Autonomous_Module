#include <SoftwareSerial.h>

SoftwareSerial GPRS(7, 8);
boolean state, lastState;

void setup()
{
  pinMode(2, INPUT_PULLUP);
  state = digitalRead(2);
  lastState = state;
  
  GPRS.begin(9600);
  Serial.begin(9600);
  
  GPRS.println("AT+CMGF=1");
  
  delay(1000);
}

void loop()
{
  while(GPRS.available()) {
    Serial.write(GPRS.read());
  }

  lastState = state;
  state = digitalRead(2);
  
  if ( state != lastState ) {
    sendSMS();
  }
  
  delay(500);
}

void sendSMS() {
  Serial.print("Switch was turned ");
  Serial.println(state ? "on" : "off");
  
  GPRS.println("AT+CMGS=\"+64123456789\"");
  
  delay(500);
  
  GPRS.print("Switch was turned ");
  GPRS.println(state ? "on" : "off");
  GPRS.write( 0x1a ); // ctrl+Z character
  
  delay(500);
}
