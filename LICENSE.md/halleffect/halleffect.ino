int sensor = A6; //sensor pin
int val; //numeric variable

void setup()
{
  Serial.begin(9600);
  pinMode(sensor, INPUT); //set sensor pin as input
}

void loop()
{
  val = analogRead(sensor); //Read the sensor
  Serial.print("Value = ");
  Serial.println(val);
  delay(2000);
}
