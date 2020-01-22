
int distance;

void setup()
{
  Serial.begin(9600);
  pinMode(A1, INPUT);
}

void loop()
{
  float sensorValue = analogRead(A0);
  float cm = 10650.08 * pow(sensorValue,-0.935) - 10;
  cm = roundf(cm);
  Serial.print("\nDistance in centimeters: ");
  Serial.print(cm);  
  delay(500); 
}
