
int sensor = A0;

float convert(sensor)
{
  float x = analogRead(sensor);
  float cm = -0.00000177 * pow(x, 3) + 0.002202 * pow(x, 2) - 0.9961 * x + 187.5; // -1.77e-06 x^3 + 0.002202 x^2 - 0.9961 x + 187.5
  cm = roundf(cm);
  return cm;
}

void setup()
{
  Serial.begin(9600);
  pinMode(sensor, INPUT);
}

void loop()
{
  Serial.print("Raw: ");
  Serial.println(analogRead(sensor));
  Serial.print("Convert: ");
  Serial.println(convert(sensor));
  delay(500);
}
