int photoresistor = A0;

void setup() {
  pinMode(photoresistor, INPUT); 
  Serial.begin(9600);
}


void loop() {
//initialize
  Serial.println(analogRead(photoresistor));
}
