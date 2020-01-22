#include <Servo.h>

int servo_motor1 =
int servo_motor2 =
int power_motor1 =
int power_motor2 =
int arm_motor =
int distance_sensor1 =
int distance_sensor2 =
int light_sensor =

class chassis {
  public:
  
}

class rescue_arm {
  public:

}

class sensors {
  public:

}

void state0(int* state)
{
  
}

void state1(int* state)
{
  
}

void state2(int* state)
{
  
}

void state3(int* state)
{
  
}

void state4(int* state)
{
  
}

void state4(int* state)
{
  
}

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo_motor1); 
  servo2.attach(servo_motor2); 
  pinMode(power_motor1, OUTPUT); 
  pinMode(power_motor2, OUTPUT); 
  pinMode(arm_motor, OUTPUT);
  pinMode(distance_sensor1, INPUT); 
  pinMode(distance_sensor2, INPUT); 
  pinMode(light_sensor, INPUT); 
}

int state = 0;
void loop() {
//initialize
servo1.write(0);
servo2.write(0);
digitalWrite(power_motor1, LOW);
digitalWrite(power_motor2, LOW);
digitalWrite(arm_motor, LOW);

  switch(state){
    case 0:
      state0(&state); 
      break;
    case 1:
      state1(&state);
      break;
    case 2:
      state2(&state);
      break;
    case 3:
      state3(&state);
      break;
    case 4:
      state4(&state);
      break;
    case 5:
      state5(&state);
      break;
  }
}
