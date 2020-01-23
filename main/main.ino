#include <Servo.h>
Servo servo1;
Servo servo2;

int servo_motor1 = 1;
int servo_motor2 = 2;
int power_motorL = 3;
int power_motorR = 4;
int arm_motor = 5;
int distance_sensor1 = A1;
int distance_sensor2 = A2;
int light_sensorL = A3;
int light_sensorR = A4;

class chassis {
  public:
  int traverse_speed = 50; //pwm of 0-255
  int rotate_speed = 10; //pwm of 0-255
  int rotate90_duration = 1000; //TODO: measure how long to rotate 90 deg
  void traverse(bool front) //if front = 1, go forward
  {
    if(front == 1)
    {
      analogWrite(power_motorR, traverse_speed);
      analogWrite(power_motorL, traverse_speed);
    }
    else //figure out how to drive motor backwards
    {
      analogWrite(power_motorR, -1 * rotate_speed);
      analogWrite(power_motorL, -1 * rotate_speed);
    }
  }
  void rotate(bool right) //if right = 1, turn rightward
  {
    if(right == 1)
    {
      analogWrite(power_motorR, -1 * traverse_speed);
      analogWrite(power_motorL, traverse_speed);
    }
    else //figure out how to drive motor backwards
    {
      analogWrite(power_motorR, traverse_speed);
      analogWrite(power_motorL, -1 * traverse_speed);
    }
  }
  void rotate90(bool right)
  {
    if(right == 1)
    {
      analogWrite(power_motorR, -1 * traverse_speed);
      analogWrite(power_motorL, traverse_speed);
    }
    else //figure out how to drive motor backwards
    {
      analogWrite(power_motorR, traverse_speed);
      analogWrite(power_motorL, -1 * traverse_speed);
    }
    delay(rotate90_duration);
    digitalWrite(power_motorR, LOW);
    digitalWrite(power_motorL, LOW);
  }
  void halt()
  {
    digitalWrite(power_motorR, LOW);
    digitalWrite(power_motorL, LOW);
  }
};

class rescue_arm {
  public:
  int motor_power = 50; //pwm of 0-255 TODO:callibrate this
  void hold()
  {
    analogWrite(arm_motor, motor_power);
  }
  void relax()
  {
    digitalWrite(arm_motor, 0);
  }
};

class sensors {
  public:
  int color_threshold = 490; //TODO: callibrate this
  int detect_white() //0 if both black, 1 if left white, 2 if right white, 3 if both white
  {
    if((analogRead(light_sensorL) < color_threshold) and  (analogRead(light_sensorR) < color_threshold)){return 0;}
    else if((analogRead(light_sensorL) > color_threshold) and  (analogRead(light_sensorR) < color_threshold)){return 1;}
    else if((analogRead(light_sensorL) < color_threshold) and  (analogRead(light_sensorR) > color_threshold)){return 2;}
    else if((analogRead(light_sensorL) > color_threshold) and  (analogRead(light_sensorR) > color_threshold)){return 3;}
  }
  float detect_distance1() //return distance from sensor 1 in cm 
  {
    float sensorValue = analogRead(distance_sensor1);
    float cm = 10650.08 * pow(sensorValue,-0.935) - 10;
    cm = roundf(cm);
    return cm;
  }
  float detect_distance2() //return distance from sensor 2 in cm 
  {
    float sensorValue = analogRead(distance_sensor2);
    float cm = 10650.08 * pow(sensorValue,-0.935) - 10;
    cm = roundf(cm);
    return cm;
  }
  void radar_angle(int angle) //TODO: test this
  {
    servo1.write(angle);
  }
};

chassis chassis;
rescue_arm rescue_arm;
sensors sensors;

void state0(int* state)
{
  //initialize
  servo1.write(90);
  servo2.write(90);
  chassis.halt();
  digitalWrite(arm_motor, LOW);
  delay(100);

  
  while(sensors.detect_white() != 2)
  {
    //if(sensors.detect_white() != 0){//throw color sensor calliration error}
    chassis.traverse(1);
  }
  delay(10);
  chassis.rotate90(1);
  delay(10);
  while(sensors.detect_white() != 3)
  {
    //if(sensors.detect_white() != 0){//throw color sensor calliration error}
    chassis.traverse(1);
  }
  chassis.halt();
  *state = 1;
}

void state1(int* state)
{
  int location = 3; //0 = at center, 1 = at left, 2 = at right, 3 = error
  float distance_to_wall = 100; //TODO: callibrate this
  int angle = 80;
  sensors.radar_angle(angle);
  while(angle < 100)
  {
    float distance1 = sensors.detect_distance1();
    float distance2 = sensors.detect_distance2();
    if ((distance1 < distance_to_wall) and (distance2 < distance_to_wall))
    {
      location = 0;
    }
    sensors.radar_angle(angle);
    angle++;
  }
  if(location == 0)
  {
    sensors.radar_angle(90);
    while((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
    {
      chassis.traverse(1);
    }
    chassis.halt();
    *state = 2;
  }
  else
  {
    bool at_left = 0;
    float cnt = 0;//TODO: callibrate this
    float distance_to_side = 100; //TODO: callibrate this
    sensors.radar_angle(0);
    while(cnt < 10000)
    {
      chassis.traverse(1);
      if((sensors.detect_distance1() < distance_to_side) and (sensors.detect_distance2() < distance_to_side))
      {
        at_left = 1;
        break;
      }
    }
    chassis.halt();
    if(at_left == 1)
    {
      chassis.rotate90(0);
      while((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
      {
        chassis.traverse(1);
      }
      chassis.halt();
      *state = 2;
    }
    else
    {
      chassis.rotate90(0);
      chassis.rotate90(0);
      int at_right;
      cnt = 0;
      while(cnt < 10000)
      {
        chassis.traverse(1);
        if((sensors.detect_distance1() < distance_to_side) and (sensors.detect_distance2() < distance_to_side))
        {
          at_right = 1;
          break;
        }
      }
      if(at_right == 1)
      {
        chassis.rotate90(0);
        while((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
        {
          chassis.traverse(1);
        }
        chassis.halt();
        *state = 2;
      }
      else
      {
        //throw victim not found error
      }
    }
  }
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

void state5(int* state)
{
  
}

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo_motor1); 
  servo2.attach(servo_motor2); 
  pinMode(power_motorL, OUTPUT); 
  pinMode(power_motorR, OUTPUT); 
  pinMode(arm_motor, OUTPUT);
  pinMode(distance_sensor1, INPUT); 
  pinMode(distance_sensor2, INPUT); 
  pinMode(light_sensorL, INPUT); 
  pinMode(light_sensorR, INPUT);
}

int state = 0;

void loop() {

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
