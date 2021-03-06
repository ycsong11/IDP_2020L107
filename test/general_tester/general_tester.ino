#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Servo servo1;
Servo servo2;

int servo_motor1 = 1;
int servo_motor2 = 2;
int power_motorL = 1;
int power_motorR = 2;
int arm_motor = 5;
int distance_sensor1 = A1;
int distance_sensor2 = A2;
int light_sensorL = A3;
int light_sensorR = A4;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(power_motorL);
Adafruit_DCMotor *motorR = AFMS.getMotor(power_motorR);


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
  int power_at_rest = 10;
  int rescue_arm_duration = 500; //the time before the motor reduces power
  void hold()
  {
    analogWrite(arm_motor, motor_power);
    delay(rescue_arm_duration);
    analogWrite(arm_motor, power_at_rest);
  }
  void relax()
  {
    analogWrite(arm_motor, -1 * motor_power);
    delay(rescue_arm_duration);
    digitalWrite(arm_motor, LOW);
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

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo_motor1); 
  servo2.attach(servo_motor2); 
  //pinMode(power_motorL, OUTPUT); 
  //pinMode(power_motorR, OUTPUT); 
  pinMode(arm_motor, OUTPUT);
  pinMode(distance_sensor1, INPUT); 
  pinMode(distance_sensor2, INPUT); 
  pinMode(light_sensorL, INPUT); 
  pinMode(light_sensorR, INPUT);
  AFMS.begin();
  Serial.begin(9600);
}

chassis chassis;
rescue_arm rescue_arm;
sensors sensors;

void loop() {
    //Serial.println("Test no. to run:");
    //string num_of_test = Serial.readString();
    motorL->setSpeed(100);
    motorR->setSpeed(100);
    Serial.println("M1 running");
    motorL->run(FORWARD);
    motorR->run(FORWARD);
    delay(1000);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
}
