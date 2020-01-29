#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Servo servo1;
Servo servo2;

int servo_motor1 = 9;
int servo_motor2 = 10;
int power_motorL = 1;
int power_motorR = 2;
int arm_motor = 3;
int distance_sensor1 = A1;
int distance_sensor2 = A2;
int light_sensorL = A2;
int light_sensorR = A3;


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(power_motorL);
Adafruit_DCMotor *motorR = AFMS.getMotor(power_motorR);
Adafruit_DCMotor *motor_arm = AFMS.getMotor(arm_motor);

class chassis {
  public:
  int traverse_speed = 255; //pwm of 0-255
  int rotate_speed = 100; //pwm of 0-255
  int rotate90_duration = 2675; //TODO: measure how long to rotate 90 deg
  void traverse(bool front) //if front = 1, go forward
  {
    motorL->setSpeed(100);
    motorR->setSpeed(100);
    if(front == 1)
    {
      motorL->run(FORWARD);
      motorR->run(FORWARD);
      delay(100);
      motorL->setSpeed(traverse_speed - 6);
      motorR->setSpeed(traverse_speed);
    }
    else 
    {
      motorL->run(BACKWARD);
      motorR->run(BACKWARD);
      delay(100);
      motorL->setSpeed(traverse_speed - 6);
      motorR->setSpeed(traverse_speed);
    }
  }
  void rotate(bool right) //if right = 1, turn rightward
  {
    motorL->setSpeed(rotate_speed);
    motorR->setSpeed(rotate_speed);
    if(right == 1)
    {
      motorL->run(FORWARD);
      motorR->run(BACKWARD);
    }
    else 
    {
      motorL->run(BACKWARD);
      motorR->run(FORWARD);
    }
  }
  void rotate90(bool right)
  {
    motorL->setSpeed(rotate_speed);
    motorR->setSpeed(rotate_speed);
    if(right == 1)
    {
      motorL->run(FORWARD);
      motorR->run(BACKWARD);
    }
    else //figure out how to drive motor backwards
    {
      motorL->run(BACKWARD);
      motorR->run(FORWARD);
    }
    delay(rotate90_duration);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(10);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
  }
  void halt()
  {
    motorL->setSpeed(100);
    motorR->setSpeed(100);
    delay(100);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    //delay(10);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
  }
};

class rescue_arm {
  public:
  int motor_speed = 50; //pwm of 0-255 TODO:callibrate this
  int power_at_rest = 10;
  int rescue_arm_duration = 500; //the time before the motor reduces power
  void hold()
  {
    motor_arm->run(FORWARD);
    motor_arm->setSpeed(motor_speed);
    delay(rescue_arm_duration);
    motor_arm->setSpeed(0);
  }
  void relax()
  {
    motor_arm->run(BACKWARD);
    motor_arm->setSpeed(motor_speed);
    delay(rescue_arm_duration);
    motor_arm->setSpeed(0);
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

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo_motor1); 
  servo2.attach(servo_motor2); 
  pinMode(distance_sensor1, INPUT); 
  pinMode(distance_sensor2, INPUT); 
  pinMode(light_sensorL, INPUT); 
  pinMode(light_sensorR, INPUT);
  AFMS.begin();
  Serial.begin(9600);
}


void loop() {
//initialize
  motorL->run(RELEASE);
  motorR->run(RELEASE);
  Serial.print(analogRead(light_sensorL));
  Serial.print("   ");
  Serial.print(analogRead(light_sensorR));
  Serial.print("   ");
  Serial.println(sensors.detect_white());
  delay(50);
}
