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
int arm_motor = 4;
int distance_sensor1 = A0;
int distance_sensor2 = A1;
int light_sensorL = A2;
int light_sensorR = A3;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = AFMS.getMotor(power_motorL);
Adafruit_DCMotor *motorR = AFMS.getMotor(power_motorR);
Adafruit_DCMotor *motor_arm = AFMS.getMotor(arm_motor);

class Chassis
{
public:
  int traverse_speed = 200; //pwm of 0-255
  int rotate_speed = 100;   //pwm of 0-255
  int rotate90_duration = 2725;
  int scan90_duration = 6000;

  void traverse(bool front)
  {
    motorL->setSpeed(100);
    motorR->setSpeed(100);
    if (front == 1)
    {
      motorL->run(FORWARD);
      motorR->run(FORWARD);
      delay(100);
      motorL->setSpeed(traverse_speed + 5);
      motorR->setSpeed(traverse_speed);
    }
    else
    {
      motorL->run(BACKWARD);
      motorR->run(BACKWARD);
      delay(100);
      motorL->setSpeed(traverse_speed + 7);
      motorR->setSpeed(traverse_speed);
    }
  }

  void manual(int speed_L, int speed_R)
  {
    if (speed_L >= 0)
    {
      motorL->run(FORWARD);
    }
    else
    {
      motorL->run(BACKWARD);
      speed_L = -speed_L;
    }
    if (speed_R >= 0)
    {
      motorR->run(FORWARD);
    }
    else
    {
      speed_R = -speed_R;
      motorR->run(BACKWARD);
    }
    delay(1);
    motorL->setSpeed(speed_L);
    motorR->setSpeed(speed_R);
  }

  void rotate(bool right)
  {
    motorL->setSpeed(rotate_speed);
    motorR->setSpeed(rotate_speed);
    if (right == 1)
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
    if (right == 1)
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

  void rotate45(bool right)
  {
    motorL->setSpeed(rotate_speed);
    motorR->setSpeed(rotate_speed);
    if (right == 1)
    {
      motorL->run(FORWARD);
      motorR->run(BACKWARD);
    }
    else //figure out how to drive motor backwards
    {
      motorL->run(BACKWARD);
      motorR->run(FORWARD);
    }
    delay(rotate90_duration / 2);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(10);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
  }

  void scan90(bool right)
  {
    motorL->setSpeed(50);
    motorR->setSpeed(50);
    if (right == 1)
    {
      motorL->run(FORWARD);
      motorR->run(BACKWARD);
    }
    else //figure out how to drive motor backwards
    {
      motorL->run(BACKWARD);
      motorR->run(FORWARD);
    }
    delay(scan90_duration);
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

class Sensors
{
public:
  int distance1 = distance_sensor1;
  int distance2 = distance_sensor2;
  int lightL = light_sensorL;
  int lightR = light_sensorR;
  int color_threshold = 490;

  int detect_white() //0 if both black, 1 if left white, 2 if right white, 3 if both white
  {
    if ((analogRead(lightL) < color_threshold) and (analogRead(lightR) < color_threshold))
    {
      return 0;
    }
    else if ((analogRead(lightL) > color_threshold) and (analogRead(lightR) < color_threshold))
    {
      return 1;
    }
    else if ((analogRead(lightL) < color_threshold) and (analogRead(lightR) > color_threshold))
    {
      return 2;
    }
    else if ((analogRead(lightL) > color_threshold) and (analogRead(lightR) > color_threshold))
    {
      return 3;
    }
  }

  float detect_distance1()
  {
    float x = analogRead(distance1);
    float cm = -0.00000177 * pow(x, 3) + 0.002202 * pow(x, 2) - 0.9961 * x + 187.5; // -1.77e-06 x^3 + 0.002202 x^2 - 0.9961 x + 187.5
    cm = roundf(cm);
    return cm;
  }

  float detect_distance2()
  {
    float sensorValue = analogRead(distance2);
    float cm = 10650.08 * pow(sensorValue, -0.935) - 10;
    cm = roundf(cm);
    return cm;
  }

  void radar_angle(int angle) // positive: sensor face right
  {
    servo1.write(angle / 1.4 + 170 / 2 + 10); // reduction ratio = 1.5
  }

  float reference_distance(float time, int scan90_duration)
  {
    float pi = 3.14159265358979323846;
    float radian_per_ms = pi / 2 / scan90_duration;
    float radian = time * radian_per_ms;
    float centre_to_sensor = 17.0;
    float half_width = 120.0;
    float depth = 92.0;
    if (radian < 0.9167)
    {
      return (depth / cos(radian) - centre_to_sensor);
    }
    else
    {
      return (half_width / cos(pi / 2 - radian) - centre_to_sensor);
    }
  }
};

class RescueArm
{
public:
  int motor_speed = 70;
  int power_at_rest = 10;
  int rescue_arm_duration = 3000;

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

Chassis chassis;
Sensors sensors;
RescueArm rescue_arm;
unsigned long start_time;

void state0(int *state)
{
  //initialize
  motor_arm->setSpeed(0);
  servo1.write(90);
  servo2.write(90);
  chassis.halt();
  delay(100);
  chassis.traverse(1);
  delay(2000);
  start_time = millis();
  while ((millis() - start_time) < 6000)
  {
    if (sensors.detect_white() == 2)
    {
      chassis.manual(240, 1);
      delay(400);
    }
    else
    {
      chassis.manual(240, 255);
    }
    delay(1);
  }
  chassis.halt();
  delay(1000);
  while (1)
  {
    if (sensors.detect_white() == 2)
    {
      chassis.manual(150, 1);
      delay(150);
    }
    else if (sensors.detect_white() == 1)
    {
      chassis.manual(1, 150);
      delay(150);
    }
    else if (sensors.detect_white() == 0)
    {
      chassis.manual(255, 255);
      delay(50);
    }
    else
    {
      chassis.manual(0, 0);
      break;
    }
    delay(1);
  }
  *state = 1;
}

void state1(int *state)

{
  int location = 3;             //0 = at center, 1 = at left, 2 = at right, 3 = error
  float distance_to_wall = 100; //TODO: callibrate this
  int angle = 80;
  servo1.write(angle);
  while (angle < 100)
  {
    float distance1 = sensors.detect_distance1();
    float distance2 = sensors.detect_distance2();
    if ((distance1 < distance_to_wall) and (distance2 < distance_to_wall))
    {
      location = 0;
    }
    servo1.write(angle);
    angle++;
    delay(10);
  }
  if (location == 0)
  {
    servo1.write(90);
    while ((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
    {
      chassis.traverse(1);
    }
    chassis.halt();
    *state = 2;
  }
  else
  {
    bool at_left = 0;
    float cnt = 0;                //TODO: callibrate this
    float distance_to_side = 100; //TODO: callibrate this
    servo1.write(0);
    while (cnt < 10000)
    {
      chassis.traverse(1);
      if ((sensors.detect_distance1() < distance_to_side) and (sensors.detect_distance2() < distance_to_side))

      {
        at_left = 1;
        break;
      }
    }
    chassis.halt();
    if (at_left == 1)
    {
      chassis.rotate90(0);
      while ((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
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
      while (cnt < 10000)
      {
        chassis.traverse(1);
        if ((sensors.detect_distance1() < distance_to_side) and (sensors.detect_distance2() < distance_to_side))
        {
          at_right = 1;
          break;
        }
      }
      if (at_right == 1)
      {
        chassis.rotate90(0);
        while ((sensors.detect_distance1() > 5) and (sensors.detect_distance2() > 5))
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

void approach1()
{
  // approach 1 - scan the region by rotating the robot
  float accu = 0;
  int count = 0;
  chassis.rotate(0);
  float start_time = millis();
  while ((millis() - start_time < chassis.rotate90_duration))
  {
    float diff = sensors.reference_distance((millis() - start_time), chassis.rotate90_duration) - sensors.detect_distance1();
    /*
    Serial.print("Ref: ");
    Serial.println(sensors.reference_distance((millis() - start_time), chassis.rotate90_duration));
    Serial.print("Actual: ");
    Serial.println(sensors.detect_distance1());
    */
    accu += diff;
    count += 1;
    if (count == 2)
    {
      if (accu > 80)
      {
        chassis.halt();
        chassis.manual(50,-50);
        delay(100);
        break;
      }
      count = 0;
      accu = 0;
    }
  }
}

void approach2()
{
  // approach 2 - detect the victim sideways by cruising
  sensors.radar_angle(90);
  bool swc = 0; // 1 if there's a victim in the front
  for (int angle = 80; angle < 100; angle++)
  {
    float diff = 120.0 - sensors.detect_distance1();
    if (diff > 800)
    {
      chassis.rotate90(0);
      chassis.traverse(0);
      delay(2000);
      chassis.halt();
      swc = 1;
      break;
    }
  }
  sensors.radar_angle(180);
  chassis.manual(50, 50);
  start_time = millis();
  float record_time1 = 10000; // default value is the max time duration
  while (millis() - start_time < record_time1)
  {
    if (sensors.detect_distance1() < 100)
    {
      record_time1 = millis() - start_time;
      delay(500);
      chassis.halt();
      break;
    }
  }
  chassis.rotate90(1);
  sensors.radar_angle(90);
  start_time = millis();
  chassis.traverse(1);
  float record_time2 = 10000;
  while (millis() - start_time < record_time2)
  {
    if (sensors.detect_distance1() < 20)
    {
      delay(500);
      record_time2 = millis() - start_time;
      chassis.halt();
      break;
    }
  }
  delay(5000); // time for LED reactions
  rescue_arm.hold();
  delay(1000);
  chassis.rotate90(1);
  chassis.rotate90(1);
  chassis.traverse(1);
  delay(record_time2);
  chassis.halt();
  chassis.rotate90(0);
  chassis.manual(50, 50);
  delay(record_time1);
  chassis.halt();
}

void state2(int *state)
{
  int swc = 1;
  chassis.manual(75, 75);
  start_time = millis();
  while ((millis() - start_time) < 6000)
  {
    if (sensors.detect_white() == 0)
    {
      chassis.manual(75, 75);
    }
    else if (sensors.detect_white() == 1)
    {
      chassis.manual(0, 75);
      delay(200);
      chassis.manual(-50, -50);
      delay(200);
    }
    else if (sensors.detect_white() == 2)
    {
      chassis.manual(75, 0);
      delay(200);
      chassis.manual(-50, -50);
      delay(200);
    }
    else
    {
      if (swc == 0)
      {
        break;
      }
      chassis.manual(-25, -25);
      delay(200);
      swc = 0;
    }
    delay(1);
  }
  chassis.halt();
  while (1)
  {
  }
  chassis.traverse(1);
  bool swc1 = 0;
  bool swc2 = 0;
  while (swc2 == 0)
  {
    if (sensors.detect_white() == 3)
    {
      swc1 = 1;
    }
    if ((sensors.detect_white() == 0) and (swc1 == 1))
    {
      swc2 = 1;
    }
  }
  int i = 0;
  while (i < 30)
  {
    if (sensors.detect_white() == 2)
    {
      chassis.manual(240, 1);
      delay(2);
    }
    else
    {
      chassis.manual(240, 255);
    }
    delay(1);
    i++;
  }
  i = 0;
  //chassis.rotate45(1);
  //chassis.manual(140, 150);
  /*
  while(1)
  {
    if(sensors.detect_white() == 2)
    {
      chassis.rotate45(1);
      break;
    }
  }*/

  while (i < 40)
  {
    if ((sensors.detect_white() == 2) or (sensors.detect_white() == 3))
    {
      chassis.rotate90(1);
    }
    else
    {
      chassis.manual(255, 255);
    }
    delay(1);
    i++;
  }
  //while(1){chassis.halt();}
  /*
  motorL->setSpeed(50);
  motorR->setSpeed(50);*/
  //int swc = 1;

  while (1) //i < 50)
  {
    if (sensors.detect_white() == 2)
    {
      chassis.manual(255, 1);
    }
    else if (sensors.detect_white() == 1)
    {
      chassis.manual(1, 255);
    }
    else if (sensors.detect_white() == 0)
    {
      chassis.manual(255, 255);
    }
    else
    {
      chassis.manual(0, 0);
      break;
    }
    delay(1);
    i++;
  }
  *state = 3;
}

void state3(int *state)
{
  //chassis.traverse(1);
  int i = 0;
  bool swc = 1;
  while (i < 50)
  {
    if (sensors.detect_white() == 1)
    {
      chassis.rotate90(0);
    }
    else
    {
      chassis.manual(255, 255);
    }
    delay(1);
    i++;
  }
  /*
  motorL->setSpeed(50);
  motorR->setSpeed(50);*/
  //int swc = 1;

  while (1)
  {
    if (sensors.detect_white() == 2)
    {
      chassis.manual(255, 1);
    }
    else if (sensors.detect_white() == 1)
    {
      chassis.manual(1, 255);
    }
    else if (sensors.detect_white() == 0)
    {
      chassis.manual(255, 255);
    }
    else
    {
      chassis.manual(0, 0);
      break;
    }

    delay(1);
    i++;
  }

  *state = 1;
}

void state4(int *state)
{
}

void state5(int *state)
{
}

void setup()
{
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

int state = 0;

void loop()
{
  bool swc = 1;
  if (swc)
  {
    delay(3000);
  }
  approach1();
  /*
  while (1)
  {
    Serial.print("\nReading: ");
    Serial.print(analogRead(distance_sensor1));
    Serial.print("\nConversion: ");
    Serial.print(sensors.detect_distance1());
    delay(500);
  }
  */
  swc == 0;
  chassis.halt();
  while (1)
  {
  }
}
