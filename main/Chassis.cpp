#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"
#include "Chassis.h"

Chassis::Chassis(int power_motorL, int power_motorR)
{
    AFMS = Adafruit_MotorShield();
    motorL = AFMS.getMotor(power_motorL);
    motorR = AFMS.getMotor(power_motorR);
    traverse_speed = 200; //pwm of 0-255
    rotate_speed = 100;   //pwm of 0-255
    rotate90_duration = 2630;
}

void Chassis::traverse(bool front)
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

void Chassis::manual(int speed_L, int speed_R)
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

void Chassis::rotate(bool right)
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

void Chassis::rotate90(bool right)
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

void Chassis::rotate45(bool right)
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

void Chassis::rotate10(bool right)
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
    delay(rotate90_duration / 9);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(10);
    motorL->run(RELEASE);
    motorR->run(RELEASE);
}

void Chassis::halt()
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
