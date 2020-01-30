#ifndef Chassis_h
#define Chassis_h

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"

class Chassis
{
    public:
        Chassis(int power_motorL, int power_motorR);
        void traverse(bool front);
        void manual(int speed_L, int speed_R);
        void rotate(bool right);
        void rotate90(bool right);
        void rotate45(bool right);
        void rotate10(bool right);
        void halt();
    private:
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *motorL;
        Adafruit_DCMotor *motorR;
        int traverse_speed;
        int rotate_speed;
        int rotate90_duration;
};

#endif