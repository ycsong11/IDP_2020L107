#ifndef RescueArm_h
#define RescueArm_h

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"

class RescueArm
{
    public:
        RescueArm(int arm_motor);
        void hold();
        void relax();
    private:
        Adafruit_MotorShield AFMS;
        Adafruit_DCMotor *arm;
        int motor_speed;
        int power_at_rest;
        int rescue_arm_duration;
};

#endif