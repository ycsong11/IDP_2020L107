#ifndef Sensors_h
#define Sensors_h

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"

class Sensors
{
    public:
        Sensors(int distance_sensor1, int distance_sensor2, int light_sensorL, int light_sensorR);
        int detect_white();
        float detect_distance1();
        float detect_distance2();
    private:
        int distance1;
        int distance2;
        int lightL;
        int lightR;
        int color_threshold;

};

#endif