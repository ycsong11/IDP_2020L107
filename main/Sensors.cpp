#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"
#include "Sensors.h"

Sensors::Sensors(int distance_sensor1, int distance_sensor2, int light_sensorL, int light_sensorR)
{
    distance1 = distance_sensor1;
    distance2 = distance_sensor2;
    lightL = light_sensorL;
    lightR = light_sensorR;
    color_threshold = 490;
}

int Sensors::detect_white() //0 if both black, 1 if left white, 2 if right white, 3 if both white
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

float Sensors::detect_distance1()
{
    float sensorValue = analogRead(distance1);
    float cm = 10650.08 * pow(sensorValue, -0.935) - 10;
    cm = roundf(cm);
    return cm;
}

float Sensors::detect_distance2()
{
    float sensorValue = analogRead(distance2);
    float cm = 10650.08 * pow(sensorValue, -0.935) - 10;
    cm = roundf(cm);
    return cm;
}