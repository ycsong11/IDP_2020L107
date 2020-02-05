#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "Arduino.h"
#include "RescueArm.h"

RescueArm::RescueArm(int arm_motor)
{
    AFMS = Adafruit_MotorShield();
    arm = AFMS.getMotor(arm_motor); 
    motor_speed = 70;
    power_at_rest = 10;
    rescue_arm_duration = 3000;
}

void RescueArm::hold()
{
    arm->run(FORWARD);
    arm->setSpeed(motor_speed);
    delay(rescue_arm_duration);
    arm->setSpeed(0);
}

void RescueArm::relax()
{
    arm->run(BACKWARD);
    arm->setSpeed(motor_speed);
    delay(rescue_arm_duration);
    arm->setSpeed(0);
}
