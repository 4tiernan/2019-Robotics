#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include <Pins_Main.h>
#include <PID.h>
#include <Functions.h>

class Motor {
public:
    Motor() {}
    Motor(int pwm, int dir1, int dir2) {
        pwmPin = pwm;
        dirPin1 = dir1;
        dirPin2 = dir2;  
        motor_pwm = 0; 
    }

    void init();
    void drive(int speed);
    void Update(double Setpoint, double Motor_rpm);
    void stop();

private:
    int pwmPin;
    int dirPin1;
    int dirPin2;
    int motor_pwm;

    PID pid = PID(0.05,0,1000);
};

class MotorDriver {
public:
    void init();
    void drive(int Direction, int Rotation, int Power, int Output, int Orbit, int M1_rpm, int M2_rpm, int M3_rpm, int M4_rpm);
    void stop();

private:
    Motor motorA = Motor(APWM, APIN1, APIN2);
    Motor motorB = Motor(BPWM, BPIN1, BPIN2);
    Motor motorC = Motor(CPWM, CPIN1, CPIN2);
    Motor motorD = Motor(DPWM, DPIN1, DPIN2);
};

#endif