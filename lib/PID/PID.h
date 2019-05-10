
#ifndef PID_h
#define PID_h
#include <Arduino.h>

class PID {
public:
    PID(double kp, double ki, double kd){
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
    double Compute(double Input, double Setpoint){
        /*How long since we last calculated*/
        unsigned long now = micros();
        double timeChange = (double)(now - lastTime);
        /*Compute all the working error variables*/
        double error = Setpoint - Input;
        errSum += (error * timeChange);
        double dErr = (error - lastErr) / timeChange; 
        /*Compute PID Output*/
        /*Remember some variables for next time*/
        lastErr = error;
        lastTime = now;
        return Kp * error + Ki * errSum + Kd * dErr;
    }

    double Kp, Ki, Kd, lastErr, errSum;
    unsigned long lastTime;
};

#endif