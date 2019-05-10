#include "MotorDriver.h"

#define PWM_FREQUENCY 29296.875
#define MAX_PWM 2047
unsigned long lastTime;
double errSum, lastErr, Output;

void Motor::init() {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    analogWriteFrequency(pwmPin, PWM_FREQUENCY);
}
void Motor::Update(double Setpoint, double Motor_rpm){
    motor_pwm = constrain(abs(motor_pwm)+pid.Compute(abs(Motor_rpm), abs(Setpoint)), 0, MAX_PWM) * sign(Setpoint);
    drive(motor_pwm);
}
void Motor::drive(int speed) {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);
    if(speed > 0){
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
        analogWrite(pwmPin, speed);
    }else if(speed < 0){
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        analogWrite(pwmPin, -speed);
    }else if(speed == 0){
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);
        digitalWrite(pwmPin, HIGH);
    }
}

void MotorDriver::init() {
    motorA.init();
    motorB.init();
    motorC.init();
    motorD.init();
    analogWriteResolution(11);


}

void MotorDriver::drive(int Direction, int Rotation, int Power, int Output, int Orbit, int M1_rpm, int M2_rpm, int M3_rpm, int M4_rpm) {
    if(Power != 0 || Output != 0 || Rotation != 0){
        float anglerad = Direction / 57.2958;
        motorA.Update((-Power * ((cos(anglerad)*cos(0.7853))-(sin(anglerad)*sin(0.7853)))) + Output + Rotation,M1_rpm);
        motorB.Update((Power * ((cos(anglerad)*cos(2.3562))-(sin(anglerad)*sin(2.3562)))) + Output + Rotation, M2_rpm);
        motorC.Update((-Power * ((cos(anglerad)*cos(3.9270))-(sin(anglerad)*sin(3.9270)))) + Output + Rotation,M3_rpm);
        motorD.Update((Power * ((cos(anglerad)*cos(5.4978))-(sin(anglerad)*sin(5.4978)))) + Output + Rotation, M4_rpm);  
    }else if(Power == 0 && Rotation == 0 && Output == 0){
        motorA.drive(0);
        motorB.drive(0);
        motorC.drive(0);
        motorD.drive(0);

    }
    
}




