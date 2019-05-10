#include <MotorDriver.h> //Motor driver library
#include <Pins_Main.h> //Pinout library
#include <Arduino.h>
//#include <LSM303.h> //Compass library
#include <Wire.h>
#include <Pixy2I2C.h> //Pixy Library
//#include "spi4teensy3.h" 
//#include "Adafruit_GFX.h" 
//#include "Adafruit_ILI9341.h"
#include <stdint.h> //C++ Library
//#include "TouchScreen.h"
#include "Encoder.h"
#include "EEPROM.h"
#include <Functions.h> //Math functions
#include <PID.h> //PID library



#define MATH_E 2.7182818284590452353602874713527 //Eulers number



#define STARTDISTANCE 10 //ball calibration distance
#define BALLPIXELS 116 //ball size in pixels at 10cm
#define BALL_SIGNATURE 1 //Signature of the ball on pixy
#define PIXY_UPDATE_TIME 16666 //16.66ms between cycles 60fps 
#define PRINTDELAY 200 //Delay between each print to the screen
#define LONGDIST 75 //Distance after which the robot will just move forward
#define SHORTDIST 5 //Closer than this distance the robot will just move perpendicular to the ball
#define MAXBALLANGLE 15 //the ball angle must be smaller than this angle for the robot to move forward
#define GOALPIXELS 50 //goal size n pixels at goal start distance
#define GOALSTARTDISTANCE 100 //Distance to the Goal for Calibration
#define CAL_SPINS 14770 //the amount of encoder clicks the robot should turn when spinning for cal 1477 = one rotation
#define PIXY_PIXELS 316 //The number of horizontal pixels in the pixy
#define PIXY_FOV 60 //The Pixy's feild of view in the horizontal in degrees
#define GOAL_CORRECT_MULTIPLIER 2
uint16_t blocks; //Variable to store the pixy's blocks


MotorDriver driver;


Pixy2I2C pixy;
Block ballBlock = Block();
Block goalBlock = Block();

//Screen Variables
int Debug = 0; // serial print


int lps, power, orbitSpeed, rotateSpeed, OrbitAngle, OrbitDirect, speed, Current_Goal_Signature = 2, robot;
float GoalNorth, rawmag, heading, ballAngle, AbsBallAngle, goalAngle, absGoalAngle, goalDistance;
float Speed, multiplier, x, y;
bool GO = 0, Ready, seeBall, seeGoal, Compass_Reset, Compass_Calibrate, Encoder_Reset, newGoal, newBall;
unsigned long loops, lastrun, lastprint; 
double magheading, ballDistance;
int last_block_0_x, last_block_0_y, last_block_0_width, last_block_0_height;
int last_block_1_x, last_block_1_y, last_block_1_width, last_block_1_height;
unsigned long last_check, last_rpm_read, lastBallTime, lastGoalTime;
bool Cal, Pixy, Stats, Screen_main, Button_Status, Stop_Ready, Orbiting;
bool ClockWiseSpin, Spin,left;
double M1_encoder, M2_encoder, M3_encoder, M4_encoder;
double M1_rpm, M2_rpm, M3_rpm, M4_rpm;
double M1_last_rpm, M2_last_rpm, M3_last_rpm, M4_last_rpm;
//encoders 
Encoder M1_raw_encoder(24, 25);
Encoder M2_raw_encoder(26, 27);
Encoder M3_raw_encoder(28, 29);
Encoder M4_raw_encoder(30, 31);

PID rotationPID(40,0,50); //sets pid values for rotation of the entire robot
void buttons(); //Checks if the go button has been pressed
void loopdetection(); //checks how fast the code is running
void runpixy(); 
void printserial();
void robotDetection();
void GoalCentring();
void drive();
void SpinMotors();
void encoders();
void reset_Encoders();
void Button_Detection();
void Goally();
void Comms();

void setup() {
    Serial.begin(9600);
    Serial.println("Setup Started");
    //EEPROM.write(255,1); //individual robot detection
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    robotDetection();//Was individual robot detection now is variable and compass setting
    Wire.begin();
    Serial.println("I2C UP");
    EEPROM.begin();
    driver.init();//Motor driver library initalise
    Serial.println("MOTORS UP");
    pixy.init();
    Serial.println("PIXY UP");
    Serial.println("Devices Operational");
    pixy.setLamp(0,0);//(White Lights, RBG) set's the pixy's light status 
    while(pixy.ccc.getBlocks() < 0 && millis() < 3000){
    }
    blocks = pixy.ccc.numBlocks;
    bool foundgoal = 0;
    for(int i=0; i < blocks; i++){
      if (pixy.ccc.blocks[i].m_signature != 1) {
        foundgoal = true;
        if(pixy.ccc.blocks[i].m_signature == 2){
          //blue goal
          //tft.fillScreen(ILI9341_BLUE); 
        }else if(pixy.ccc.blocks[i].m_signature == 3){
          //yellow Goal
          //tft.fillScreen(ILI9341_YELLOW);
        }  
        Current_Goal_Signature = pixy.ccc.blocks[i].m_signature;
        break;
    }          
    }
    robot = EEPROM.read(255);
}
void loop() {
  buttons();
  loopdetection();
  runpixy(); 
  //GoalCentring();
  drive();
  //power = 10200 * Speed;
   //power = 0;
   //if(GO)driver.drive(0,0,power,rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
   //if(!GO)driver.drive(0,0,0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
  encoders();

 
  if(Debug){
    printserial();
  }
}
void encoders(){
    M1_encoder = M1_raw_encoder.read() / 48;
    M2_encoder = M2_raw_encoder.read() / 48;
    M3_encoder = M3_raw_encoder.read() / 48;
    M4_encoder = M4_raw_encoder.read() / 48;
    M1_rpm = 60000000*((M1_raw_encoder.read()-M1_last_rpm)/48)/(micros()-last_rpm_read);
    M2_rpm = 60000000*((M2_raw_encoder.read()-M2_last_rpm)/48)/(micros()-last_rpm_read);
    M3_rpm = 60000000*((M3_raw_encoder.read()-M3_last_rpm)/48)/(micros()-last_rpm_read);
    M4_rpm = 60000000*((M4_raw_encoder.read()-M4_last_rpm)/48)/(micros()-last_rpm_read);
    M1_last_rpm = M1_raw_encoder.read();
    M2_last_rpm = M2_raw_encoder.read();
    M3_last_rpm = M3_raw_encoder.read();
    M4_last_rpm = M4_raw_encoder.read();
    last_rpm_read = micros();
}
void reset_Encoders(int value){
  M1_raw_encoder.write(value);
  M2_raw_encoder.write(value);
  M3_raw_encoder.write(value);
  M4_raw_encoder.write(value);
}
void runpixy(){
  if(pixy.ccc.getBlocks() >= 0){
    blocks = pixy.ccc.numBlocks;
    seeBall = false;
    seeGoal = false;
    newGoal = false;
    newBall = false;
    //if pixy.ccc.getBlocks() returns 
    //-2 then new data is not available yet
    //-1 then some kind of bitstream error
    //0 then no blocks are detected
  }   
  for(int i=0; i < blocks; i++){
    if (pixy.ccc.blocks[i].m_signature == BALL_SIGNATURE) {
      ballBlock = pixy.ccc.blocks[i];
      seeBall = true;
      newBall = true;
    }
    if (pixy.ccc.blocks[i].m_signature == Current_Goal_Signature) {
      goalBlock = pixy.ccc.blocks[i];
      seeGoal = true;   
      newGoal = true;
    }
  }
  if (newGoal) {
    goalAngle = (goalBlock.m_x - (PIXY_PIXELS/2)) / (PIXY_PIXELS/PIXY_FOV);
    absGoalAngle = goalAngle + magnometer();
    if (abs(absGoalAngle) > 90) {
      goalAngle = 0;
      absGoalAngle = 0;
    }
  }else{
    goalAngle = 0;
    absGoalAngle = 0;
  }
  if (newBall) {
    if (ballBlock.m_width > ballBlock.m_height) {
      ballDistance = (STARTDISTANCE*BALLPIXELS) / ballBlock.m_width;
    } else {
      ballDistance = (STARTDISTANCE*BALLPIXELS) / ballBlock.m_height;
    }
    ballAngle = (ballBlock.m_x - (PIXY_PIXELS/2))/(PIXY_PIXELS/PIXY_FOV);
    AbsBallAngle = ballAngle - goalAngle;
  }
}
void drive() {
  if (GO) {
    if (seeBall) {
      if(ballAngle+magnometer()-goalAngle < -MAXBALLANGLE) {
        OrbitDirect = -1;
      } else if(ballAngle+magnometer()-goalAngle > MAXBALLANGLE) {
        OrbitDirect = 1;
      } else if(ballAngle+magnometer()-goalAngle < MAXBALLANGLE && ballAngle+magnometer()-goalAngle > -MAXBALLANGLE){
        OrbitDirect = 0;
      }
      multiplier = constrain((float)(LONGDIST-ballDistance)/(float)(LONGDIST-SHORTDIST), 0, 1);
      // if(abs(AbsBallAngle) < MAXBALLANGLE){
      //   OrbitAngle = ballAngle;
      // }else{
      if(ballDistance > 25 || !seeGoal || !seeBall){
        Orbiting = 0;
      }
      if(Orbiting){
          OrbitAngle = 1.5*ballAngle;
      }
      if(ballDistance < 20 && seeGoal && seeBall && abs(ballAngle) < MAXBALLANGLE){
        Orbiting = 1;
      }else if(!Orbiting){
        //OrbitAngle = constrain(GOAL_CORRECT_MULTIPLIER*AbsBallAngle,-90,90);
        if(abs(magnometer() + ballAngle) < 40){
          OrbitAngle = ballAngle;
          digitalWrite(LED_PIN, HIGH);
        }else{
          OrbitAngle = ballAngle + constrain(90*OrbitDirect*multiplier, -90, 90);
          digitalWrite(LED_PIN, LOW);
        }
      }
        

        //OrbitAngle = ballAngle + constrain(abs(AbsBallAngle)*OrbitDirect*multiplier, -90, 90);
      //}
      power = 10200 * Speed;
      if(Orbiting){
        driver.drive(OrbitAngle,0,power, rotationPID.Compute(-goalAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }else if(abs(magnometer() + ballAngle) > 20){
        driver.drive(OrbitAngle,0,power, rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }else if(abs(magnometer() + ballAngle) < 20){
        driver.drive(OrbitAngle,0,power, rotationPID.Compute(magnometer(),0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }
    } else {
      if(ballAngle >= 0){
        driver.drive(0,rotateSpeed,0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      } else {
        driver.drive(0,-rotateSpeed,0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }
    }
  } else if(!Spin) {
    driver.drive(0, 0, 0, 0, 0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
  }
}
/*
void Goally(){
  if(GO){
    if(seeBall){
      if(goalDistance <= 90){
        //go back 
        driver.drive(180,0,power, rotationPID.Compute(magnometer(),0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }else{
        if(ballAngle + magnometer() < -10){
          driver.drive(-90,0,power, rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
        }else if(ballAngle + magnometer() > 10){
          driver.drive(90,0,power, rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
        }else{
          driver.drive(0,0,0, rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
        }
      }
       
    }else{
      if(goalDistance < 180){
        driver.drive(180,0,power, rotationPID.Compute(-ballAngle,0),0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }else{
        driver.drive(0,0,0, 0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }
      if(magnometer() > 90){
        left = 1;
      }
      if(magnometer() < -90){
        left = 0;
      }
        driver.drive(0,-left*rotateSpeed,0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
      }
    

    
  }else if(!Spin) {
    driver.drive(0, 0, 0, 0, 0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
  }
}
*/
void Comms(){
  Wire.setSCL(19);
  Wire.setSDA(18);
}
void loopdetection(){
  if(micros() - lastrun >= 1000000){
    lps = loops;
    loops = 0;
    lastrun = micros();
  }
  loops += 1;
}
void buttons(){
  Button_Status = digitalRead(34);
  if(Button_Status && Stop_Ready && GO){
    GO = 0;
    Stop_Ready = 0;
    if(Screen_main){
      tft.fillRect(START_BUTTON_X-Startwidth/2,START_BUTTON_Y-Startheight/2,Startwidth,Startheight, ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setTextColor(ILI9341_GREEN);
      tft.getTextBounds("START", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
      tft.setCursor(START_BUTTON_X-(Startwidth/2),START_BUTTON_Y-(Startheight/2));
      tft.print("START");
    }
    return;
  }
  if(Button_Status == 0 && GO == 0){
    Ready = 1;
    return;
  }
  if(Button_Status == 0 && GO && !Ready){
    Stop_Ready = 1;
    return;
  }
  if(Button_Status == 1 && Ready == 1){
    GO = 1;
    Ready = 0;
    if(Screen_main){
      tft.fillRect(START_BUTTON_X-Startwidth/2,START_BUTTON_Y-Startheight/2,Startwidth,Startheight, ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setTextColor(ILI9341_RED);
      tft.getTextBounds("START", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
      tft.setCursor(START_BUTTON_X-(Startwidth/2),START_BUTTON_Y-(Startheight/2));
      tft.print("START");
    }
    return;
  }
}
void SpinMotors(){
  if(M1_raw_encoder.read() < CAL_SPINS && ClockWiseSpin){
    driver.drive(0,constrain(400*CAL_SPINS-M1_raw_encoder.read(), -3000, 3000),0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
  }
  if(M1_raw_encoder.read() >= CAL_SPINS){
    ClockWiseSpin = 0;
  }
  if(M1_raw_encoder.read() != 0 && !ClockWiseSpin){
    driver.drive(0,-constrain(400*M1_raw_encoder.read(), -3000, 3000),0,0,0, M1_rpm, M2_rpm, M3_rpm, M4_rpm);
  }
  if(M1_raw_encoder.read() > -30 && M1_raw_encoder.read() < 30 && !ClockWiseSpin && Spin){
    Spin = 0;
  }
}
void robotDetection(){
    orbitSpeed = 1200;
    rotateSpeed = 1500;
    Speed = 0.4; //0.3
    //kp = 2.2;
    //ki = 0;
    //kd = 0;
    Serial.println("Constants Set Successfully");
    int16_t Compass_Values[6];
      for(int i; i<6; i++){
        Compass_Values[i] = (int16_t)(EEPROM.read(2*i)<<8|EEPROM.read(2*i+1));
      }
    
    compass.m_min = {Compass_Values[0], Compass_Values[1], Compass_Values[2]};
    compass.m_max = {Compass_Values[3], Compass_Values[4], Compass_Values[5]};
    Serial.println("Compass Calibration Set Successfully");
}
void printserial(){
  Serial.print("Ball_dist:");
  Serial.print(ballDistance);
  Serial.print("  Ball_width:");
  Serial.print(ballBlock.m_width);
  Serial.print("  See Ball:");
  Serial.print(seeBall);
  Serial.print("  See Goal:");
  Serial.print(seeGoal);
  Serial.print(" Output:");
  Serial.print("  Power:");
  Serial.print(power);
  Serial.print("  GO:");
  Serial.print(GO);
  Serial.print("  Magerror:");
  Serial.println(magnometer());
}
void GoalCentring(){
    goalDistance = GOALPIXELS / goalBlock.m_width * GOALSTARTDISTANCE;
    y = cos(absGoalAngle/57.2958)*goalDistance;
    x = sin(absGoalAngle/57.2958)*goalDistance;

  
}
void save_Compass_Calibration(){
  int16_t Compass_Values[6] = {compass.m_min.x, compass.m_min.y, compass.m_min.z, compass.m_max.x, compass.m_max.y, compass.m_max.z};
    for(int i; i<6; i++){
    EEPROM.write(i*2,(uint16_t)(Compass_Values[i])>>8);
    EEPROM.write(i*2+1,(uint16_t)(Compass_Values[i])&0xff);
    }
}
bool isValidTouch(int16_t pressure) {
  return pressure > 100 && pressure < 600;
}
