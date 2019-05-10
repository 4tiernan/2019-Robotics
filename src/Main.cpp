#include <MotorDriver.h> //Motor driver library
#include <Pins_Main.h> //Pinout library
#include <Arduino.h>
#include <LSM303.h> //Compass library
#include <Wire.h>
#include <Pixy2I2C.h> //Pixy Library
#include "spi4teensy3.h" 
#include "Adafruit_GFX.h" 
#include "Adafruit_ILI9341.h"
#include <stdint.h> 
#include "TouchScreen.h"
#include "Encoder.h"
#include "EEPROM.h"
#include <Functions.h> //Math functions
#include <PID.h> //PID library

//TouchScreen /////////////////////////////////////////////////////////////////
// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 256.6);
#define START_BUTTON_X 120 
#define START_BUTTON_Y 260
#define START_BUTTON_RADIUS 55
#define CAL_BUTTON_X 50
#define CAL_BUTTON_Y 50
#define CAL_BUTTON_RADIUS 50
#define PIXY_BUTTON_X 185
#define PIXY_BUTTON_Y 50
#define PIXY_BUTTON_RADIUS 50
#define STATS_BUTTON_X 120
#define STATS_BUTTON_Y 130
#define STATS_BUTTON_RADIUS 50
#define Xmax 900 //Touchscreen values for calibration
#define Ymax 910
#define Xmin 140
#define Ymin 80

#define MATH_E 2.7182818284590452353602874713527 //Eulers number

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(tft_CS, tft_DC, tft_MOSI, tft_CLK, tft_RST, tft_MISO);
//End Touch Screen//////////////////////////////////////////////////////


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
LSM303 compass;

Pixy2I2C pixy;
Block ballBlock = Block();
Block goalBlock = Block();

//Screen Variables
int Debug = 0; // serial print
int16_t tempX, tempY;
uint16_t Startwidth,Startheight, CalWidth, CalHeight, PixyWidth, PixyHeight, StatsWidth, StatsHeight, ExitWidth, ExitHeight, SaveWidth, SaveHeight, SerialWidth, SerialHeight;
uint16_t Goal_StatusWidth, Goal_StatusHeight;
uint16_t SpinWidth, SpinHeight;
int touch_y, touch_x, touch_z;

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
//PID rotationPID(21,0.0000000001,4500));
//PID rotationPID(30,0.0000000001,60);
//PID rotationPID(40,0,50);
PID rotationPID(40,0,50); //sets pid values for rotation of the entire robot
void buttons(); //Checks if the go button has been pressed
double magnometer(); //returns the heading relative to the feild
void resetmag(); //resets the magnomter to 0
void screen_main_print(); //prints the touch screen main page
void screen(); //runs the screen 
void loopdetection(); //checks how fast the code is running
void runpixy(); 
void printserial();
void robotDetection();
void GoalCentring();
void drive();
void print_Exit_Button();
void print_Save_Button();
void print_Spin_Button();
void SpinMotors();
void save_Compass_Calibration();
void encoders();
void reset_Encoders();
bool isValidTouch(int16_t pressure);
void Button_Detection();
void Goally();
void Comms();

void setup() {
    Serial.begin(9600);
    Serial.println("Setup Started");
    SPI.begin();//spi is initialised at the begining of the code so it cannot overwrite pin functionality
    Serial.println("SPI UP");
    //EEPROM.write(255,1); //individual robot detection
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    robotDetection();//Was individual robot detection now is variable and compass setting
    Wire.begin();
    Serial.println("I2C UP");
    tft.begin();
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0,0);
    tft.print("EEPROM:");
    tft.setCursor(0,20);
    tft.print("MOTORS:");
    tft.setCursor(0,40);
    tft.print("  PIXY:");
    tft.setCursor(0,60);
    tft.print("  MAG:");
    Serial.println("TFT UP");
    EEPROM.begin();
    tft.setCursor(85,0);
    tft.print("UP");
    Serial.println("EEPROM UP");
    driver.init();//Motor driver library initalise
    tft.setCursor(85,20);
    tft.print("UP");
    Serial.println("MOTORS UP");
    pixy.init();
    tft.setCursor(85,40);
    tft.print("UP"); 
    Serial.println("PIXY UP");
    compass.init();
    compass.read();
    tft.setCursor(85, 60);
    Serial.println(compass.heading()); //check if the compass is working
    if(compass.heading() == 0 || isnan(compass.heading()) || compass.heading() > 356){
      tft.print("NAN");
      Serial.println(compass.heading());
      while(compass.heading()==0 || isnan(compass.heading()) || compass.heading() > 356){
        compass.init();
        compass.read();
      }
      tft.print("UP");
    }
    compass.enableDefault();   
    tft.print("UP");
    Serial.println("MAGNOMTER UP");
    tft.setCursor(0,100);
    tft.print("DEVICES OPPERATIONAL");
    tft.setCursor(0,260);
    tft.print("SERIAL:");
    tft.setCursor(90,260);
    if(Debug){
      tft.print("ON");
    }else{
      tft.print("OFF");
    }
    Serial.println("Devices Operational");
    pixy.setLamp(0,0);//(White Lights, RBG) set's the pixy's light status 
    tft.setCursor(0,280);
    tft.print("MILLIS:");
    tft.setCursor(90,280);
    tft.print(millis());                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
    resetmag();
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
    screen_main_print(); 
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

  screen();
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
double magnometer(){
  compass.read();
  heading = compass.heading();
  rawmag = (heading - GoalNorth)*1;
  if(rawmag > 180)rawmag -= 360;
  if(rawmag < -180)rawmag += 360;
  magheading = (double)rawmag;
  return magheading;
}
void resetmag(){
  compass.read();
  GoalNorth = compass.heading();
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
void print_Exit_Button(){
  tft.setTextSize(7);
  tft.getTextBounds("EXIT", 0, 0, &tempX, &tempY, &ExitWidth, &ExitHeight);
  tft.drawRect(117-ExitWidth/2, 317-ExitHeight,ExitWidth, ExitHeight, ILI9341_WHITE);
  tft.drawRect(116-ExitWidth/2, 316-ExitHeight,ExitWidth+2, ExitHeight+2, ILI9341_WHITE);
  tft.setCursor(120-ExitWidth/2, 320-ExitHeight);
  tft.setTextColor(ILI9341_RED);
  tft.print("EXIT");
}
void print_Save_Button(){
  tft.setTextSize(7);
  tft.getTextBounds("SAVE", 0, 0, &tempX, &tempY, &SaveWidth, &SaveHeight);
  tft.drawRect(117-SaveWidth/2, 257-SaveHeight,SaveWidth, SaveHeight, ILI9341_WHITE);
  tft.drawRect(116-SaveWidth/2, 256-SaveHeight,SaveWidth+2, SaveHeight+2, ILI9341_WHITE);
  tft.setCursor(120-SaveWidth/2, 260-SaveHeight);
  tft.setTextColor(ILI9341_RED);
  tft.print("SAVE");
}
void print_Spin_Button(){
  tft.setTextSize(7);
  tft.getTextBounds("SPIN", 0, 0, &tempX, &tempY, &SpinWidth, &SpinHeight);
  tft.drawRect(117-SpinWidth/2, 197-SpinHeight,SpinWidth, SpinHeight, ILI9341_WHITE);
  tft.drawRect(116-SpinWidth/2, 196-SpinHeight,SpinWidth+2, SpinHeight+2, ILI9341_WHITE);
  tft.setCursor(120-SpinWidth/2, 200-SpinHeight);
  tft.setTextColor(ILI9341_RED);
  tft.print("SPIN");
}
void screen_main_print(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(6);
  tft.setCursor(102, 24);
  tft.print(EEPROM.read(255));
  

  tft.drawCircle(START_BUTTON_X, START_BUTTON_Y, START_BUTTON_RADIUS, ILI9341_WHITE);
  tft.drawCircle(START_BUTTON_X, START_BUTTON_Y, START_BUTTON_RADIUS-2, ILI9341_WHITE);
  tft.setTextSize(3);
  if(GO == 0)tft.setTextColor(ILI9341_GREEN);
  if(GO)tft.setTextColor(ILI9341_RED);
  tft.getTextBounds("START", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
  tft.setCursor(START_BUTTON_X-(Startwidth/2),START_BUTTON_Y-(Startheight/2));
  tft.print("START");
  
  tft.drawCircle(CAL_BUTTON_X, CAL_BUTTON_Y, CAL_BUTTON_RADIUS, ILI9341_WHITE);
  tft.drawCircle(CAL_BUTTON_X, CAL_BUTTON_Y, CAL_BUTTON_RADIUS-2, ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLUE);
  tft.getTextBounds("CAL", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
  tft.setCursor(CAL_BUTTON_X-(Startwidth/2),CAL_BUTTON_Y-(Startheight/2));
  tft.print("CAL");

  tft.drawCircle(PIXY_BUTTON_X, PIXY_BUTTON_Y, PIXY_BUTTON_RADIUS, ILI9341_WHITE);
  tft.drawCircle(PIXY_BUTTON_X, PIXY_BUTTON_Y, PIXY_BUTTON_RADIUS-2, ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLUE);
  tft.getTextBounds("PIXY", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
  tft.setCursor(PIXY_BUTTON_X-(Startwidth/2),PIXY_BUTTON_Y-(Startheight/2));
  tft.print("PIXY");

  tft.drawCircle(STATS_BUTTON_X, STATS_BUTTON_Y, STATS_BUTTON_RADIUS, ILI9341_WHITE);
  tft.drawCircle(STATS_BUTTON_X, STATS_BUTTON_Y, STATS_BUTTON_RADIUS-2, ILI9341_WHITE);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_BLUE);
  tft.getTextBounds("STATS", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
  tft.setCursor(STATS_BUTTON_X-(Startwidth/2),STATS_BUTTON_Y-(Startheight/2));
  tft.print("STATS");

  
  if(Current_Goal_Signature == 3){
    tft.fillRect(0,160,65,65,ILI9341_YELLOW);
  }else if(Current_Goal_Signature == 2){
    tft.fillRect(0,160,65,65,ILI9341_BLUE);
  }
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_BLACK);
  tft.getTextBounds("GOAL", 0, 0, &tempX, &tempY, &Goal_StatusWidth, &Goal_StatusHeight);
  tft.setCursor(32-Goal_StatusWidth/2, 192-Goal_StatusHeight/2);
  tft.print("GOAL"); 
  tft.fillTriangle(210, 100, 240, 160, 180,160,ILI9341_GREEN);
  tft.fillTriangle(210, 240, 240, 180, 180,180,ILI9341_GREEN);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(195,165);
  tft.print(Speed,1);
  if(Debug){
    tft.fillRect(0, 245,50,50,ILI9341_GREEN);
  }else{
    tft.fillRect(0, 245,50,50,ILI9341_RED);
  }

  Screen_main = 1;
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
void screen(){
  if(Spin)SpinMotors();
  TSPoint p = ts.getPoint();
  touch_x = map(p.x, Xmin, Xmax, 0, 240);
  touch_y = map(p.y, Ymin, Ymax, 0, 360);
  touch_z = p.z;
  
  if(isValidTouch(p.z) && touch_x < 117+ExitWidth/2 && touch_x > 117-ExitWidth/2 && touch_y > 360-ExitHeight && Stats){
    Stats = 0;
  }else if(isValidTouch(p.z) && touch_x < 117+ExitWidth/2 && touch_x > 117-ExitWidth/2 && touch_y > 360-ExitHeight && Pixy){
    Pixy = 0;
  }else if(isValidTouch(p.z) && touch_x < 117+ExitWidth/2 && touch_x > 117-ExitWidth/2 && touch_y > 360-ExitHeight && Cal){
    Cal = 0;
    Compass_Reset = 0;
    Compass_Calibrate = 0;
    Encoder_Reset = 0;
  }else if(isValidTouch(p.z) && touch_y > 160 && touch_y < 235 && touch_x > 0 && touch_x < 75 && Screen_main){
    if(Current_Goal_Signature == 2){
      Current_Goal_Signature = 3;
      tft.fillRect(0,160,65,65,ILI9341_YELLOW);
    }else if(Current_Goal_Signature == 3){
      Current_Goal_Signature = 2;
      tft.fillRect(0,160,65,65,ILI9341_BLUE);
    }
    tft.setTextSize(2);
    tft.setTextColor(ILI9341_BLACK);
    tft.getTextBounds("GOAL", 0, 0, &tempX, &tempY, &Goal_StatusWidth, &Goal_StatusHeight);
    tft.setCursor(32-Goal_StatusWidth/2, 192-Goal_StatusHeight/2);
    tft.print("GOAL"); 
    delay(100);
  }else if(isValidTouch(p.z) && touch_y > 100 && touch_y < 180 && touch_x > 180 && touch_x < 240 && Screen_main){
    if(Speed < 1)Speed += 0.1;
    delay(300);
    tft.fillRect(195,165,35,15,ILI9341_BLACK);
    tft.setCursor(195,165);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(Speed,1);
  }else if(isValidTouch(p.z) && touch_y > 200 && touch_y < 240 && touch_x > 180 && touch_x < 240 && Screen_main){
    if(Speed >= 0.1)Speed -= 0.1;
    delay(300);
    tft.fillRect(195,165,35,15,ILI9341_BLACK);
    tft.setCursor(195,165);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.print(Speed,1);
  }else if(isValidTouch(p.z) && touch_y > 257-SaveHeight && touch_y < 257 && Cal && Compass_Calibrate){
    save_Compass_Calibration();
    tft.fillRect(00,30,160,95, ILI9341_BLACK);
    tft.setTextSize(8);
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(0,40);
    tft.print("SAVED");
    delay(300);
    Cal = 0;
    Compass_Reset = 0;
    Compass_Calibrate = 0;
    Encoder_Reset = 0;
  }else if(isValidTouch(p.z) && touch_y > 197-SaveHeight && touch_y < 197 && Cal && Compass_Calibrate){
    Spin = 1;
    ClockWiseSpin = 1;
    delay(200);
  }else if(Cal && isValidTouch(p.z) && Compass_Reset == 1 && Compass_Calibrate == 1 && Encoder_Reset == 1){
    if(touch_y < 89){
      Compass_Reset = 1;
      Compass_Calibrate = 0;
      Encoder_Reset = 0;
    }else if(touch_y >= 89 && touch_y <= 149){
      Compass_Calibrate = 1;
      reset_Encoders(0);
      Compass_Reset = 0;
      Encoder_Reset = 0;
      compass.m_min = {32767, 32767, 32767};
      compass.m_max = {-32768, -32768, -32768};
      tft.fillScreen(ILI9341_BLACK);
      print_Spin_Button();
      print_Exit_Button();
      print_Save_Button();
      tft.setTextSize(2);
      tft.getTextBounds("Compass Calibrate", 0, 0, &tempX, &tempY, &CalWidth, &CalHeight);
      tft.setCursor(120-CalWidth/2, 0);
      tft.fillRect(120-CalWidth/2, 0, CalWidth, CalHeight, ILI9341_RED);
      tft.setTextColor(ILI9341_WHITE);
      tft.print("Compass Calibrate");
    }else if(touch_y > 149 && touch_y <= 229 ){
      Encoder_Reset = 1;
      Compass_Reset = 0;
      Compass_Calibrate = 0;
    }
  }else if(isValidTouch(p.z)) {
      if(Screen_main && touch_x < START_BUTTON_X + START_BUTTON_RADIUS && touch_x > START_BUTTON_X - START_BUTTON_RADIUS && touch_y < START_BUTTON_Y + START_BUTTON_RADIUS && touch_y > START_BUTTON_Y - START_BUTTON_RADIUS){ 
        while(isValidTouch(touch_z)){
        TSPoint p = ts.getPoint();
        touch_z = p.z;
        
        }
        tft.fillRect(START_BUTTON_X-Startwidth/2,START_BUTTON_Y-Startheight/2,Startwidth,Startheight, ILI9341_BLACK);
        tft.setTextSize(3);
        if(GO){
          tft.setTextColor(ILI9341_GREEN);
          GO = 0;
        }else{
          tft.setTextColor(ILI9341_RED);
          GO = 1;
        }
        tft.getTextBounds("START", 0, 0, &tempX, &tempY, &Startwidth, &Startheight);
        tft.setCursor(START_BUTTON_X-(Startwidth/2),START_BUTTON_Y-(Startheight/2));
        tft.print("START");
        delay(100);
    }else if(Stats || Cal || Pixy){
      //exit the if statements
    }else if(touch_x < CAL_BUTTON_X + CAL_BUTTON_RADIUS && touch_x > CAL_BUTTON_X - CAL_BUTTON_RADIUS && touch_y < CAL_BUTTON_Y + CAL_BUTTON_RADIUS && touch_y > CAL_BUTTON_Y - CAL_BUTTON_RADIUS){
      Cal = 1; 
      tft.fillScreen(ILI9341_BLACK);
      Screen_main = 0;
    }else if(touch_x < PIXY_BUTTON_X + PIXY_BUTTON_RADIUS && touch_x > PIXY_BUTTON_X - PIXY_BUTTON_RADIUS && touch_y < PIXY_BUTTON_Y + PIXY_BUTTON_RADIUS && touch_y > PIXY_BUTTON_Y - PIXY_BUTTON_RADIUS){
      Pixy = 1;  
      tft.fillScreen(ILI9341_BLACK);
      Screen_main = 0;
    }else if(touch_x < STATS_BUTTON_X + STATS_BUTTON_RADIUS && touch_x > STATS_BUTTON_X - STATS_BUTTON_RADIUS && touch_y < STATS_BUTTON_Y + STATS_BUTTON_RADIUS && touch_y > STATS_BUTTON_Y - STATS_BUTTON_RADIUS){
      Stats = 1; 
      tft.fillScreen(ILI9341_BLACK);
      print_Exit_Button();
      Screen_main = 0;
      }
    }else if(touch_x < 50 && touch_x > 0 && touch_y > 245 && touch_y < 310){
      if(Debug){
        Debug = 0;
        tft.fillRect(0, 245,50,50,ILI9341_RED);
      }else if(!Debug){
        Debug = 1;
        tft.fillRect(0, 245,50,50,ILI9341_GREEN);
      }
      delay(100);

    }
 
  if(millis() - lastprint > PRINTDELAY){
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    if(Screen_main == 0){
      if(Stats){
        tft.fillRect(63,0,50,230,ILI9341_BLACK);
        magnometer();
        tft.setTextSize(1);
        tft.setCursor(0, 0);
        tft.setTextColor(ILI9341_WHITE);
        tft.print("Magnometer:");
        tft.print(magnometer());
        tft.setCursor(0,10);
        tft.print("Encoder 1 :");
        tft.print(M1_encoder);
        tft.setCursor(0,20);
        tft.print("Encoder 2 :");
        tft.print(M2_encoder);
        tft.setCursor(0,30);
        tft.print("Encoder 3 :");
        tft.print(M3_encoder);
        tft.setCursor(0,40);
        tft.print("Encoder 4 :");
        tft.print(M4_encoder);
        tft.setCursor(0,50);
        tft.print("       LPS:");
        tft.print(lps);
        tft.setCursor(0,60);
        tft.print("   Seeball:");
        tft.print(seeBall);
        tft.setCursor(0,70);
        tft.print("   Seegoal:");
        tft.print(seeGoal);
        tft.setCursor(0,80);
        tft.print("   M1 RPM :");
        tft.print(M1_rpm);
        tft.setCursor(0,90);
        tft.print("   M2 RPM :");
        tft.print(M2_rpm);
        tft.setCursor(0,100);
        tft.print("   M3 RPM :");
        tft.print(M3_rpm);
        tft.setCursor(0,110);
        tft.print("   M4 RPM :");
        tft.print(M4_rpm);
        tft.setCursor(0,120);
        tft.print("Ball  Dist:");
        tft.print(ballDistance);
        tft.setCursor(0,130);
        tft.print("Ball Width:");
        tft.print(ballBlock.m_width);
        tft.setCursor(0,140);
        tft.print("Goal Angle: ");
        tft.print(goalAngle);
        tft.setCursor(0,150);
        tft.print("Ball Angle: ");
        tft.print(ballAngle);
        tft.setCursor(0,160);
        tft.print("AbsBallang: ");
        tft.print(AbsBallAngle);
        tft.setCursor(0,170);
        tft.print("AbsGoalang: ");
        tft.print(absGoalAngle);
        tft.setCursor(0,180);
        tft.print("OrbitAngle: ");
        tft.print(OrbitAngle);
        tft.setCursor(0,190);
        tft.print("   Feild X: ");
        tft.print(x);
        tft.setCursor(0,200);
        tft.print("   Feild Y: ");
        tft.print(y);
        tft.setCursor(0,210);
        tft.print("Goal Dist : ");
        tft.print(goalDistance);
        tft.setCursor(0,220);
        tft.print("Goal Width: ");
        tft.print(goalBlock.m_width);
        
        
        
      }else if(Pixy){
        print_Exit_Button();
        tft.drawRect(last_block_0_x, last_block_0_y, last_block_0_width, last_block_0_height, ILI9341_BLACK);
        tft.drawRect(last_block_1_x, last_block_1_y, last_block_1_width, last_block_1_height, ILI9341_BLACK);
        tft.drawRect(ballBlock.m_x,ballBlock.m_y,ballBlock.m_width,ballBlock.m_height, ILI9341_ORANGE);
        tft.drawRect(goalBlock.m_x,goalBlock.m_y,goalBlock.m_width,goalBlock.m_height, ILI9341_BLUE);
        last_block_0_x = ballBlock.m_x;
        last_block_0_y = ballBlock.m_y;
        last_block_0_width = ballBlock.m_width;
        last_block_0_height = ballBlock.m_height;
        last_block_1_x = goalBlock.m_x;
        last_block_1_y = goalBlock.m_y;
        last_block_1_width = goalBlock.m_width;
        last_block_1_height = goalBlock.m_height;
      }else if(Cal && Compass_Reset == 0 && Compass_Calibrate == 0 && Encoder_Reset == 0){
        tft.setTextSize(5);
        tft.getTextBounds("Compass Reset", 0, 0, &tempX, &tempY, &CalWidth, &CalHeight);
        tft.setCursor(122-CalWidth/2, 2);
        tft.drawRect(120-CalWidth/2, 0, CalWidth-2, CalHeight, ILI9341_GREEN);
        tft.drawRect(119-CalWidth/2, 1, CalWidth, CalHeight-2, ILI9341_GREEN);
        tft.setTextColor(ILI9341_WHITE);
        tft.print("Compass Reset");
        tft.setTextSize(4);
        tft.getTextBounds("Compass   Calibrate", 0, 0, &tempX, &tempY, &CalWidth, &CalHeight);
        tft.setCursor(122-CalWidth/2, 82);
        tft.drawRect(120-CalWidth/2, 80, CalWidth, CalHeight, ILI9341_RED);
        tft.drawRect(121-CalWidth/2, 81, CalWidth-2, CalHeight-2, ILI9341_RED);
        tft.setTextColor(ILI9341_WHITE);
        tft.print("Compass   Calibrate");
        tft.setTextSize(5);
        tft.getTextBounds("Encoder Reset", 0, 0, &tempX, &tempY, &CalWidth, &CalHeight);
        tft.setCursor(123-CalWidth/2, 146);
        tft.drawRect(120-CalWidth/2, 144, CalWidth, CalHeight-1, ILI9341_BLUE);
        tft.drawRect(121-CalWidth/2, 145, CalWidth-2, CalHeight-1, ILI9341_BLUE);
        tft.setTextColor(ILI9341_WHITE);
        tft.print("Encoder Reset");
        print_Exit_Button();       
        Compass_Reset = 1;
        Compass_Calibrate = 1;
        Encoder_Reset = 1;
      }else if(Compass_Reset == 1 && Compass_Calibrate== 0 && Encoder_Reset == 0){
        while(pixy.ccc.getBlocks() < 0){
        }
        blocks = pixy.ccc.numBlocks;
        bool foundgoal = 0;
        for(int i=0; i < blocks; i++){
          if (pixy.ccc.blocks[i].m_signature != 1) {
            foundgoal = true;
            if(pixy.ccc.blocks[i].m_signature == 2){
              //blue goal
              tft.fillScreen(ILI9341_BLUE); 
            }else if(pixy.ccc.blocks[i].m_signature == 3){
              //yellow Goal
              tft.fillScreen(ILI9341_YELLOW);
            }  
            Current_Goal_Signature = pixy.ccc.blocks[i].m_signature;
            break;
          }          
        }
        if(!foundgoal){
          tft.fillScreen(ILI9341_GREEN);
        }
        resetmag();
        //print_Exit_Button();
        Compass_Reset = 0;
        Compass_Calibrate = 0;
        Encoder_Reset = 0;
        Cal = 0;
      }else if(Compass_Reset == 0 && Compass_Calibrate == 1 && Encoder_Reset == 0){
        compass.read();
        compass.m_min.x = min(compass.m_min.x, compass.m.x);
        compass.m_min.y = min(compass.m_min.y, compass.m.y);
        compass.m_min.z = min(compass.m_min.z, compass.m.z);
        compass.m_max.x = max(compass.m_max.x, compass.m.x);
        compass.m_max.y = max(compass.m_max.y, compass.m.y);
        compass.m_max.z = max(compass.m_max.z, compass.m.z);
        tft.fillRect(70,30,90,95, ILI9341_BLACK);
        tft.setTextSize(2);
        tft.setCursor(0, 30);
        tft.printf("Min X: %d\n", compass.m_min.x);
        tft.printf("Min Y: %d\n", compass.m_min.y);
        tft.printf("Min Z: %d\n", compass.m_min.z);
        tft.printf("Max X: %d\n", compass.m_max.x);
        tft.printf("Max Y: %d\n", compass.m_max.y);
        tft.printf("Max Z: %d\n", compass.m_max.z);
      }else if(Compass_Reset == 0 && Compass_Calibrate == 0 && Encoder_Reset == 1){
        tft.fillScreen(ILI9341_BLUE);
        reset_Encoders(0);
        Compass_Reset = 0;
        Compass_Calibrate = 0;
        Encoder_Reset = 0;
        Cal = 0;
      }else if(Cal == 0 && Pixy == 0 && Stats == 0 && Compass_Reset == 0 && Compass_Calibrate == 0 && Encoder_Reset == 0){
        screen_main_print();
      }
    }
    
    lastprint = millis();
  }
}