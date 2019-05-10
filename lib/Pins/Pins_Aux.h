#ifndef PINS_Aux_H
#define PINS_AUX_H

#define PROCESSOR_SCL 3
#define PROCESSOR_SDA 4
#define PROCESSOR_TX 7
#define PROCESSOR_RX 8 
#define BUTTON_LIGHTS 24
#define MAG_INT_1 20
#define MAG_INT_2 21
#define MAG_SCL 19
#define MAG_SDA 18
#define CELL_1 A21
#define CELL_2 A22
#define Cell_3 A14

#define TFT_DC 16 //tft screen pins
#define TFT_CS 15
#define TFT_MOSI 11
#define TFT_MISO 12
#define TFT_CLK 14
#define TFT_RST -1//dummy pin as the rst pin is not connected tft pins

//TouchScreen /////////////////////////////////////////////////////////////////
#define YP A20  // must be an analog pin, use "An" notation!
#define XM A17  // must be an analog pin, use "An" notation!
#define YM 38   // can be a digital pin
#define XP 37   // can be a digital pin


//XBEE 
#define XBEE_MOSI 0
#define XBEE_MISO 1
#define XBEE_CS 31
#define XBEE_SCK 32





#endif
