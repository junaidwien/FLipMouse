  
/* 
     FLipWare - AsTeRICS Foundation
     For more info please visit: http://www.asterics-academy.net

     Module: FLipWare.ino  (main module)
     
        This firmware allows control of HID functions via FLipmouse module and/or AT-commands
        For a description of the supported commands see: commands.h

        HW-requirements:  
                  TeensyLC with external EEPROM (see FlipMouse board schematics)
                  4 FSR force sensors connected via voltage dividers to ADC pins A6-A9
                  1 pressure sensor connected to ADC pin A0
                  3 momentary switches connected to GPIO pins 0,1,2
                  3 slot indication LEDs connected to GPIO pins 5,16,17
                  1 TSOP 38kHz IR-receiver connected to GPIO pin 4
                  1 high current IR-LED connected to GPIO pin 6 via MOSEFT
                  optional: FlipMouse Bluetooth daughter board

        SW-requirements:  
                  Teensyduino AddOn for Arduino IDE
                  USB-type set to USB composite device (Serial + Keyboard + Mouse + Joystick)
          
   For a list of supported AT commands, see commands.h / commands.cpp

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation.
  
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; See the GNU General Public License: 
   http://www.gnu.org/licenses/gpl-3.0.en.html
 
 */ 

#include "FlipWare.h"        //  FABI command definitions
#include <EEPROM.h>
#include <Wire.h>        // for the external EEPROM
#include "math.h" 

// Constants and Macro definitions

#define LOOP_WAIT_TIME       5000   // wait time for one loop interation in microseconds

// Analog input pins (4FSRs + 1 pressure sensor)
#define MAX_ADC_VALUE 4096
#define PRESSURE_SENSOR_PIN A0
#define DOWN_SENSOR_PIN     A6
#define LEFT_SENSOR_PIN     A9
#define UP_SENSOR_PIN       A7
#define RIGHT_SENSOR_PIN    A8

// timings for getting coordinate values
#define CALIB_COORDINATES_TIME 500
#define CALIB_COORDINATES_TONEINTERVAL 40

//Piezo Pin (for tone generation)
#define TONE_PIN  9

// Global variables
int8_t  input_map[NUMBER_OF_PHYSICAL_BUTTONS]={0,2,1};  	//  maps physical button pins to button index 0,1,2
uint8_t IR_SENSOR_PIN = 4;								//  input pin of the TSOP IR receiver
int8_t  led_map[NUMBER_OF_LEDS]={5,16,17};              	//  maps leds pins   
uint8_t LED_PIN = 13;                                   	//  Led output pin, ATTENTION: if SPI (AUX header) is used, this pin is also SCK!!!
uint8_t IR_LED_PIN = 6;                                 	//  IR-Led output pin

struct slotGeneralSettings settings = {      // default settings valus, for type definition see fabi.h
    1,                                // stickMode: Mouse cursor movement active
    60, 60, 20, 20, 50, 50,           // accx, accy, deadzone x, deadzone y, maxspeed, acceleration time
    400, 600, 3,                      // threshold sip, threshold puff, wheel step,
    800, 10,                          // threshold strong puff, threshold strong sip
    50, 50, 50, 50 ,                  // gain up / down / left / right
    0, 0,                             // offset x / y
    0,                                // orientation
    1,                                // bt-mode 1: USB, 2: Bluetooth, 3: both (2 & 3 need daughter board)) 
    "",                               // no ir idle code
}; 

#define FULL_FORCE 1500
struct polarCoordinates standardCoordinates[8] = { {FULL_FORCE,-PI/2},{FULL_FORCE,0},{FULL_FORCE,PI/2},{FULL_FORCE,PI}};
struct polarCoordinates customCoordinates[8]   = { {0,0},{0,0},{0,0},{0,0} };

uint8_t workingmem[WORKINGMEM_SIZE];     // working memory (command parser, IR-rec/play)

char slotName[MAX_NAME_LEN] = "empty";
int EmptySlotAddress = 0;
uint8_t reportSlotParameters = REPORT_NONE;
uint8_t reportRawValues = 0;
uint8_t actSlot=0;

uint8_t  calib_zero = 1;                       // calibrate zeropoint right at startup !
uint8_t  calib_coordinates = 0;               // calibrate up/down/left/right on demand
											
//for chatty serial interface use: DEBUG_FULLOUTPUT (attention: not GUI compatible...)
//if set to DEBUG_FULLOUTPUT please activate the following preprocessor warning
uint8_t DebugOutput = DEBUG_NOOUTPUT;       
//uint8_t DebugOutput = DEBUG_FULLOUTPUT;       
//#warning "DEACTIVATE DEBUG_FULLOUTPUT AGAIN!!!"

int up,down,left,right,tmp;
int x,y;
int pressure;
double dz=0,force=0,angle=0;
int16_t  cx=0,cy=0;

uint8_t blinkCount=0;
uint8_t blinkTime=0;
uint8_t blinkStartTime=0;

int inByte=0;
char * keystring=0;

elapsedMicros loopTime;

// function declarations 
void UpdateLeds();
void UpdateTones();
void reportValues();
void applyDeadzone();


extern void handleCimMode(void);
extern void init_CIM_frame(void);
extern uint8_t StandAloneMode;


////////////////////////////////////////
// Setup: program execution starts here
////////////////////////////////////////

void setup() {
   Serial.begin(115200);
   
   //initialise BT module, if available
   initBluetooth();
   
   if (DebugOutput==DEBUG_FULLOUTPUT)  
     Serial.println("FLipMouse started, Flexible Assistive Button Interface ready !");

   Wire.begin();
   pinMode(IR_SENSOR_PIN,INPUT);
   analogWriteFrequency(IR_LED_PIN, 38000);  // TBD: flexible carrier frequency for IR, not only 38kHz !

   analogReadResolution(12); 
   analogReadAveraging(64);

   pinMode(LED_PIN,OUTPUT);
   digitalWrite(LED_PIN,LOW);

   pinMode(IR_LED_PIN,OUTPUT);
   digitalWrite(LED_PIN,LOW);

   for (int i=0; i<NUMBER_OF_PHYSICAL_BUTTONS; i++)   // initialize physical buttons and bouncers
      pinMode (input_map[i], INPUT_PULLUP);   // configure the pins for input mode with pullup resistors

   for (int i=0; i<NUMBER_OF_LEDS; i++)   // initialize physical buttons and bouncers
      pinMode (led_map[i], OUTPUT);   // configure the pins for input mode with pullup resistors

   release_all();
   initDebouncers();
   for (int i=0; i<NUMBER_OF_BUTTONS; i++)   // initialize button array
   {
      buttons[i].value=0;
      keystringButtons[i]=0;
      keystringBufferLen=0;
   }
   
   init_CIM_frame();  // for AsTeRICS CIM protocol compatibility
   initButtons();
   
   bootstrapSlotAddresses();
   readFromEEPROMSlotNumber(0,true);  // read slot from first EEPROM slot if available !  

   blinkCount=10;  blinkStartTime=25;
   
   if (DebugOutput==DEBUG_FULLOUTPUT) 
   {   Serial.print("Free RAM:");  Serial.println(freeRam());}
   
}

int ccc=0;
char str[90];

///////////////////////////////
// Loop: the main program loop
///////////////////////////////

void loop() { 
 
    pressure = analogRead(PRESSURE_SENSOR_PIN);
    
    up =       (uint16_t)((uint32_t)analogRead(UP_SENSOR_PIN)  * settings.gd/50); if (up>MAX_ADC_VALUE) up=MAX_ADC_VALUE; if (up<0) up=0;
    down =     (uint16_t)((uint32_t)analogRead(DOWN_SENSOR_PIN) * settings.gu/50); if (down>MAX_ADC_VALUE) down=MAX_ADC_VALUE; if (down<0) down=0;
    left =     (uint16_t)((uint32_t)analogRead(LEFT_SENSOR_PIN) * settings.gr/50); if (left>MAX_ADC_VALUE) left=MAX_ADC_VALUE; if (left<0) left=0;
    right =    (uint16_t)((uint32_t)analogRead(RIGHT_SENSOR_PIN) * settings.gl/50); if (right>MAX_ADC_VALUE) right=MAX_ADC_VALUE; if (right<0) right=0;

    switch (settings.ro) {
      case 90: tmp=up; up=left; left=down; down=right; right=tmp; break;
      case 180: tmp=up; up=down; down=tmp; tmp=right; right=left; left=tmp; break;
      case 270: tmp=up; up=right; right=down; down=left; left=tmp;break;
    }

    while (Serial.available() > 0) {
      // get incoming byte:
      inByte = Serial.read();
      parseByte (inByte);      // implemented in parser.cpp
    }

    if (StandAloneMode)  {              
          if (calib_zero == 0)  {      // no calibration, use current values for x and y offset !
              x = (left-right) - cx;
              y = (up-down) - cy;
          }
          else  {
              calib_zero--;           // wait for calibration
              if (calib_zero ==0) {   // calibrate now !! get new offset values
                 settings.cx = (left-right);                                                   
                 settings.cy = (up-down);
                 cx=settings.cx;
                 cy=settings.cy;
              }
          }    

          // Calculate polar coordinates
          force=sqrt(x*x+y*y);
          if (force!=0) {
            angle = atan2 ((double)y/force, (double)x/force );
            dz= settings.dx * (fabs((double)x)/force) + settings.dy * (fabs((double)y)/force);
          }
          else { angle=0; dz=settings.dx; }

          if (force > MAX_ADC_VALUE) force=MAX_ADC_VALUE;
          // Apply Deadzone and update x and y
          if (force<dz) force=0; else force-=dz;
          x=(int)(force*cos(angle));
          y=(int)(force*sin(angle));

          reportValues();     // send live data to serial 

          if (calib_coordinates) 
              calib_coordinates=handle_coordinate_calibration(); // calibrate polar coordinates
          else {
              apply_mapping();
              handleModeState(x, y, pressure);  // handle all mouse / joystick / button activities
          }
          while (loopTime < LOOP_WAIT_TIME);
          loopTime=0; // -=LOOP_WAIT_TIME;
    }  
    else 
       handleCimMode();   // create periodic reports if running in AsTeRICS CIM compatibility mode

    UpdateLeds();
    UpdateTones();
}

void apply_mapping() {
  uint8_t primaryIndex=0,secondaryIndex;
  double minDifference=2*PI,angleDifference[4],secondaryDifference;

  //sprintf(str,"pre mapping force=%04d, angle=%04d",(int)(force),(int)(angle*180/PI)); 
  //Serial.println(str);

  if (customCoordinates[3].force==0) return; 

  // find nearest vector from saved directions (0:up 1:down 2:left 3:right)
  for (int i=0;i<4;i++) { 
     angleDifference[i]=calcAngleDifference(angle,customCoordinates[i].angle);
     if (minDifference>fabs(angleDifference[i])) {
        minDifference=fabs(angleDifference[i]);
        primaryIndex=i;
     }
  }

  // find correct (left or right) neighbour
  if (calcAngleDifference(angle,customCoordinates[primaryIndex].angle) < 0)
    secondaryIndex= primaryIndex>0?primaryIndex-1:3;
  else secondaryIndex= primaryIndex<3?primaryIndex+1:0;
  
  secondaryDifference=fabs(calcAngleDifference(angle,customCoordinates[secondaryIndex].angle));

  // calculate ratios to both neighbours
  double totalDifference = fabs(calcAngleDifference(customCoordinates[primaryIndex].angle,customCoordinates[secondaryIndex].angle));
  if (totalDifference==0) return;
  double f1=1.0-minDifference/totalDifference;
  // double f2=1.0-secondaryDifference/totalDifference; if (f2<0) f2=0;
  double f2=1.0-f1;

  // estimate force level for current vector
  double estimatedForce=f1*customCoordinates[primaryIndex].force + f2*customCoordinates[secondaryIndex].force;
  double forceFactor= force/estimatedForce;

  // map current vector to standard coordinates
  double mappedForce=f1*standardCoordinates[primaryIndex].force + f2*standardCoordinates[secondaryIndex].force;
  mappedForce*=forceFactor;

  if (((primaryIndex == 0) && (secondaryIndex == 3)) || ((primaryIndex == 3) && (secondaryIndex == 0)))
    standardCoordinates[3].angle=-PI;
  else standardCoordinates[3].angle=PI;
   
  angle=f1*standardCoordinates[primaryIndex].angle + f2*standardCoordinates[secondaryIndex].angle;
  
  x=(int)(mappedForce*cos(angle));
  y=(int)(mappedForce*sin(angle));
  
  if ((force) && ((ccc++) % 10 == 0)) {
    sprintf(str,"primary=%01d (%03d), secondary=%01d (%03d), angle=%04d, force=%04d",
                 primaryIndex,(int)(f1*100), secondaryIndex,(int)(f2*100), (int)(angle*180/PI), (int)(forceFactor*100)); 
    Serial.println(str);
    /*
    sprintf(str,"nearest: vektor=%01d, distance=%04d",primaryIndex,(int)(minDifference*180/PI)); 
    Serial.println(str);
    sprintf(str,"   next: vektor=%01d, distance=%04d",n,(int)(ad*180/PI)); 
    Serial.println(str);
    */
  }
}

double calcAngleDifference(double a1, double a2)
{
    double ad1,ad2;
    ad1=a1-a2;

    if (a1>a2) 
      ad2= a1-(a2+2*PI);  
    else ad2= (a1+2*PI)-a2;        

    if (fabs(ad1)<fabs(ad2)) return(ad1);
    else return(ad2);
}
/*
double calcAngleDifference(double a1, double a2)
{
    double ad1,ad2;
    ad1=fabs(a1-a2);

    if (a1>a2) 
      ad2=fabs(a1-(a2+2*PI));  
    else ad2=fabs((a1+2*PI)-a2);        

    if (ad1<ad2) return(ad1);
    else return(ad2);
}

 */

uint8_t handle_coordinate_calibration()
{
    static int calibCount=0;
    static uint8_t calibState=0;
    static double maxForce=0;
              
    if (force>maxForce) { 
      maxForce=force; 
      customCoordinates[calibState].force = force; 
      customCoordinates[calibState].angle = angle; 

      sprintf(str,"Calib state %d: X=%04d/Y=%04d -> Force=%04d, Angle=%04d",calibState,x,y,(int)force, (int)(angle*180/PI)); 
      Serial.println(str);
    }

    if (!(calibCount++ % CALIB_COORDINATES_TONEINTERVAL))
        tone(TONE_PIN, 100+200*calibState, 15);

    if (calibCount==CALIB_COORDINATES_TIME) {
      calibCount=0; 
      maxForce=0; 
      delay(1000);
      if (++calibState==4) {
        calibState=0;
        Serial.println("Calibration done:");
        sprintf(str,"   Up: Force=%04d, Angle=%04d",(int)customCoordinates[0].force, (int)(customCoordinates[0].angle*180/PI)); 
        Serial.println(str);
        sprintf(str,"Right: Force=%04d, Angle=%04d",(int)customCoordinates[1].force, (int)(customCoordinates[1].angle*180/PI)); 
        Serial.println(str);
        sprintf(str," Down: Force=%04d, Angle=%04d",(int)customCoordinates[2].force, (int)(customCoordinates[2].angle*180/PI)); 
        Serial.println(str);
        sprintf(str," Left: Force=%04d, Angle=%04d",(int)customCoordinates[3].force, (int)(customCoordinates[3].angle*180/PI)); 
        Serial.println(str);
        return(0);    // calibration done!
      }
    }
    return(1);
}


int calcDistance(int x1, int y1, int x2, int y2)
{
   return (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}

void reportValues()
{
    static uint8_t valueReportCount =0; 
    if (!reportRawValues)   return; 
    
    if (valueReportCount++ > 10) {                    // report raw values !
      Serial.print("VALUES:");Serial.print(pressure);Serial.print(",");
      Serial.print(up);Serial.print(",");Serial.print(down);Serial.print(",");
      Serial.print(left);Serial.print(",");Serial.print(right);Serial.print(",");
      Serial.print(x);Serial.print(",");Serial.println(y);
      /*
      Serial.print("AnalogRAW:");
      Serial.print(analogRead(UP_SENSOR_PIN));
      Serial.print(",");
      Serial.print(analogRead(DOWN_SENSOR_PIN));
      Serial.print(",");
      Serial.print(analogRead(LEFT_SENSOR_PIN));
      Serial.print(",");
      Serial.println(analogRead(RIGHT_SENSOR_PIN));
      */
      valueReportCount=0;
    }
}

void release_all()  // releases all previously pressed keys
{
    release_all_keys();
    mouseRelease(MOUSE_LEFT);
    mouseRelease(MOUSE_MIDDLE);
    mouseRelease(MOUSE_RIGHT);
    moveX=0;
    moveY=0;
}

void initBlink(uint8_t  count, uint8_t startTime)
{
    blinkCount=count;
    blinkStartTime=startTime;
}

void UpdateLeds()
{  
  if (StandAloneMode)
  {
       digitalWrite(LED_PIN,LOW); 
       
       if (blinkCount==0) {
         if ((actSlot+1) & 1) digitalWrite (led_map[0],LOW); else digitalWrite (led_map[0],HIGH); 
         if ((actSlot+1) & 2) digitalWrite (led_map[1],LOW); else digitalWrite (led_map[1],HIGH); 
         if ((actSlot+1) & 4) digitalWrite (led_map[2],LOW); else digitalWrite (led_map[2],HIGH); 
        }
        else {
          if (blinkTime==0)
          {
             blinkTime=blinkStartTime;
             blinkCount--;
             if (blinkCount%2) { 
                 digitalWrite (led_map[0],LOW); digitalWrite (led_map[1],LOW); digitalWrite (led_map[2],LOW); }
              else { 
                 digitalWrite (led_map[0],HIGH); digitalWrite (led_map[1],HIGH); digitalWrite (led_map[2],HIGH); }
          } else blinkTime--;
       }
  }
  else
     digitalWrite(LED_PIN,HIGH);       
}

uint16_t toneHeight;
uint16_t toneOnTime;
uint16_t toneOffTime;
uint16_t toneCount=0;

void UpdateTones()
{  
  static uint16_t toneState=0;
  static uint16_t cnt=0;

  if (!toneCount) return;

  uint8_t tonePin = TONE_PIN;

  switch(toneState) {
      case 0:
            tone(tonePin, toneHeight, toneOnTime);
            toneState++;
            break;
      case 1:
            if (++cnt > (toneOnTime+toneOffTime)/5 )  {
              toneCount--;
              toneState=0;
              cnt=0;
            }
            break;
    }
  }


void makeTone(uint8_t kind, uint8_t param)
{
	uint8_t tonePin = TONE_PIN;
		
    switch (kind) {
		case TONE_ENTER_STRONGPUFF: 
			tone(tonePin, 400, 200);
            break;
		case TONE_EXIT_STRONGPUFF: 
			tone(tonePin, 400, 100);
            break;
		case TONE_CALIB: 
			tone(tonePin, 200, 400);
            break;
		case TONE_CHANGESLOT:
          if (!toneCount) {
            toneHeight=2000+200*param;
            toneOnTime=150;
            toneOffTime=50;
            toneCount=param+1;
          }
          break;
		case TONE_ENTER_STRONGSIP:
  		tone(tonePin, 300, 200); 
  				break;
    case TONE_EXIT_STRONGSIP:
      tone(tonePin, 300, 100); 
          break;
		case TONE_IR:
			tone(tonePin, 2500, 30);
			break;
    case TONE_IR_REC:
      tone(tonePin, 350, 500);
      break;
    case TONE_INDICATE_SIP:
      tone(tonePin, 5000, 5);
      break;
    case TONE_INDICATE_PUFF:
      tone(tonePin, 4000, 5);
      break;
    case TONE_BT_PAIRING:
      tone(tonePin, 230, 4000);
      break;
     }
}

int freeRam ()  // TBD: has to be adapted for TeensyLC ...
{
//    extern int __heap_start, *__brkval;
//    int v;
//    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
      return(1);
}
