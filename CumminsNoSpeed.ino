
// Cummins Commander by LH
// Engine speed control and timed intake heater command for '1st generation' Dodge trucks with 5.9l Cummins Turbo Diesel engine.
// This sketch utilizes the ADAFruit TFT libraries modified by Joao Lopes to work on low cost 2.4" TFTs not normally supported.

// Modified for SPFD5408 Library by Joao Lopes
// Version 0.9.2 - Rotation for Mega

// *** SPFD5408 change -- Begin
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>
// *** SPFD5408 change -- End

#include <EEPROM.h>

#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif

#define YP A3  // must be an analog pin, use "An" notation!
#define XM A2  // must be an analog pin, use "An" notation!
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// Original values
//#define TS_MINX 150
//#define TS_MINY 120
//#define TS_MAXX 920
//#define TS_MAXY 940

// Touchscreen Calibration.   These values are specific to each touchscreen/LCD combination.
#define TS_MINX 100
#define TS_MINY 96
#define TS_MAXX 822
#define TS_MAXY 836

#define MINPRESSURE 10
#define MAXPRESSURE 1000

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

#define BOXSIZE 62
#define BOXLENGTH 180
#define BOXHEIGHT 50
#define BOXXOFFSET 30
#define TEXTYOFFSET 18
#define TITLEFONTSIZE 2

#define HEATER 12  //Active low relay
#define VENT 10    //Active low relay
#define VAC  11   //Active low relay
#define BRAKE A5

byte servoVentPulse;   //pulse width in ms
byte servoVacPulse;   //pulse width in ms


boolean controlMode;   //0 = Off(not controlling speed) 1 = On  2


void setup(void) {

  pinMode(HEATER, OUTPUT);
  pinMode(VENT, OUTPUT);
  pinMode(VAC, OUTPUT);
  pinMode(BRAKE, INPUT);
  
  digitalWrite(HEATER, HIGH);
  digitalWrite(VENT, HIGH);
  digitalWrite(VAC, HIGH);

  initLCD();

  if (EEPROM.read(0) != 1){  //if EEPROM is empty, load default values and update EEPROM
    loadDefaults();
  }else readEm();   //load stored values from EEPROM

  controlMode = 0;

  drawSplash();
  drawIHCScreen();
  
  TSPoint p = waitOneTouch();

  if (p.y < (BOXSIZE * 2)) {
    delay(500);
  } else if (p.y < (BOXSIZE * 3)) {
    drawWTSScreen(5);
    afterStartCycle(5);
  } else if (p.y < (BOXSIZE * 4)) {
    drawWTSScreen(10);
    afterStartCycle(10);
  } else if (p.y < (BOXSIZE * 5)) {
    drawWTSScreen(15);
    afterStartCycle(15);
  }

  drawESCScreen();
  drawControl();
}



void loop() {

  if ((digitalRead(BRAKE) == LOW) && (controlMode == 1)) { //if brake is pressed and speed control active, cancel control
   cancel();
  }
  
  TSPoint p = getTouchData();       // we have some minimum pressure we consider 'valid'

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {     //Check for valid touch

    if (p.y < (BOXSIZE * 2)) {//  Control/cancel button
      if(controlMode == 0){
         controlMode = 1;
         drawCancel();
         digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)                                    
      } else if (controlMode == 1) {  //button shows cancel
        cancel();
      }
    } else if (p.y < (BOXSIZE * 3)) {  //speed up button
         if(controlMode == 1){
           pulseVac();
         }          
    } else if (p.y < (BOXSIZE * 4)) {  //reduce speed button
         if(controlMode == 1){
           pulseVent();
         }              
    } else if ((p.y < (BOXSIZE * 5)) && controlMode == 0) {// adjust button, only active when not controlling speed
      drawPAS();
      drawESCScreen();
      drawControl();
    }

    delay(200);    //sort of a switch debouncer
  }

}




void  initLCD () {    //Initalize TFT LCD    Modified from ADAFruit to work with cheap generic 2.4" TFTs with touch

  tft.reset();

  tft.begin(0x9341); // SDFP5408
  tft.setRotation(2); // 0 and 2 are portrait. Doesn't currently work in landscape.  Touch will auto-orientate.

  // *** SPFD5408 change -- End

}




TSPoint getTouchData() {        //Read data from touchscreen

  TSPoint p;

  //digitalWrite(13, HIGH);  Not sure why these lines are here but I needed pin 13 and it seems to work without.
  p = ts.getPoint();
  // digitalWrite(13, LOW);

  pinMode(XM, OUTPUT);   //  fix the directions of the touchscreen pins
  pinMode(YP, OUTPUT);

  // scale from 0->1023 to tft.width
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
  // *** SPFD5408 change -- End
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());;


  if((tft.getRotation()) == 2){  //correct touchscreen values to match TFT orientation
    p.x = (tft.width() - p.x);
    p.y = (tft.height() - p.y);
  }
  
  return p;

}




TSPoint waitOneTouch() {         // wait 1 touch to exit function

  TSPoint p;

  do {
    p = getTouchData();

  } while ((p.z < MINPRESSURE ) || (p.z > MAXPRESSURE));

  return p;
}




void drawBorder () {             //Draw border

  uint16_t width = tft.width() - 1;
  uint16_t height = tft.height() - 1;
  uint8_t border = 4;

  tft.fillScreen(WHITE);
  tft.fillRect(border, border, (width - border * 2), (height - border * 2), BLACK);

}



void drawSplash () {     // Draw splashscreen

  drawBorder();

  tft.setCursor (58, 50);
  tft.setTextSize (3);
  tft.setTextColor(WHITE);
  tft.println("Cummins");
  tft.setCursor (40, 85);
  tft.println("Commander");  //I really didn't try too hard to think of a name
  tft.setCursor (110, 125);
  tft.setTextSize (TITLEFONTSIZE);
  tft.println("by");
  tft.drawCircle(120,218, 57, BLUE);
  for (int j = 0; j < 28; j++) {
    tft.drawCircle(120,218, j*2, CYAN);
  }  
  tft.setTextColor(WHITE);
  tft.setCursor (87, 184);
  tft.setTextSize (7);
  tft.println("L");         //go ahead, put your name here. I know the truth
  tft.setCursor (116, 205);
  tft.println("H");
  delay(2500);

}



void drawIHCScreen () {             //Draw intake heat control screen

  drawBorder();
  tft.setCursor (35, TEXTYOFFSET);
  tft.setTextSize (TITLEFONTSIZE);
  tft.setTextColor(WHITE);
  tft.println("Intake Preheat");

  tft.fillRect(BOXXOFFSET, BOXSIZE,   BOXLENGTH, BOXHEIGHT, RED);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, 0xFD35);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 3), BOXLENGTH, BOXHEIGHT, 0x07FF);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, BLUE);

  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 65, BOXSIZE + TEXTYOFFSET);
  tft.println("NONE");
  tft.setCursor (BOXXOFFSET + 60, (BOXSIZE * 2) + TEXTYOFFSET);
  tft.println("5 Sec");
  tft.setCursor (BOXXOFFSET + 55, (BOXSIZE * 3) + TEXTYOFFSET);
  tft.println("10 Sec");
  tft.setCursor (BOXXOFFSET + 55, (BOXSIZE * 4) + TEXTYOFFSET);
  tft.println("15 Sec");

}



void drawWTSScreen (int t) {             //Draw wait to start screen

  drawBorder();
  tft.setTextSize (3);
  tft.setTextColor(RED);
  tft.setCursor (57, TEXTYOFFSET);
  tft.println("WAIT TO");
  tft.setCursor (77, TEXTYOFFSET + 35);
  tft.println("START");
  tft.setTextSize (8);

  heatOn();   //Activate output pin to control preheat relay

  do {
    tft.setTextColor(RED);
    tft.setCursor (90, 120);
    tft.println(t);
    delay(1000);
    tft.setTextColor(BLACK);
    tft.setCursor (90, 120);
    tft.println(t);
    t = t - 1;
  } while (t > 0);

  heatOff();    //Deactivate output pin

}

void afterStartCycle (byte t) {   //continues to operate the intake heater after engine start(if you did indeed start the engine when the Wait to Start disappeared)

  drawBorder();

  tft.setTextSize (3);
  tft.setTextColor(RED);
  tft.setCursor (57, TEXTYOFFSET);
  tft.println("WARM UP");
  tft.setCursor (77, TEXTYOFFSET + 35);
  tft.println("CYCLE");

  if (t == 5) {   //completely arbitrary cycle times, does keep the smoke down nicely.

    delay(7000);  //stay off for 7sec
    heatOn();
    delay(5000);   //on for 5sec
    heatOff();     //back to off

  } else if (t == 10) {

    delay(7000);  //stay off for 7sec
    heatOn();
    delay(7000);   //on for 7sec
    heatOff();     //back to off
    delay(7000);  //stay off for 7sec
    heatOn();
    delay(5000);   //on for 5sec
    heatOff();     //back to off

  } else if (t == 15) {

    delay(5000);  //stay off for 5sec
    heatOn();
    delay(8000);   //on for 8sec
    heatOff();     //back to off
    delay(7000);  //stay off for 7sec
    heatOn();
    delay(7000);   //on for 5sec
    heatOff();     //back to off
    delay(7000);  //stay off for 7sec
    heatOn();
    delay(6000);   //on for 6sec
    heatOff();     //back to off
  }

}

void drawESCScreen () {             //Draw engine speed control screen


  drawBorder();
  tft.setCursor (42, TEXTYOFFSET);
  tft.setTextSize (TITLEFONTSIZE);
  tft.setTextColor(WHITE);
  tft.println("Speed Control");

}

void drawCancel() {   //control mode buttons
  tft.setTextSize (4);
  tft.fillRect(BOXXOFFSET, BOXSIZE, BOXLENGTH, BOXHEIGHT, RED);
  tft.setTextColor(WHITE);
  tft.setCursor (BOXXOFFSET + 20, BOXSIZE + (TEXTYOFFSET - 7));
  tft.println("CANCEL");

  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextSize (7);
  tft.setTextColor(RED);
  tft.setCursor (BOXXOFFSET + 72, (BOXSIZE * 2) + (TEXTYOFFSET - 18));
  tft.println("+");
  
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 3), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextSize (7);
  tft.setTextColor(BLUE);
  tft.setCursor (BOXXOFFSET + 72, (BOXSIZE * 3) + (TEXTYOFFSET - 18));
  tft.println("-");
  
  tft.fillRect(BOXXOFFSET, BOXSIZE * 4, BOXLENGTH, BOXHEIGHT, BLACK);
}


void drawControl() {  //changes buttons to no control mode
  tft.setTextSize (4);
  tft.fillRect(BOXXOFFSET, BOXSIZE, BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 9, BOXSIZE + (TEXTYOFFSET - 7));
  tft.println("CONTROL");
  
  tft.fillRect(BOXXOFFSET, BOXSIZE * 2, BOXLENGTH, BOXHEIGHT, BLACK);
  tft.fillRect(BOXXOFFSET, BOXSIZE * 3, BOXLENGTH, BOXHEIGHT, BLACK);
  
  tft.fillRect(BOXXOFFSET, BOXSIZE *4 , BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextSize (4);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 55, (BOXSIZE * 4) + (TEXTYOFFSET - 7));
  tft.println("ADJ");
}



void drawPlus() {
  tft.setTextSize (6);
  tft.setCursor (175, 250);
  tft.setTextColor(RED);
  tft.println("+");
}

void erasePlus() {
  tft.setTextSize (6);
  tft.setCursor (175, 250);
  tft.setTextColor(BLACK);
  tft.println("+");
}

void drawMinus() {
  tft.setTextSize (6);
  tft.setCursor (45, 250);
  tft.setTextColor(BLUE);
  tft.println("-");
}

void eraseMinus() {
  tft.setTextSize (6);
  tft.setCursor (45, 250);
  tft.setTextColor(BLACK);
  tft.println("-");
}


void drawPAS() {   //Draw parameter adjustment screen

  tft.fillScreen(BLACK);
  tft.setCursor (42, 10);
  tft.setTextSize (TITLEFONTSIZE);
  tft.setTextColor(WHITE);
  tft.println("Adjust Values");

  tft.fillRect(BOXXOFFSET, BOXSIZE,   BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 3), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, WHITE);

  tft.setTextSize (7);
  tft.setTextColor(RED);
  tft.setCursor (BOXXOFFSET + 72, BOXSIZE + (TEXTYOFFSET - 18));
  tft.println("+");
  tft.setTextColor(BLUE);
  tft.setCursor (BOXXOFFSET + 72, (BOXSIZE * 3) + (TEXTYOFFSET - 18));
  tft.println("-");
  tft.setTextSize (4);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 44, (BOXSIZE * 4) + (TEXTYOFFSET - 7));
  tft.println("NEXT");
  adjParameters();
}

void drawUVS() {   //Draw parameter adjustment screen

  tft.fillScreen(BLACK);
  tft.setCursor (42, 10);
  tft.setTextSize (TITLEFONTSIZE);
  tft.setTextColor(WHITE);
  tft.println("Update Values");

  tft.fillRect(BOXXOFFSET, BOXSIZE,   BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 3), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, WHITE);

  tft.setTextSize (4);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 43, BOXSIZE + (TEXTYOFFSET - 7));
  tft.println("SAVE");
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 55, (BOXSIZE * 2) + (TEXTYOFFSET - 7));
  tft.println("TRY");
  tft.setTextColor(RED);
  tft.setCursor (BOXXOFFSET + 20, (BOXSIZE * 3) + (TEXTYOFFSET - 7));
  tft.println("CANCEL");
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 9, (BOXSIZE * 4) + (TEXTYOFFSET - 7));
  tft.println("DEFAULT");
}

void drawVc() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("VacPulse");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(servoVacPulse);
}

void drawVn() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("VentPulse");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(servoVentPulse);
}

void writeEm(){
  
  EEPROM.put(2, servoVacPulse);
  EEPROM.put(3, servoVentPulse); 
  
}


void readEm(){
  
  EEPROM.get(2, servoVacPulse);
  EEPROM.get(3, servoVentPulse); 
  
}


void adjParameters() {

  drawVc();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (servoVacPulse<254){
          servoVacPulse++;
          drawVc();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (servoVacPulse>2){
          servoVacPulse--;
          drawVc();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (servoVacPulse > -1);

  drawVn();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (servoVentPulse<254){
        servoVentPulse++;
        drawVn();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (servoVentPulse>2){
        servoVentPulse--;
        drawVn();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (servoVentPulse > -1);

  exitadj:;

  drawUVS();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < (BOXSIZE * 2)) {                   //save to ROM
        writeEm();
        break;
      } else if (p.y < (BOXSIZE * 3)) {             //try without saving
        break;  
      } else if (p.y < (BOXSIZE * 4)) {             //forget recent changes
        readEm();
        break;  
      } else if (p.y < (BOXSIZE * 5)) {               //load defaults
        loadDefaults();
        break;
      }
    }
  } while (servoVentPulse > -1);

}



void heatOn () {
  digitalWrite(HEATER, LOW);//Activate heater
}



void heatOff () {
  digitalWrite(HEATER, HIGH);//Deactivate heater
}


void pulseVent () {

  drawMinus();

  digitalWrite(VENT, HIGH);//Deactivate Servo Vent control   (vent vaccuum, reduce throttle)
  delay(servoVentPulse);
  digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)

  eraseMinus();
}



void pulseVac () {

  drawPlus();

  digitalWrite(VAC, LOW); //Activate Servo Vaccuum control   (Add vaccuum, increase throttle)
  delay(servoVacPulse);
  digitalWrite(VAC, HIGH);//Deactivate Servo Vaccuum control  (Stop adding vaccuum)

  erasePlus();
}



void cancel () {

  digitalWrite(VENT, HIGH);//Deactivate Servo Vent control   (vent vaccuum, reduce throttle)
  digitalWrite(VAC, HIGH);//Deactivate Servo Vaccuum control
  controlMode = 0;
  drawControl();
}


void loadDefaults(){
 
    servoVentPulse = 100;  //pulse width in ms
    servoVacPulse = 100;  //pulse width in ms
    
    writeEm();
    EEPROM.put(0, 1);
}

