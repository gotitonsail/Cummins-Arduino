
// Cummins Commander by LH
// PID Cruise control and timed intake heater command for '1st generation' Dodge trucks with 5.9l Cummins Turbo Diesel engine.
// This sketch utilizes the ADAFruit TFT libraries modified by Joao Lopes to work on low cost 2.4" TFTs not normally supported.

// Modified for SPFD5408 Library by Joao Lopes
// Version 0.9.2 - Rotation for Mega

// *** SPFD5408 change -- Begin
#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>
// *** SPFD5408 change -- End

#include <EEPROM.h>

#include <PID_v1.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptBoards.h>

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
#define SPEEDIN 13  //5Vdc square wave converted from vehicle or engine speed sensor.
#define BRAKE A5

byte sampleNum; //number of samples used to calculate average speed.

byte speedStep;    //increment when desired speed is adjusted using +/- buttons.
int speedTimeStep;   //delay between speed info refresh
int maintStep;   //delay between PID automatic servo adjustments

byte servoVentPulse;   //pulse width in ms
byte servoVacPulse;   //pulse width in ms

int Ki;
int Kp;
int Kd;

double pidOutput;

unsigned long spdTimer;
double currentSpeed;
int oldSpeed;
double desiredSpeed;
int oldDesired;
volatile boolean oldMode;    //0 = auto speed control has NOT yet been used 1 = Resume enabled, override disabled
volatile byte controlMode;   //0 = Off(not controlling speed) 1 = On  2 = Override

PID speedPID(&currentSpeed, &pidOutput, &desiredSpeed,Kp,Ki,Kd, DIRECT);  //(In,Out,Set,Kp,Ki,Kd,Dir)


ISR (PCINT1_vect) { // handle pin change interrupt for A0 to A5 here

  if ((digitalRead(BRAKE) == LOW) && (controlMode > 0)) { //check to see if brake signal changed
   cancel();
  }
  do {
    delay(1);
  } while (digitalRead(BRAKE) == LOW);
}  // end of PCINT1_vect



void setup(void) {

  pinMode(HEATER, OUTPUT);
  pinMode(VENT, OUTPUT);
  pinMode(VAC, OUTPUT);
  pinMode(BRAKE, INPUT);
  pinMode(SPEEDIN, INPUT);

  digitalWrite(HEATER, HIGH);
  digitalWrite(VENT, HIGH);
  digitalWrite(VAC, HIGH);

  PCMSK1 |= bit (PCINT13);  // want pin A5     The interrupt code is from Gammon's great write-up.
  PCIFR  |= bit (PCIF1);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE1);   // enable pin change interrupts for A0 to A5

  initLCD();

  if (EEPROM.read(0) != 1){  //if EEPROM is empty, load default values and update EEPROM
    loadDefaults();
  }else readEm();   //load stored values from EEPROM


  currentSpeed = 0;
  oldSpeed = 0;
  oldMode = 0;
  oldDesired = 0;
  desiredSpeed = 0;
  controlMode = 0;
  spdTimer = millis();
 
  speedPID.SetTunings(Kp, Ki, Kd);
  speedPID.SetOutputLimits(-100, 100);
  speedPID.SetSampleTime(maintStep);
  
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
  drawCurrentSpeed();
 

}



void loop() {

  if(controlMode == 1) {
    maintainSpeed();    
  }
  
  if ((millis() - spdTimer) >= speedTimeStep) {

    oldSpeed = currentSpeed;
    //currentSpeed = getRPM();       //disabled till I get a good incoming speed signal

    if (currentSpeed != oldSpeed) {  // no reason to run if there is no change
      drawCurrentSpeed();
    }
    spdTimer = millis();
  }

  TSPoint p = getTouchData();
  // we have some minimum pressure we consider 'valid'
  // pressure of 0 means no pressing!

  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {     //Check for valid touch

    if ((p.y < BOXSIZE) && (controlMode == 0)) {
      drawPAS();

      drawESCScreen();
      drawCurrentSpeed();

    } else if (p.y < (BOXSIZE * 2)) {
      oldDesired = desiredSpeed;
      desiredSpeed = currentSpeed;
      oldMode = 1;
      controlMode = 1;
      speedPID.SetMode(AUTOMATIC);
      drawCancel();
      digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)
      drawDesired();

    } else if (p.y < (BOXSIZE * 3)) {
      if (controlMode == 1) {
        oldDesired = desiredSpeed;
        desiredSpeed = desiredSpeed + speedStep;
        drawDesired();
      } else if (controlMode == 2) {
        pulseVac();
      }
    } else if (p.y < (BOXSIZE * 4)) {
      if ((controlMode == 1 && desiredSpeed > 0)) {
        oldDesired = desiredSpeed;
        desiredSpeed = desiredSpeed - speedStep;
        drawDesired();
      } else if (controlMode == 2) {
        pulseVent();
      }

    } else if (p.y < (BOXSIZE * 5)) {
      if ((controlMode == 0 && oldMode == 0)) { //button shows override
        controlMode = 2;  //override
        digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)
        drawCancel();                          //now shows cancel
      } else if ((controlMode == 0 && oldMode == 1)) { //now shows resume
        controlMode = 1;  //control
        speedPID.SetMode(AUTOMATIC);
        digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)
        drawCancel();                  //now shows cancel
      } else if (controlMode > 0) {  //button shows cancel
        cancel();
      }

    }

    delay(300);
  }

}




void  initLCD () {    //Initalize TFT LCD    Modified from ADAFruit to work with cheap generic 2.4" TFTs with touch

  tft.reset();

  // *** SPFD5408 change -- Begin
  //  uint16_t identifier = tft.readID();
  //
  //  if(identifier == 0x9325) {
  //    Serial.println(F("Found ILI9325 LCD driver"));
  //  } else if(identifier == 0x9328) {
  //    Serial.println(F("Found ILI9328 LCD driver"));
  //  } else if(identifier == 0x7575) {
  //    Serial.println(F("Found HX8347G LCD driver"));
  //  } else if(identifier == 0x9341) {
  //    Serial.println(F("Found ILI9341 LCD driver"));
  //  } else if(identifier == 0x8357) {
  //    Serial.println(F("Found HX8357D LCD driver"));
  //  } else {
  //    Serial.print(F("Unknown LCD driver chip: "));
  //    Serial.println(identifier, HEX);
  //    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
  //    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
  //    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
  //    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
  //    Serial.println(F("Also if using the breakout, double-check that all wiring"));
  //    Serial.println(F("matches the tutorial."));
  //    return;
  //  }
  //
  //  tft.begin(identifier);

  tft.begin(0x9341); // SDFP5408
  tft.setRotation(2); // 0 and 2 are portrait. Doesn't currently work in landscape.  Touch will auto-orientate.

  // *** SPFD5408 change -- End

}




TSPoint getTouchData() {        //Read data from touchscreen

  TSPoint p;

  //digitalWrite(13, HIGH);  Not sure why these lines are here but I needed pin 13 and it seems to work without.
  p = ts.getPoint();
  // digitalWrite(13, LOW);

  pinMode(XM, OUTPUT);   // if sharing pins, you'll need to fix the directions of the touchscreen pins
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
  tft.println("Commander");
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
  tft.println("L");
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

void afterStartCycle (byte t) {

  drawBorder();

  tft.setTextSize (3);
  tft.setTextColor(RED);
  tft.setCursor (57, TEXTYOFFSET);
  tft.println("WARM UP");
  tft.setCursor (77, TEXTYOFFSET + 35);
  tft.println("CYCLE");

  if (t == 5) {

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

  }

}

void drawESCScreen () {             //Draw engine speed control screen


  drawBorder();
  tft.setCursor (42, TEXTYOFFSET);
  tft.setTextSize (TITLEFONTSIZE);
  tft.setTextColor(WHITE);
  tft.println("Speed Control");

  tft.fillRect(BOXXOFFSET, BOXSIZE,   BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 3), BOXLENGTH, BOXHEIGHT, WHITE);

  tft.setTextSize (4);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 55, BOXSIZE + (TEXTYOFFSET - 7));
  tft.println("SET");
  tft.setTextSize (7);
  tft.setTextColor(RED);
  tft.setCursor (BOXXOFFSET + 72, (BOXSIZE * 2) + (TEXTYOFFSET - 18));
  tft.println("+");
  tft.setTextColor(BLUE);
  tft.setCursor (BOXXOFFSET + 72, (BOXSIZE * 3) + (TEXTYOFFSET - 18));
  tft.println("-");
  drawOverride();

}

void drawCancel() {
  tft.setTextSize (4);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, RED);
  tft.setTextColor(WHITE);
  tft.setCursor (BOXXOFFSET + 20, (BOXSIZE * 4) + (TEXTYOFFSET - 7));
  tft.println("CANCEL");
}


void drawResume() {
  tft.setTextSize (4);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextColor(BLACK);
  tft.setCursor (BOXXOFFSET + 20, (BOXSIZE * 4) + (TEXTYOFFSET - 7));
  tft.println("RESUME");
}


void drawOverride() {
  tft.setTextSize (3);
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 4), BOXLENGTH, BOXHEIGHT, WHITE);
  tft.setTextColor(RED);
  tft.setCursor (BOXXOFFSET + 20, (BOXSIZE * 4) + (TEXTYOFFSET - 4));
  tft.println("OVERRIDE");
}

void drawPlus() {
  tft.setTextSize (4);
  tft.setCursor (175, 70);
  tft.setTextColor(RED);
  tft.println("+");
}

void erasePlus() {
  tft.setTextSize (4);
  tft.setCursor (175, 70);
  tft.setTextColor(WHITE);
  tft.println("+");
}

void drawMinus() {
  tft.setTextSize (4);
  tft.setCursor (45, 70);
  tft.setTextColor(BLUE);
  tft.println("-");
}

void eraseMinus() {
  tft.setTextSize (4);
  tft.setCursor (45, 70);
  tft.setTextColor(WHITE);
  tft.println("-");
}

void drawDesired() {
  tft.setTextSize (TITLEFONTSIZE);
  tft.setCursor (38, 40);
  tft.setTextColor(BLACK);
  tft.println(int(oldDesired));
  tft.setCursor (38, 40);
  tft.setTextColor(RED);
  tft.println(int(desiredSpeed));
}

void drawCurrentSpeed() {
  tft.setTextSize (TITLEFONTSIZE);
  tft.setCursor (115, 40);
  tft.setTextColor(BLACK);
  tft.println(int(oldSpeed));
  tft.setCursor (115, 40);
  tft.setTextColor(WHITE);
  tft.println(int(currentSpeed));
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

void drawts() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("SpeedRefresh");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(speedTimeStep);
}

void drawms() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("ServoDelay");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(maintStep);
}

void drawsn() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("SpeedSamples");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(sampleNum);
}

void drawKp() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("Kp");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(Kp);
}

void drawKi() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("Ki");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(Ki);
}

void drawKd() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("Kd");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(Kd);
}

void drawss() {
  tft.fillRect(BOXXOFFSET, (BOXSIZE * 2), BOXLENGTH, BOXHEIGHT, BLACK);
  tft.setTextSize (2);
  tft.setCursor (BOXXOFFSET + 12, (BOXSIZE * 2) + (TEXTYOFFSET - 16));
  tft.setTextColor(WHITE);
  tft.println("Increment");
  tft.setCursor (BOXXOFFSET + 65, (BOXSIZE * 2) + (TEXTYOFFSET + 10));
  tft.println(speedStep);
}

void writeEm(){
  
  EEPROM.put(2, servoVacPulse);
  EEPROM.put(3, servoVentPulse); 
  EEPROM.put(4, speedTimeStep);    //int
  EEPROM.put(6, maintStep);        //int
  EEPROM.put(8, sampleNum);
  EEPROM.put(9, speedStep);
  EEPROM.put(10, Kp);           //double
  EEPROM.put(14, Ki);
  EEPROM.put(18, Kd);
  
  
}


void readEm(){
  
  EEPROM.get(2, servoVacPulse);
  EEPROM.get(3, servoVentPulse); 
  EEPROM.get(4, speedTimeStep);    //int
  EEPROM.get(6, maintStep);        //int
  EEPROM.get(8, sampleNum);
  EEPROM.get(9, speedStep);
  EEPROM.get(10, Kp);           //double
  EEPROM.get(14, Ki);
  EEPROM.get(18, Kd);
  
}


void adjParameters() {

  drawVc();
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

  drawts();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (speedTimeStep<2000){ 
          speedTimeStep++;
          drawts();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (speedTimeStep>2){
          speedTimeStep--;
          drawts();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (speedTimeStep > 0);

  drawms();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (maintStep<2000){
          maintStep++;
          drawms();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (maintStep>2){
          maintStep--;
          drawms();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (maintStep > 0);

  drawsn();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (sampleNum<254){
          sampleNum++;
          drawsn();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (sampleNum>2){
          sampleNum--;
          drawsn();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (sampleNum > 0);

  drawKp();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (Kp<254){
          Kp++;
          drawKp();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (Kp>1){
          Kp--;
          drawKp();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (Kp > 0);

  drawKi();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (Ki<254){
          Ki++;
          drawKi();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (Ki>2){
          Ki--;
          drawKi();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (Ki > 0);

  drawKd();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (Kd<254){
          Kd++;
          drawKd();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (Kd>2){
          Kd--;
          drawKd();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (Kd > 0);

  drawss();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < BOXSIZE){
        goto exitadj;
      }else if (p.y < (BOXSIZE * 2)) {                   // add to value
        if (speedStep<254){
          speedStep++;
          drawss();
        }
      } else if (p.y < (BOXSIZE * 4)) {
        if (speedStep>2){
          speedStep--;
          drawss();
        }
      } else if (p.y < (BOXSIZE * 5)) {
        break;
      }
    }
  } while (speedStep > 0);
  exitadj:;

  drawUVS();
  delay(100);
  do {
    TSPoint p = getTouchData();
    if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
      if (p.y < (BOXSIZE * 2)) {                   //save to ROM
        writeEm();
        speedPID.SetTunings(Kp, Ki, Kd);
        break;
      } else if (p.y < (BOXSIZE * 3)) {             //try without saving
        speedPID.SetTunings(Kp, Ki, Kd);
        break;  
      } else if (p.y < (BOXSIZE * 4)) {             //forget recent changes
        readEm();
        break;  
      } else if (p.y < (BOXSIZE * 5)) {               //load defaults
        loadDefaults();
        speedPID.SetTunings(Kp, Ki, Kd);
        break;
      }
    }
  } while (speedStep > 0);

}

int getRPM () {

  unsigned long samplel;
  unsigned long sampleh;
  unsigned long suml;
  unsigned long sumh;
  unsigned long sum;

 for (int j = 0; j < sampleNum; j++) {
    
   samplel = pulseIn(SPEEDIN, LOW, 250000);
      //samplel = (samplel / 30);
      suml = (suml + samplel);
   sampleh = pulseIn(SPEEDIN, HIGH, 250000);
      //sampleh = (sampleh / 30);
      sumh = (sumh + sampleh);
 }
 suml = (suml / sampleNum);
 sumh = (sumh / sampleNum);
 sum = ((suml + sumh)/100);
 //sum = int(60 * (1 / (((sum * 100) / Duty))));
 
 return sum;
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

  if (controlMode == 1) {
    noInterrupts ();
    controlMode = 0;
    oldMode = 1;
    drawResume();
    speedPID.SetMode(MANUAL);
    interrupts ();
  } else if (controlMode == 2) {
    noInterrupts ();
    controlMode = 0;
    oldMode = 0;
    drawOverride();
    speedPID.SetMode(MANUAL);
    interrupts ();
  }

}

void maintainSpeed () {
  speedPID.Compute();
  if (pidOutput > 0) {

    drawPlus();
      digitalWrite(VAC, LOW); //Activate Servo Vaccuum control   (Add vaccuum, increase throttle)
      delay(pidOutput);
      digitalWrite(VAC, HIGH);//Deactivate Servo Vaccuum control  (Stop adding vaccuum)
    erasePlus();

  } else if (pidOutput < 0) {

    drawMinus();
      digitalWrite(VENT, HIGH);//Deactivate Servo Vent control   (vent vaccuum, reduce throttle)
      delay(abs(pidOutput));
      digitalWrite(VENT, LOW);//Activate Servo Vent control      (close vent, hold throttle)
    eraseMinus();
  }
}


void loadDefaults(){
    Kp = 2; //See Arduino PID
    Ki = 5; 
    Kd = 1;
    
    sampleNum = 5; //number of samples used to calculate average speed.

    speedStep = 2;   //increment when desired speed is adjusted using +/- buttons.
    speedTimeStep = 250;  //delay between speed info refresh
    maintStep = 400;  //delay between automatic servo adjustments

    servoVentPulse = 100;  //pulse width in ms
    servoVacPulse = 100;  //pulse width in ms
    
    writeEm();
    EEPROM.put(0, 1);
}

