/**
   Turku Ekotuki
   http://www.ekotuki.net/

   Copyright 2015-2017 Kaj-Michael Lang

   License: GPLv2

*/

// ehheh
#define ONE_DIRECTION 1

// Define if BW TFT screen is connected to i2c
#define TFT_128x64 1

#include <Wire.h>
#include <U8glib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>
#include <IRremote.h>

#define BACKLIGHT_PIN     3
//#define DISPLAY_V_A 1
//#define DEBUG_INFO 1

// Do we halt if track voltage drops under minTrackVoltage ? 
#define BROWNOUT_TRACK 1
const float minTrackVoltage=7.0;
const float normTrackVoltage=12.0;

#define IR_PIN 12

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Set the LCD I2C address

Adafruit_INA219 ina219;

IRrecv irrecv(IR_PIN);

#define VERSION_STR "TE-v006"

const char *version=VERSION_STR;

#define SPONSOR_LOGO_LSHJ 1
#define SPONSOR_LOGO_FOLI 1
#define SPONSOR_LOGO_TSP 1
#define SPONSOR_LOGO_TE 1
#define SPONSOR_LOGO_EKOTUKI 1

#ifdef TFT_128x64
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);

#ifdef SPONSOR_LOGO_LSJH
#include "lsjh.h"
#endif

#ifdef SPONSOR_LOGO_FOLI
#include "foli_logo.h"
#endif

#ifdef SPONSOR_LOGO_TSP
#include "tsp_logo.h"
#endif

#ifdef SPONSOR_LOGO_TE
#include "te_logo.h"
#endif

#ifdef SPONSOR_LOGO_EKOTUKI
#include "ekotuki_logo.h"
#endif

#endif

volatile byte cnt1 = 0; // Back counter
volatile byte cnt2 = 0; // Station counter

byte stopCnt = 0;

byte lcdPage = 1;

enum {
  NORMAL,
  BIKE,
  MANUAL,
  DEMO
} world_mode_t;

byte mode=NORMAL;

enum {
  ST_INIT = 0,
  ST_STARTING,
  ST_RUNNING,
  ST_STOPPING,
  ST_STOPATSTATION,
  ST_STATION,
  ST_UNDERVOLTAGE
} train_mode_t;

// Current state
byte state = ST_INIT;
byte pstate = state;

// Status variables
int tspeed = 0; // Target speed
int cspeed = 0; // Current speed
byte aspeed = 1; // Adjust speed
byte ptime = 10; // Pause time, initial startup delay

enum {
  TRAIN_FORWARD,
  TRAIN_BACKWARD,
  TRAIN_BRAKE
} train_direction_t;

int runTime;
int stopTime;

const byte runSpeedMin=180;
const byte runSpeedMax=230;
const byte stopSpeed=130;

float shuntvoltage;
float busvoltage;
float current_mA;
float loadvoltage;

// Lights
byte led1 = 0;
byte led2 = 0;
byte led3 = 0;

// "Travel" direction, set to random value for display purposes
byte travel = 0;
byte traindirection = TRAIN_BRAKE;

// How many trips around the track before stopping at station, max
byte maxRounds = 6;

// Sponsor display helpers
enum {
  SPONS_EKOTUKI=0,
  SPONS_FOLI,
  SPONS_TURKU_ENERGIA,
  SPONS_LSJH,
  SPONS_TSP,
  SPONS_CNT
} sponsors_t;
byte sponsTicker=0;
const int sponsDelay = 2000;
unsigned long scm = 0;

// Update delay
const int updateDelay = 500;
unsigned long ucm = 0;

// Track sensor counters
unsigned long cm = 0;
volatile unsigned long icm = 0; // IRQ delay
const int irqdelay = 2000;

// Default delays (about 1/10 second)
byte stationDelay = 10;
byte startDelay = 10;
byte stopDelay = 10;
byte runningTime = 100;
const byte minDelay=10;

// IRQ handlers for track sensors

// Single track sensor, back country
void trackTick1()
{
  if (cm > (icm + irqdelay)) {
    cnt1++;
    icm = cm;
  }
}

// Double track sensor, station
void trackTick2()
{
  if (cm > (icm + irqdelay)) {
    cnt2++;
    icm = cm;
  }
}

// Setup 16x2 character LCD used for information display (and debug)
void lcd_init()
{
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(BACKLIGHT_ON);
  lcd.begin(16, 2);
  lcd.home();
  lcd.print(version);
}

int readAnalogSetting(int pin, int vmin, int vmax)
{  
  return map(analogRead(pin), 0, 1024, vmin, vmax);  
}

// Read two analog settings from pins A0 and A1
void readSettings()
{  
  runTime = 4*minDelay + readAnalogSetting(A0, 1, 255);  
  stopTime = minDelay + readAnalogSetting(A1, 1, 255);
}

#ifdef TFT_128x64

void drawFoli(void)
{
#ifdef SPONSOR_LOGO_FOLI
u8g.drawXBMP( 0, 0, foli_logo_width, foli_logo_height, foli_logo_bits);
#endif
}

void drawTurkuEnergia(void)
{
#ifdef SPONSOR_LOGO_TE
u8g.drawXBMP( 0, 0, te_logo_width, te_logo_height, te_logo_bits);
#endif
}

void drawLSJH(void)
{
#ifdef SPONSOR_LOGO_LSJH
u8g.drawXBMP( 0, 0, lsjh_width, lsjh_height, lsjh_bits);
#endif
}

void drawTSP(void)
{
#ifdef SPONSOR_LOGO_TSP
u8g.drawXBMP( 0, 0, tsp_logo_width, tsp_logo_height, tsp_logo_bits);
#endif
}

void drawEkotuki(void)
{
#ifdef SPONSOR_LOGO_EKOTUKI
u8g.drawXBMP( 0, 0, ekotuki_logo_width, ekotuki_logo_height, ekotuki_logo_bits);
#endif
}

void drawSponsor(void)
{
if (cm>scm+sponsDelay) {
  sponsTicker++;
  if (sponsTicker>=SPONS_CNT)
    sponsTicker=SPONS_EKOTUKI;
  scm=cm;
} else {
  //return;
}
switch (sponsTicker) {
  case SPONS_EKOTUKI:
    drawEkotuki();
  break;
  case SPONS_FOLI:
    drawFoli();
  break;
  case SPONS_TURKU_ENERGIA:
    drawTurkuEnergia();
  break;
  case SPONS_LSJH:
    drawLSJH();
  break;
  case SPONS_TSP:
    drawTSP();
  break;
}

}

void draw(void)
{  
  switch (state) {
    case ST_INIT:
      drawEkotuki();
      break;
    case ST_RUNNING:
      drawSponsor();
      break;
    case ST_STOPPING:
    case ST_STOPATSTATION:
      u8g.setFont(u8g_font_unifont);
      if (travel == 0) {
        u8g.drawStr(0, 22, "Helsingista");        
        u8g.drawStr(76, 22, "17:48");
      } else {
        u8g.drawStr(0, 22, "Turusta");        
        u8g.drawStr(76, 22, "15:23");
      }      
      break;
    case ST_STARTING:
    case ST_STATION:
      u8g.setFont(u8g_font_unifont);
      if (travel == 0) {
        u8g.drawStr(0, 22, "Helsinki");        
        u8g.drawStr(76, 22, "13:30");
      } else {        
        u8g.drawStr(0, 22, "Turku");
        u8g.drawStr(76, 22, "16:10");        
      }
      break;
    case ST_UNDERVOLTAGE:
      u8g.setFont(u8g_font_unifont);
      u8g.drawStr(20, 22, "!Verkkovika!");
      u8g.drawStr(40, 34, "Polje");
      u8g.drawStr(40, 34, "Lujempaa!");
      break;
  }
}
#endif

void readINA(void)
{
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
}

void errorMsg(const char *msg, int mdelay=500)
{
  lcd.clear();
  lcd.home();
  lcd.print(msg);
  Serial.println(msg);
  delay(mdelay);
}


void setLCDPage(int page)
{
  lcdPage = page;
  lcd.clear();
  updateLCD();
}

void updateLCD()
{
  switch (lcdPage) {
    case 1:
      updateLCDBasePage();
      break;
    case 2:
      updateLCDDebugPage();
      break;
  }
}

void lcdPrintIntAt(byte c, byte r, const int a)
{
  lcd.setCursor(c, r);
  lcd.print(a);
}

void lcdPrintIntAt(byte c, byte r, const float a)
{
  lcd.setCursor(c, r);
  lcd.print(a);
}

void updateLCDDebugPage()
{
#ifdef DEBUG_INFO 
  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);
  
  lcdPrintIntAt(2, 0, maxRounds);
  lcdPrintIntAt(2, 1, mode);
  lcdPrintIntAt(3, 1, sponsTicker);

  lcdPrintIntAt(5, 0, runTime);
  lcdPrintIntAt(5, 1, stopTime);

  lcdPrintIntAt(11, 0, busvoltage);
  lcdPrintIntAt(11, 1, current_mA);
#else
  updateLCDBasePage();
#endif
}

void updateLCDBasePage()
{
  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);

  lcdPrintIntAt(2, 0, state);  
  lcdPrintIntAt(2, 1, stopCnt);
  
  lcdPrintIntAt(3, 0, mode);
  lcdPrintIntAt(3, 1, traindirection);

  lcdPrintIntAt(5, 0, tspeed);
  lcdPrintIntAt(5, 1, cspeed);

  lcdPrintIntAt(9, 0, aspeed);
  lcdPrintIntAt(9, 1, ptime);

  lcdPrintIntAt(13, 0, busvoltage);
  lcdPrintIntAt(13, 1, current_mA);

#ifdef DISPLAY_V_A_XXX
  lcd.setCursor(13, 0);
  if (busvoltage > 9.0)
    lcd.print((int)busvoltage);
  else
    lcd.print(busvoltage);
  lcd.setCursor(13, 1);
  if (current_mA > 100.0)
    lcd.print((int)current_mA);
  else
    lcd.print(current_mA);
#endif    

}

void speedAdjust()
{
  if (tspeed > cspeed && cspeed < (255-aspeed))
    cspeed += aspeed;
  else if (tspeed < cspeed && cspeed > aspeed && cspeed>0) {
    cspeed -= aspeed;
  } else if (tspeed==0 && cspeed>0)
    cspeed=0;
}

void setNextState(int s, int p, int a)
{
  state = s;
  ptime = p;
  aspeed = a;

  Serial.println("*");
  Serial.println(s);
  Serial.println(p);
  Serial.println(a);
}

void setLights()
{
  analogWrite(6, led1);
  analogWrite(9, led2);
  analogWrite(10, led3);
}

void dump()
{
#ifdef DEBUG_INFO
  Serial.print("BV:"); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("SV:"); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("LV:"); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("CU:"); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("-");
  Serial.print("CS:");
  Serial.println(cspeed);
  Serial.print("TS:");
  Serial.println(cspeed);
  Serial.print("PT:");
  Serial.println(ptime);
  Serial.print("ST:");
  Serial.println(state);
  Serial.println("-");
#endif
}

void loopDemoMode()
{
readSettings();
setLCDPage(2);
state = ST_RUNNING;
delay(500);
}

void setTrainDirection(byte d)
{
switch (d) {
  case TRAIN_FORWARD:
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
  break;
  case TRAIN_BACKWARD:
    digitalWrite(4, LOW);
    digitalWrite(7, HIGH);
  break;
  case TRAIN_BRAKE:
  default: ;
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
  }  
}

void adjustTargetSpeed(int t)
{
  tspeed = t - ((busvoltage-normTrackVoltage)*2) + (cnt1+cnt2);
  if (tspeed>runSpeedMax)
    tspeed=runSpeedMax;
}

void loopNormalMode()
{
    switch (state) {
    case ST_INIT: //0
      led1 += 2;
      led2 += 2;
      led3 += 2;
      if (ptime == 0) {
        state = ST_STARTING;
        ptime = 20;
        aspeed = 6;
        cnt1 = 0;
        cnt2 = 0;
        stopCnt = 0;
        led1 = 64;
        led2 = 64;
        led3 = 64;
        traindirection=TRAIN_FORWARD;
      }
      setLCDPage(1);
      break;
    case ST_STARTING: //1
      adjustTargetSpeed(runSpeedMax);
      if (ptime == 0) {
        setNextState(ST_RUNNING, 120, 4);
      }
      break;
    case ST_RUNNING: //2
      adjustTargetSpeed(runSpeedMin);
      if (ptime == 0 || cnt1 > maxRounds) {
        setNextState(ST_STOPPING, 30, random(2)+1);
        travel = random(100) > 50 ? 0 : 1;
      }
      break;
    case ST_STOPPING: //3
      adjustTargetSpeed(stopSpeed);      
      if (ptime == 0) {
        setNextState(ST_STOPATSTATION, 5+random(10), 4);
        stopCnt = cnt2;
        if (random(10)>5)
          led2=0;
        if (random(10)>5)
          led3=0;
        if (random(10)>5)
          led3=0;
      }
      break;
    case ST_STOPATSTATION: //4
      if (stopCnt < cnt2 && ptime==0) {
        setNextState(ST_STATION, stopTime+random(20), 10);
        tspeed=0;
      }
    break;
    case ST_STATION:
      //tspeed = 0;
      //cspeed = 0;
      traindirection=TRAIN_BRAKE;
      if (ptime == 0) {
        setNextState(ST_STARTING, stopTime+random(20), 6);
        stopCnt = 0;
        cnt1 = 0;
        cnt2 = 0;
        
        if (led2==0)
          led2=64;
        if (led3==0)
          led3=64;
        if (led3==0)
          led3=64;
        if (travel==0)
          traindirection=TRAIN_FORWARD;
        else
          traindirection=TRAIN_BACKWARD;
          
      }
      break;
    case ST_UNDERVOLTAGE:
      if (busvoltage > minTrackVoltage) { // Go to initial state if we get enough power!
        state = ST_INIT;
        mode = NORMAL;
        ptime = 2;
        errorMsg("VOLTAGE UP");
      } else if (busvoltage<4.9) { // Assume USB powered development/demo mode if voltage is under 4.9
        mode = DEMO;
        state = ST_RUNNING;
        readSettings();         
        setLCDPage(2);
      } else { // Else 
        tspeed = 0;
        cspeed = 0;
        ptime = 0;
        stopCnt = 0;
        led1 = 0;
        led2 = 0;
        led3 = 0;
        // Allow adjusting settings when low-voltage + display of them
        readSettings();
        setLCDPage(2);
      }
      break;
    default:
      state = ST_INIT;
      Serial.println("?");
      break;
  }
}

void loopManualMode()
{
  
}

void updateTFT()
{
#ifdef TFT_128x64
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage());
#endif 
}

void trainUpdate()
{  
  // Train speed
  speedAdjust();    
  setTrainDirection(traindirection);
  analogWrite(5, cspeed);  
}

void readIR()
{
  decode_results results;
  static unsigned long pcode=0;

  if (!irrecv.decode(&results))
    return;

  irrecv.resume();

  /*
  if (results.value==0xFFFFFFFF && pcode>0)
    results.value=pcode;
  else
    pcode=results.value;
  */

  switch (results.value) {
    // 1-3
    case 0x011: // 1
      led1=led1>0 ? 0 : 64;
      ucm=0;
    break;
    case 0x811: // 2
      led2=led2>0 ? 0 : 64;
      ucm=0;
    break;
    case 0x411: // 3
      led3=led3>0 ? 0 : 64;
      ucm=0;
    break;
    case 0xC11: // 4
    case 0x211: // 5
    case 0xA11: // 6
    break;
    // Controls
    case 0x4D1: // Play
      if (tspeed<40)
        tspeed=runSpeedMin;
      aspeed=6;
      if (traindirection==TRAIN_BRAKE)
        traindirection=TRAIN_FORWARD;
      state=ST_RUNNING;
    break;
    case 0x9D1: // Pause
      tspeed=0;
      aspeed=6;
      state=ST_STOPPING;
    break;
    case 0x2D1: // FF
      if (tspeed<runSpeedMax)
        tspeed+=2;
    break;
    case 0xCD1: // RW
      if (tspeed>2)
        tspeed-=2;
      else
        tspeed=0;
    break;
    case 0x0D1: // Prev
      if (cspeed==0 && (traindirection==TRAIN_FORWARD || traindirection==TRAIN_BRAKE)) {      
        if (traindirection==TRAIN_FORWARD) {
          tspeed=runSpeedMin;
          state=ST_RUNNING;
        }
        traindirection=TRAIN_BACKWARD;
      } else {
        tspeed=0;
        aspeed=8;
        state=ST_STOPPING;
      }
    break;
    case 0x8D1: // Next
      if (cspeed==0 && (traindirection==TRAIN_BACKWARD || traindirection==TRAIN_BRAKE)) {
          if (traindirection==TRAIN_BACKWARD) {
            tspeed=runSpeedMin;
            state=ST_RUNNING;
          }
        traindirection=TRAIN_FORWARD;        
      } else {
        tspeed=0;
        aspeed=8;
        state=ST_STOPPING;
      }
    break;
    case 0x1D1: // Stop
      if (mode==NORMAL) {        
        mode=MANUAL;
      }

      lcd.clear();
      cspeed=tspeed=0;
      aspeed=14;
      traindirection=TRAIN_BRAKE;
      cnt1=cnt2=stopCnt=0;
      led1=led2=led3=0;
      state=ST_INIT;
    break;
    default:
      Serial.println("?");  
  }
  Serial.println(results.value, HEX);  
}

void setup()
{
  // setup PWM output pins
  // Pins 5,6 and 10 are connect to a motor controller, we abuse it a bit
  pinMode(5, OUTPUT); // Train speed
  pinMode(6, OUTPUT);  // Abused for LED
  pinMode(10, OUTPUT); // Abused for LED

  // Make sure they are not active
  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(10, 0);

  //
  pinMode(9, OUTPUT);
  analogWrite(9, 0);

  // Motor controller direction control
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(7, LOW);

  Serial.begin(115200);
  Serial.println(version);

  irrecv.enableIRIn();  

  cm = millis();

  lcd_init();

  ina219.begin();

#ifdef TFT_128x64  
  u8g.setColorIndex(1);
#endif

  readSettings();

  readINA();

  delay(500);

  // Train track IR sensors are connected to pins 2,3
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), trackTick2, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trackTick1, FALLING);

  lcd.clear();

  setLights();  
}

void loop()
{
  cm = millis();

  readINA();

  readIR();

  if (ptime>0)
    ptime--;
    
  pstate = state;

  dump();

  switch (mode) {
    case NORMAL:
    case BIKE:
      loopNormalMode();
#ifdef BROWNOUT_TRACK
      if (busvoltage < minTrackVoltage && mode==NORMAL) {
        state = ST_UNDERVOLTAGE;
        tspeed = 0;
        cspeed = 0;
        ptime = 0;        
        errorMsg("LOW VOLTAGE", 2000);        
      }
#endif      
    break;
    case DEMO:
      if (busvoltage > 5.0)
        mode=NORMAL;
      else
        loopDemoMode();
    break;
    case MANUAL:
      loopManualMode();
    break;
    default:
      mode=NORMAL;
  }  

  if (pstate != state) {
    lcd.clear();
  }
  
  updateLCD();

  trainUpdate();
  
  // Do things that are not needed on every loop
  if (cm>ucm+updateDelay) {
    updateTFT();
    setLights();
    ucm=cm;
  }

  unsigned long cm_now = millis();

  Serial.println(cm_now-cm);

  delay(100);
}
