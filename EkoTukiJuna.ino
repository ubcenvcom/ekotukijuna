/**
   Turku Ekotuki
   http://www.ekotuki.net/

   Copyright 2015 Kaj-Michael Lang

   License: GPLv2

*/

#define SD_ChipSelectPin 4

#include <SPI.h>
#include <Wire.h>
#include <U8glib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>

#ifdef SOUND
#include <pcmRF.h>
#include <TMRpcm.h>
#include <pcmConfig.h>
#endif

#define BACKLIGHT_PIN     3

#define DEBUG 1

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Set the LCD I2C address

Adafruit_INA219 ina219;

#ifdef TFT_128x64
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);
#endif

#ifdef SOUND
TMRpcm pcm;
#endif

volatile int cnt1 = 0; // Back counter
volatile int cnt2 = 0; // Station counter

int stopCnt = 0;

byte lcdPage = 1;

bool hasSD = false;

enum {
  ST_INIT = 0,
  ST_STARTING,
  ST_RUNNING,
  ST_STOPPING,
  ST_STOPATSTATION,
  ST_STATION,
  ST_UNDERVOLTAGE
};

// Current state
byte state = ST_INIT;
byte pstate = state;

// Status variables
int tspeed = 0; // Target speed
int cspeed = 0; // Current speed
byte aspeed = 1; // Adjust speed
byte ptime = 10; // Pause time

int runTime;
int stopTime;

const byte runSpeedMin=150;
const byte stopSpeed=70;

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

// How many trips around the track before stopping at station
byte maxRounds = 8;

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
  lcd.print("Turku Ekotuki");
  lcd.setCursor(0, 1);
  lcd.print("J-001");
}

int readAnalogSetting(int pin, int vmin, int vmax)
{
  int v = analogRead(pin);
  v = map(v, 0, 1024, vmin, vmax);
  return v;
}

// Read two analog settings from pins A0 and A1
void readSettings()
{
  int r;
  r = readAnalogSetting(A0, 1, 255);
  runTime = 4*minDelay + r;
  
  r = readAnalogSetting(A1, 1, 255);
  stopTime = minDelay + r;
}

#ifdef TFT_128x64
void draw(void)
{
  u8g.setFont(u8g_font_unifont);
  switch (state) {
    case ST_STARTING:
      u8g.drawStr(0, 22, "Ekotuki Turku");
      //u8g.drawStr(0, 34, " Turku!");
      break;
    case ST_RUNNING:
      u8g.drawStr(0, 22, "Tunnin Juna");
      //u8g.drawStr(0, 34, " Juna!");
      break;
    case ST_STOPPING:
      u8g.drawStr(0, 22, "Folilla");
      u8g.drawStr(0, 34, "Kotiin!");
      u8g.drawStr(0, 42, "www.foli.fi");
      break;
    case ST_STATION:
      if (travel == 0) {
        u8g.drawStr(0, 22, "Turku");
        u8g.drawStr(0, 34, "Helsinki");
      } else {
        u8g.drawStr(0, 22, "Helsinki");
        u8g.drawStr(0, 34, "Turku");
      }
      break;
    case ST_UNDERVOLTAGE:
      u8g.drawStr(0, 22, "Sahkoverkkossa");
      u8g.drawStr(30, 34, "vikaa");
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

void errorMsg(const char *msg)
{
  lcd.clear();
  lcd.home();
  lcd.print(msg);
  Serial.println(msg);
  delay(100);
}

void setup()
{
  // setup PWM output pins
  // Pins 5,6 and 10 are connect to a motor controller, we abuse it a bit
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(10, OUTPUT);

  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(10, 0);

  Serial.begin(115200);

  lcd_init();

  ina219.begin();

#ifdef TFT_128x64  
  u8g.setColorIndex(1);
#endif

  readSettings();

  readINA();
#ifdef SOUND
  pcm.speakerPin = 9;

  if (!SD.begin(SD_ChipSelectPin)) {
    errorMsg("SD!");
    hasSD = false;
  } else {
    hasSD = true;
  }
#endif

  delay(500);

  // Train track IR sensors are connected to pins 2,3
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), trackTick2, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trackTick1, FALLING);

  lcd.clear();

  setLights();
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
  lcdPrintIntAt(5, 0, runTime);
  lcdPrintIntAt(5, 1, stopTime);

  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);

  lcdPrintIntAt(2, 0, maxRounds);

  lcdPrintIntAt(11, 0, busvoltage);
  lcdPrintIntAt(11, 1, current_mA);
}

void updateLCDBasePage()
{
  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);

#ifdef DISPLAY_V_A
  lcd.setCursor(9, 0);
  if (busvoltage > 9.0)
    lcd.print((int)busvoltage);
  else
    lcd.print(busvoltage);
  lcd.setCursor(9, 1);
  if (current_mA > 100.0)
    lcd.print((int)current_mA);
  else
    lcd.print(current_mA);
#endif    

  lcdPrintIntAt(3, 0, tspeed);
  lcdPrintIntAt(3, 1, cspeed);

  lcdPrintIntAt(8, 0, aspeed);
  lcdPrintIntAt(8, 1, ptime);
  
  lcdPrintIntAt(12, 0, state);
  lcdPrintIntAt(14, 0, (int)loadvoltage);
  lcdPrintIntAt(12, 1, stopCnt);
}

void speedAdjust()
{
  if (tspeed > cspeed && cspeed < (255-aspeed))
    cspeed += aspeed;
  else if (tspeed < cspeed && cspeed > aspeed && cspeed>0) {
    cspeed -= aspeed;
  }
}

void setNextState(int s, int p, int a)
{
  state = s;
  ptime = p;
  aspeed = a;

  Serial.println("***");
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
#ifdef DEBUG
  Serial.print("BV:"); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("SV:"); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("LV:"); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("CU:"); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("---");
  Serial.print("CS:");
  Serial.println(cspeed);
  Serial.print("TS:");
  Serial.println(cspeed);
  Serial.print("PT:");
  Serial.println(ptime);
  Serial.print("ST:");
  Serial.println(state);
  Serial.println("---");
#endif
}

void loop()
{
  cm = millis();


#ifdef TFT_128x64
  u8g.firstPage();
  do {
    draw();
  } while ( u8g.nextPage());
#endif

  readINA();

  if (ptime>0)
    ptime--;
    
  pstate = state;

  dump();

  switch (state) {
    case ST_INIT: //0
      led1 = 64;
      led2 = 64;
      led3 = 64;
      if (ptime == 0) {
        state = ST_STARTING;
        ptime = 20;
        aspeed = 4;
        cnt1 = 0;
        cnt2 = 0;
        stopCnt = 0;
      }
      setLCDPage(1);
      break;
    case ST_STARTING: //1
      tspeed = 160;
      if (ptime == 0) {
        setNextState(ST_RUNNING, 120, 2);
      }
      break;
    case ST_RUNNING: //2
      tspeed = runSpeedMin - (busvoltage * 2.0) + (cnt1+cnt2);
      if (tspeed>160)
        tspeed=160;
      if (ptime == 0 || cnt1 > maxRounds) {
        setNextState(ST_STOPPING, 30, random(2)+1);
        travel = random(100) > 50 ? 0 : 1;
      }
      break;
    case ST_STOPPING: //3
      tspeed = stopSpeed;      
      if (ptime == 0) {
        setNextState(ST_STOPATSTATION, 5+random(10), 4);
        stopCnt = cnt2;
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
      if (ptime == 0) {
        setNextState(ST_STARTING, stopTime+random(20), 2+random(6));
        stopCnt = 0;
        cnt1 = 0;
        cnt2 = 0;
      }
      break;
    case ST_UNDERVOLTAGE:
      if (busvoltage > 8.0) {
        state = ST_INIT;
        ptime = 2;
      } else {
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

  if (busvoltage < 8.0) {
    state = ST_UNDERVOLTAGE;
    tspeed = 0;
    cspeed = 0;
    ptime = 0;
  }

  speedAdjust();

  if (pstate != state) {
    lcd.clear();
  }
  updateLCD();

  // Train speed
  analogWrite(5, cspeed);
  setLights();

  delay(100);
}

