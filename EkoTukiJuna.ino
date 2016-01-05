/**
 * Turku Ekotuki
 * http://www.ekotuki.net/
 * 
 * Copyright 2015 Kaj-Michael Lang
 * 
 * License: GPLv2
 * 
 */

#define SD_ChipSelectPin 4

#include <SPI.h>
#include <Wire.h> 
#include <U8glib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>

#include <pcmRF.h>
#include <TMRpcm.h>
#include <pcmConfig.h>

#define BACKLIGHT_PIN     3

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // Set the LCD I2C address
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);
Adafruit_INA219 ina219;
TMRpcm pcm;

volatile int cnt1=0;
volatile int cnt2=0;

byte lcdPage=1;

bool hasSD=false;

enum {
  ST_INIT,
  ST_STARTING,
  ST_RUNNING,
  ST_STOPPING,
  ST_STATION,
  ST_UNDERVOLTAGE
};

// Current state
byte state=ST_INIT;
byte pstate=state;

// Status variables
byte tspeed=0; // Target speed
byte cspeed=0; // Current speed
byte aspeed=1; // Adjust speed
byte ptime=10; // Pause time

float shuntvoltage;
float busvoltage;
float current_mA;
float loadvoltage;

// Lights
byte led1=0;
byte led2=0;
byte led3=0;

byte travel=0;

// Track sensor counters
unsigned long cm=0;
volatile unsigned long icm=0; // IRQ delay
int irqdelay=1000;

// Default delays
byte stationDelay=10;
byte startDelay=10;
byte stopDelay=10;
byte runningTime=100;
byte r1, r2;

// Single track sensor, back country
void trackTick1()
{
  if (cm>(icm+irqdelay)) {
    cnt1++;
    icm=cm;
  }
}

// Double track sensor, station
void trackTick2()
{
  if (cm>(icm+irqdelay)) {
    cnt2++;
    icm=cm;
  }
}

// Setup 16x2 character LCD used for information display (and debug)
void lcd_init()
{
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(BACKLIGHT_ON);
  lcd.begin(16,2);
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
// Exact usage is TBD
void readSettings()
{
  r1=readAnalogSetting(A0, 0, 255);
  r2=readAnalogSetting(A1, 0, 255);
}

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
      if (travel==0) {
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

void readINA(void) 
{
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
#ifdef DEBUG
  Serial.print("BV:"); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("SV:"); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("LV:"); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("CU:"); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
#endif
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

  pcm.speakerPin = 9;

  Serial.begin(115200);  

  lcd_init();
  
  ina219.begin();
  u8g.setColorIndex(1);

  readINA();
  readSettings();

  if (!SD.begin(SD_ChipSelectPin)) {
    errorMsg("SD!");
    hasSD=false;
  } else {
    hasSD=true;
  }

  delay(1000);

  // Train track IR sensors are connected to pins 2,3
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), trackTick1, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trackTick2, FALLING);

  lcd.clear();

  setLights();
}

void setLCDPage(int page)
{
  lcdPage=page;
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

void lcdPrintIntAt(byte c,byte r, const int a)
{
  lcd.setCursor(c, r);
  lcd.print(a);
}

void updateLCDDebugPage()
{
  lcdPrintIntAt(4, 0, r1);
  lcdPrintIntAt(4, 1, r2);
  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);
}

void updateLCDBasePage()
{
  lcd.setCursor(3, 0);
  if (busvoltage>9.0)
    lcd.print((int)busvoltage); 
  else
    lcd.print(busvoltage); 
  lcd.setCursor(3, 1);
  if (current_mA>100.0)
    lcd.print((int)current_mA);
  else
    lcd.print(current_mA);

  lcdPrintIntAt(9, 0, tspeed);
  lcdPrintIntAt(9, 1, cspeed); 

  lcdPrintIntAt(13, 0, aspeed);
  lcdPrintIntAt(15, 0, state);
  lcdPrintIntAt(13, 1, ptime);

  lcdPrintIntAt(0, 0, cnt1);
  lcdPrintIntAt(0, 1, cnt2);
}

void speedAdjust()
{
  if (tspeed>cspeed && cspeed<255)
    cspeed+=aspeed;
  else if (tspeed<cspeed && cspeed>0) {
    cspeed-=aspeed;
  }
  Serial.print("CS:");
  Serial.println(cspeed);
}

void setNextState(int s, int p, int a)
{
  state=s;
  ptime=p;
  aspeed=a;
}

void setLights()
{
  analogWrite(6, led1);
  analogWrite(9, led2);
  analogWrite(10, led3);
}

void loop()
{
  cm=millis();
  
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage());

  Serial.print("PT:");
  ptime--;
  if (ptime<0)
    ptime=0;
  Serial.println(ptime);

  pstate=state;

  Serial.println(state);
  switch (state) {
    case ST_INIT:
      led1=64;
      led2=64;
      led3=64;
      if (ptime==0) {
        state=ST_STARTING;
        ptime=20;
        aspeed=4;
      }
    break;
    case ST_STARTING:
      tspeed=160;
      if (ptime==0) {
        setNextState(ST_RUNNING, 120, 2);
      }
    break;
    case ST_RUNNING:
      tspeed=130;
      if (ptime==0) {
        setNextState(ST_STOPPING, 20, 2);
        travel=random(100)>50 ? 0 : 1;
      }
    break;
    case ST_STOPPING:
      tspeed=0;
      cspeed/=2;
      if (ptime==0) {
        state=ST_STATION;
        ptime=40;
      }
    break;
    case ST_STATION:
      if (ptime==0) {
        state=ST_STARTING;
        aspeed=6;
        ptime=20;
      }
    break;
    case ST_UNDERVOLTAGE:
      if (busvoltage>8.0) {
        state=ST_INIT;
        readSettings();
      } else {
        tspeed=0;
        cspeed=0;
        ptime=0;
        led1=0;
        led2=0;
        led3=0;
      }
    break;
    default:
      state=ST_INIT;
      Serial.println("?");
    break;
  }
  
  speedAdjust();
  readINA();

  if (busvoltage<8.0) {
    state=ST_UNDERVOLTAGE;
    tspeed=0;
    cspeed=0;
    ptime=0;
  }
  
  if (pstate!=state) {
    lcd.clear();
  }
  updateLCD();

  // Train speed
  analogWrite(5, cspeed);
  setLights();
  
  delay(100);  
}

