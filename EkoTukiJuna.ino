/**
 * Turku Ekotuki
 * http://www.ekotuki.net/
 * 
 * Copyright 2015 Kaj-Michael Lang
 * 
 * License: GPLv2
 * 
 */

#include <Wire.h> 
#include <U8glib.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_INA219.h>

#define BACKLIGHT_PIN     3

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);  // Set the LCD I2C address
U8GLIB_SH1106_128X64 u8g(U8G_I2C_OPT_NONE);
Adafruit_INA219 ina219;

volatile int cnt1=0;
volatile int cnt2=0;

enum {
  ST_INIT,
  ST_STARTING,
  ST_RUNNING,
  ST_STOPPING,
  ST_STATION,
  ST_UNDERVOLTAGE
};

// Current state
int state=ST_INIT;
int pstate=state;

// Status variables
int tspeed=0; // Target speed
int cspeed=0; // Current speed
int aspeed=1; // Adjust speed
int ptime=10; // Pause time

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;

// Lights
int led1=0;
int led2=0;
int led3=0;

int travel=0;

// Track sensor counters
unsigned long cm=0;
volatile unsigned long icm=0; // IRQ delay
int irqdelay=1000;

// Default delays
int stationDelay=10;
int startDelay=10;
int stopDelay=10;
int runningTime=100;

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

void lcd_init()
{
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(BACKLIGHT_ON);
  lcd.begin(16,2);
  lcd.home ();
  lcd.print("Turku Ekotuki");
  lcd.setCursor (0, 1);
  lcd.print ("J-001");    
}

int readAnalogSetting(int pin, int vmin, int vmax) {
  int v = analogRead(pin);
  v = map(v, 0, 1024, vmin, vmax);
  return v;
}

void readSettings() {
  int tmp=readAnalogSetting(A0, 10, 100);
  Serial.print("RAS1: ");
  Serial.println(tmp);
}

void draw(void) {  
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
  /*
  Serial.print("BV:"); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("SV:"); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("LV:"); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("CU:"); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  */
}

void setup()
{
  // PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  analogWrite(5, 0);
  analogWrite(6, 0);
  analogWrite(9, 0);
  analogWrite(10, 0);

  Serial.begin(115200);
  Serial.println("JV001");

  lcd_init();

  ina219.begin();
  u8g.setColorIndex(1);

  readINA();
  readSettings();

  delay(1000);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), trackTick1, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trackTick2, FALLING);

  lcd.clear();

  setLights();
}

void updateLCD()
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

  lcd.setCursor(9, 0);
  lcd.print(tspeed); 
  lcd.setCursor(9, 1);
  lcd.print(cspeed); 
  
  lcd.setCursor(13, 0);
  lcd.print(aspeed);
  lcd.setCursor(15, 0);
  lcd.print(state);
  lcd.setCursor(13, 1);
  lcd.print(ptime); 

  lcd.setCursor(0, 0);
  lcd.print(cnt1); 
  lcd.setCursor(0, 1);
  lcd.print(cnt2); 
}

void speedAdjust() {
  if (tspeed>cspeed && cspeed<255)
    cspeed+=aspeed;
  else if (tspeed<cspeed && cspeed>0) {
    cspeed-=aspeed;
  }
  Serial.print("CS:");
  Serial.println(cspeed);
}

void setNextState(int s, int p, int a) {
  state=s;
  ptime=p;
  aspeed=a;
}

void setLights() {
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

