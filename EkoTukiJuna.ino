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
  ST_STATION
};

int state=ST_INIT;
int pstate=state;

int tspeed=0; // Target speed
int cspeed=0; // Current speed
int aspeed=1; // Adjust speed

int ptime=10; // Pause time

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;

unsigned long cm=0;
volatile unsigned long i1cm=0;
volatile unsigned long i2cm=0;
int irqdelay=1000;

void trackTick1()
{
  if (cm>(i1cm+irqdelay)) {
    cnt1++;
    i1cm=cm;
  }
}

void trackTick2()
{
  if (cm>(i2cm+irqdelay)) {
    cnt2++;
    i2cm=cm;
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

void draw(void) {  
  u8g.setFont(u8g_font_unifont);
  switch (state) {
    case ST_STARTING:
      u8g.drawStr(0, 22, "Ekotuki");
      u8g.drawStr(0, 34, " Turku!");
    break;
    case ST_RUNNING:
      u8g.drawStr(0, 22, "Tunnin");
      u8g.drawStr(0, 34, " Juna!");
    break;
    case ST_STOPPING:
      u8g.drawStr(0, 22, "Fölillä");
      u8g.drawStr(0, 34, "Kotiin!");
    break;
  }
}

void readINA(void) 
{
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("BV:"); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("SV:"); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("LV:"); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("CU:"); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
}

void setup()
{
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  analogWrite(5, 0);
  analogWrite(6, 0);

  Serial.begin(115200);
  Serial.println("JV001");

  lcd_init();

  ina219.begin();
  u8g.setColorIndex(1);

  readINA();

  delay(1000);
  
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), trackTick1, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), trackTick2, FALLING);

  lcd.clear();
}

void updateLCD()
{
  lcd.setCursor(3, 0);
  lcd.print(busvoltage); 
  lcd.setCursor(3, 1);
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
    default:
      state=ST_INIT;
      Serial.println("?");
    break;
  }

  if (pstate!=state) {
    lcd.clear();
  }
  
  speedAdjust();
  readINA();
  updateLCD();

  if (busvoltage<7.0) {
    // Undervoltage, do nothing
    analogWrite(5, 0);
    analogWrite(6, 0);    
  } else {
    analogWrite(5, cspeed);
    analogWrite(6, cspeed);
  }
  delay(100);  
}

