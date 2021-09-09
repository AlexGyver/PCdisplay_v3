/*
  Скетч к проекту "Контроллер СВО"
  Страница проекта (схемы, описания): https://alexgyver.ru/PCdisplay_v3/
  Исходники на GitHub: https://github.com/AlexGyver/PCdisplay_v3
  Нравится, как написан код? Поддержи автора! https://alexgyver.ru/support_alex/
  Автор: AlexGyver Technologies, 2019
  https://AlexGyver.ru/
*/

// пины
#define POT_INC 5
#define POT_UD 4
#define POT_CS 6
#define THERM_PIN A0
#define RELAY_FAN 12
#define PWM_FAN 9
#define PWM_FAN_2 10

// настройки
#define CPU_SOURCE 19       // источник температуры ЦП (0 или 19 для libre HM)
#define TIMEOUT 5000        // таймаут соединения, мс
#define TIMEOUT_P 300000    // таймаут питания, мс
#define RESIST_10K 10000    // точное сопротивление 10к резистора (Ом)

// пределы
#define PUMP_MIN 10     // мин. сигнал помпы (при TEMP_MIN)
#define PUMP_MAX 95     // макс. сигнал помпы (при TEMP_MAX)

#define PWM_MIN 200     // мин. сигнал вентиляторов (при TEMP_MIN)
#define PWM_MAX 999     // макс. сигнал вентиляторов (при TEMP_MAX)

// онлайн
#define TEMP_ON 42      // температура воды, выше которой включается вентилятор СВО
#define TEMP_OFF 38     // температура воды, ниже которой выключается вентилятор СВО

#define HW_TEMP_MIN 50  // мин. температура железа
#define HW_TEMP_MAX 70  // макс. температура железа

// оффлайн
//#define TEMP_MIN 35   // мин. температура воды (оффлайн)
//#define TEMP_MAX 47   // макс. температура воды (оффлайн)

#define COEF 0.05        // коэффициент плавности изменения температуры

// ---- термистор ----
// GND --- термистор --- A0 --- 10к --- 5V
#define RESIST_BASE 10000   // сопротивление при TEMP_BASE градусах по Цельсию (Ом), из даташита
#define TEMP_BASE 25        // температура, при которой измерено RESIST_BASE (градусов Цельсия)
#define B_COEF 3435         // бета коэффициент термистора (3000-4000)
// ---- термистор ----

// --- библиотеки ---
#include <DigiPotX9Cxxx.h>
DigiPot pot(POT_INC, POT_UD, POT_CS);

#include "thermistorMinim.h"
thermistor thermWater(THERM_PIN, RESIST_BASE, B_COEF, TEMP_BASE, RESIST_10K);
thermistor thermPS(A1, 10000, 3950, 25, 10000);

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 16, 2);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

//#include <PWM.h>

int pumpSpeed;
int waterTemp;
int PStemp;

boolean onlineFlag;
boolean powerFlag;
uint32_t timeoutTimer, drawTimer, fireTimer;

char inData[82];       // массив входных значений (СИМВОЛЫ)
int PCdata[20];        // массив численных значений показаний с компьютера
byte index = 0;
String string_convert;
float maxTemp;
boolean pinState;

byte logo0[8] = {0b00011, 0b00110,  0b01110,  0b11111,  0b11011,  0b11001,  0b00000,  0b00000};
byte logo1[8] = {0b10000, 0b00001,  0b00001,  0b00001,  0b00000,  0b10001,  0b11011,  0b11111};
byte logo2[8] = {0b11100, 0b11000,  0b10001,  0b11011,  0b11111,  0b11100,  0b00000,  0b00000};
byte logo3[8] = {0b00000, 0b00001,  0b00011,  0b00111,  0b01101,  0b00111,  0b00010,  0b00000};
byte logo4[8] = {0b11111, 0b11111,  0b11011,  0b10001,  0b00000,  0b00000,  0b00000,  0b00000};
byte logo5[8] = {0b00000, 0b10000,  0b11000,  0b11100,  0b11110,  0b11100,  0b01000,  0b00000};

byte CP_char[] = {B01000, B10100, B10100, B11111, B00000, B10001, B10001, B01110};
byte GP_char[] = {B01000, B10100, B11111, B00000, B00110, B10101, B10001, B01110};
byte RAM_char[] = {B11111, B11110, B11111, B11110, B11111, B11110, B11111, B00000};
byte drop_char[] = {B00100, B01110, B01110, B11111, B11111, B11111, B01110, B00000};

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);

  pinMode(RELAY_FAN, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);
  pinMode(PWM_FAN_2, OUTPUT);

  digitalWrite(RELAY_FAN, HIGH);
  PWM_20KHZ_D9(50);
  PWM_20KHZ_D10(50);

  pumpSpeed = PUMP_MIN;
  pot.set(PUMP_MIN);

  drawTimer = millis() - 1000;

  lcd.init();
  lcd.clear();
  lcd.backlight();
  show_logo();
  lcd.clear();
  lcd.createChar(0, CP_char);
  lcd.createChar(1, GP_char);
  lcd.createChar(2, RAM_char);
  lcd.createChar(3, drop_char);
  printData();
  maxTemp = 50;
}

void loop() {
  parsing();
  calcAndDisp();
  setPotSmooth();
}

void calcAndDisp() {
  if (millis() - drawTimer > 1000) {
    drawTimer = millis();

    if (onlineFlag && millis() - timeoutTimer > TIMEOUT) {
      onlineFlag = false;
    }
    if (powerFlag && millis() - timeoutTimer > TIMEOUT_P) {
      powerFlag = false;
      lcd.noBacklight();
    }

    waterTemp = thermWater.getTempAverage();

    // онлайн
    if (onlineFlag) {
      maxTemp += (float)(max(PCdata[CPU_SOURCE], PCdata[1]) - maxTemp) * COEF;
      int pwmSpeed = map(maxTemp, HW_TEMP_MIN, HW_TEMP_MAX, PWM_MIN, PWM_MAX);
      pwmSpeed = constrain(pwmSpeed, PWM_MIN, PWM_MAX);
      PWM_20KHZ_D9(pwmSpeed);
      pumpSpeed = map(maxTemp, HW_TEMP_MIN, HW_TEMP_MAX, PUMP_MIN, PUMP_MAX);

      if (waterTemp > TEMP_ON) digitalWrite(RELAY_FAN, 1);
      else if (waterTemp < TEMP_OFF) digitalWrite(RELAY_FAN, 0);

      // оффлайн - вент всегда включен
    } else {
      digitalWrite(RELAY_FAN, 1);
      PWM_20KHZ_D9(PWM_MAX);
      if (waterTemp < 30) pumpSpeed = 20;
      else pumpSpeed = 70;
      //pumpSpeed = map(waterTemp, TEMP_MIN, TEMP_MAX, PUMP_MIN, PUMP_MAX);
    }
    pumpSpeed = constrain(pumpSpeed, PUMP_MIN, PUMP_MAX);
    //if (pumpSpeed > 20 && pumpSpeed < 35) pumpSpeed = 20;

    pinState = !pinState;
    digitalWrite(13, pinState);

    // вентилятор бп
    PStemp = thermPS.getTempAverage();
    int PSpwm = map(PStemp, 30, 45, 100, 800);
    PSpwm = constrain(PSpwm, 100, 800);
    PWM_20KHZ_D10(PSpwm);

    printData();
  }
}

int potVal = 0;
void setPotSmooth() {
  static uint32_t potTmr;
  if (millis() - potTmr > 100) {
    potTmr = millis();
    if (potVal != pumpSpeed) potVal += (potVal < pumpSpeed) ? 1 : -1;
    pot.set(potVal);
  }
}

void PWM_20KHZ_D9(int duty) {
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011001;
  ICR1H = 3;    // highByte(799)
  ICR1L = 31;   // lowByte(799)
  duty = map(duty, 0, 1023, 0, 799);
  OCR1AH = highByte(duty);
  OCR1AL = lowByte(duty);
}
void PWM_20KHZ_D10(int duty) {
  TCCR1A = 0b10100010;
  TCCR1B = 0b00011001;
  ICR1H = 3;    // highByte(799)
  ICR1L = 31;   // lowByte(799)
  duty = map(duty, 0, 1023, 0, 799);
  OCR1BH = highByte(duty);
  OCR1BL = lowByte(duty);
}
