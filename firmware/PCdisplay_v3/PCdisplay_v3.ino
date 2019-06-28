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

// настройки
#define TIMEOUT 3000        // таймаут соединения, мс
#define TIMEOUT_P 300000    // таймаут питания, мс
#define RESIST_10K 10000    // точное сопротивление 10к резистора (Ом)

// пределы
#define HW_TEMP_MIN 40  // мин. температура железа
#define HW_TEMP_MAX 60  // макс. температура железа
#define TEMP_MIN 32     // мин. температура воды
#define TEMP_MAX 40     // макс. температура воды

// у меня СВО начинает резонансить на скоростях 20-27, поэтому избегаю этот диапазон
#define PUMP_MIN 13     // мин. сигнал помпы (при TEMP_MIN)
#define PUMP_MAX 95     // макс. сигнал помпы (при TEMP_MAX)

#define PWM_MIN 50      // мин. сигнал вентиляторов (при TEMP_MIN)
#define PWM_MAX 150     // макс. сигнал вентиляторов (при TEMP_MAX)

#define TEMP_ON 34      // температура воды, выше которой включается вентилятор СВО
#define TEMP_OFF 32     // температура воды, ниже которой выключается вентилятор СВО

#define HW_TEMP_ON 45   // температура железа, выше которой включается вентилятор СВО
#define HW_TEMP_OFF 40  // температура железа, ниже которой выключается вентилятор СВО

#define COEF 0.2        // коэффициент плавности изменения температуры

// ---- термистор ----
// GND --- термистор --- A0 --- 10к --- 5V
#define RESIST_BASE 10000   // сопротивление при TEMP_BASE градусах по Цельсию (Ом), из даташита
#define TEMP_BASE 25        // температура, при которой измерено RESIST_BASE (градусов Цельсия)
#define B_COEF 3435         // бета коэффициент термистора (3000-4000)
// ---- термистор ----

// --- библиотеки ---
#include <DigiPotX9Cxxx.h>
DigiPot pot(POT_INC, POT_UD, POT_CS);

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f, 16, 2);
//LiquidCrystal_I2C lcd(0x27, 16, 2);

#include <PWM.h>

int pumpSpeed;
int waterTemp;

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
  InitTimersSafe();
  SetPinFrequencySafe(9, 25000);

  pinMode(RELAY_FAN, OUTPUT);
  pinMode(PWM_FAN, OUTPUT);

  digitalWrite(RELAY_FAN, HIGH);
  pwmWrite(PWM_FAN, 50);
  pot.set(20);

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
  /*lcd.clear();
    lcd.home();
    lcd.print(millis());
    delay(500);*/
  parsing();
  calcAndDisp();
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

    waterTemp = getThermTemp(analogRead(THERM_PIN));

    if (onlineFlag) {
      maxTemp += (float)(max(PCdata[0], PCdata[1]) - maxTemp) * COEF;
      if (maxTemp > HW_TEMP_ON) {
        pumpSpeed = map(maxTemp, HW_TEMP_MIN, HW_TEMP_MAX, PUMP_MIN, PUMP_MAX);
        pumpSpeed = constrain(pumpSpeed, PUMP_MIN, PUMP_MAX);
        
        digitalWrite(RELAY_FAN, 1);
        pwmWrite(PWM_FAN, map(maxTemp, HW_TEMP_MIN, HW_TEMP_MAX, PWM_MIN, PWM_MAX));
      } else if (maxTemp < HW_TEMP_OFF) {
        pumpSpeed = PUMP_MIN;
        digitalWrite(RELAY_FAN, 0);
        pwmWrite(PWM_FAN, PWM_MIN);
      }
    } else {
      if (waterTemp > TEMP_ON) {
        pumpSpeed = map(waterTemp, TEMP_MIN, TEMP_MAX, PUMP_MIN, PUMP_MAX);
        pumpSpeed = constrain(pumpSpeed, PUMP_MIN, PUMP_MAX);

        digitalWrite(RELAY_FAN, 1);
        pwmWrite(PWM_FAN, map(waterTemp, TEMP_MIN, TEMP_MAX, PWM_MIN, PWM_MAX));
      } else if (waterTemp < TEMP_OFF) {
        pumpSpeed = PUMP_MIN;
        digitalWrite(RELAY_FAN, 0);
        pwmWrite(PWM_FAN, PWM_MIN);
      }
    }

    pot.set(pumpSpeed);
    printData();

    pinState = !pinState;
    digitalWrite(13, pinState);
  }
}


float getThermTemp(int rawAnalog) {
  float thermistor;
  thermistor = RESIST_10K / ((float)1023 / rawAnalog - 1);
  thermistor /= RESIST_BASE;                        // (R/Ro)
  thermistor = log(thermistor) / B_COEF;            // 1/B * ln(R/Ro)
  thermistor += (float)1.0 / (TEMP_BASE + 273.15);  // + (1/To)
  thermistor = (float)1.0 / thermistor - 273.15;    // инвертируем и конвертируем в градусы по Цельсию
  return thermistor;
}
