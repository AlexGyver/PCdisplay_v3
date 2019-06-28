/*
 * DigiPotX9Cxxx.cpp - Arduino library for managing digital potentiometers X9Cxxx (xxx = 102,103,104,503).
 * By Timo Fager, Jul 29, 2011.
 * Released to public domain.
 **/

#include "Arduino.h"
#include "DigiPotX9Cxxx.h"

DigiPot::DigiPot(uint8_t incPin, uint8_t udPin, uint8_t csPin) {
  _incPin = incPin;
  _udPin = udPin;
  _csPin = csPin;  
  _currentValue = DIGIPOT_UNKNOWN;

  pinMode(_incPin, OUTPUT);
  pinMode(_udPin, OUTPUT);
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

}

void DigiPot::reset() {
  // change down maximum number of times to ensure the value is 0
  decrease(DIGIPOT_MAX_AMOUNT);
  _currentValue = 0;
}

void DigiPot::set(uint8_t value) {
  value = constrain(value, 0, DIGIPOT_MAX_AMOUNT);
  if (_currentValue == DIGIPOT_UNKNOWN) reset();
  if (_currentValue > value) {
    change(DIGIPOT_DOWN, _currentValue-value);
  } else if (_currentValue < value) {
    change(DIGIPOT_UP, value-_currentValue);
  }
}

uint8_t DigiPot::get() {
  return _currentValue;
}

void DigiPot::increase(uint8_t amount) {
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  change(DIGIPOT_UP, amount);
}

void DigiPot::decrease(uint8_t amount) {
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  change(DIGIPOT_DOWN, amount);
}

void DigiPot::change(uint8_t direction, uint8_t amount) {
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  digitalWrite(_udPin, direction);
  digitalWrite(_incPin, HIGH);
  digitalWrite(_csPin, LOW);

  for (uint8_t i=0; i<amount; i++) {
    digitalWrite(_incPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_incPin, HIGH);
    delayMicroseconds(2);
    if (_currentValue != DIGIPOT_UNKNOWN) {
      _currentValue += (direction == DIGIPOT_UP ? 1 : -1);
      _currentValue = constrain(_currentValue, 0, DIGIPOT_MAX_AMOUNT);
    }
    
  }
  digitalWrite(_csPin, HIGH);
}

