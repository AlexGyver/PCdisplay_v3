/*
   DigiPot.pde - Example sketch for Arduino library for managing digital potentiometers X9C1xxx (xxx = 102,103,104,503).
   By Timo Fager, Jul 29, 2011.
   Released to public domain.

   For this example, connect your X9C103P (or the like) as follows:
   1 - INC - Arduino pin 2
   2 - U/D - Arduino pin 3
   3 - VH  - 5V
   4 - VSS - GND
   5 - VW  - Output: 150 Ohm resistor -> LED -> GND
   6 - VL  - GND
   7 - CS  - Arduino pin 4
   8 - VCC - 5V

 **/

#define INC 2
#define UD 3
#define CS 4

#include <DigiPotX9Cxxx.h>

DigiPot pot(INC, UD, CS);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("Starting");

  for (int i = 0; i < 100; i++) {
    Serial.print("Increasing, i = ");
    Serial.println(i, DEC);
    pot.increase(1);
    delay(200);
  }

  for (int i = 0; i < 100; i++) {
    Serial.print("Decreasing, i = ");
    Serial.println(i, DEC);
    pot.decrease(1);
    delay(200);
  }

}


