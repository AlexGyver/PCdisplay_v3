static void show_logo() {
  lcd.createChar(0, logo0);
  lcd.createChar(1, logo1);
  lcd.createChar(2, logo2);
  lcd.createChar(3, logo3);
  lcd.createChar(4, logo4);
  lcd.createChar(5, logo5);
  lcd.home();
  lcd.write(0);
  lcd.write(1);
  lcd.write(2);
  lcd.setCursor(0, 1);
  lcd.write(3);
  lcd.write(4);
  lcd.write(5);
  lcd.setCursor(5, 0);
  lcd.print("AlexGyver");
  lcd.setCursor(4, 1);
  lcd.print("Technologies");
  delay(1500);
}

void printData() {
  drawTimer = millis();
  lcd.setCursor(0, 0);
  lcd.write(0);
  if (onlineFlag) {
    lcd.print(PCdata[0]);
    lcd.write(223);
    lcd.print(PCdata[4]);
    if (PCdata[4] < 10) lcd.print("%  ");
    else lcd.print("% ");
  } else {
    lcd.print(F("---   "));
  }

  lcd.setCursor(9, 0);
  lcd.write(1);
  if (onlineFlag) {
    lcd.print(PCdata[1]);
    lcd.write(223);
    lcd.print(PCdata[5]);
    if (PCdata[5] < 10) lcd.print("% ");
    else lcd.print("%");
  } else {
    lcd.print(F("---   "));
  }

  lcd.setCursor(0, 1);
  lcd.write(2);
  if (onlineFlag) {
    lcd.print(PCdata[6]);
    lcd.print("%");
    lcd.print("-");
    lcd.print(PCdata[7]);
    lcd.print("%  ");
  } else {
    lcd.print(F("---    "));
  }

  lcd.setCursor(9, 1);
  lcd.write(3);
  lcd.print(waterTemp);
  lcd.write(223);
  lcd.print(pumpSpeed);
  if (pumpSpeed < 10)  lcd.print("% ");
  else lcd.print("%");
}
