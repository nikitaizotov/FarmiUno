#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

class LCD_Display {
public:
  LCD_Display(uint8_t address = 0x27, uint8_t cols = 16, uint8_t rows = 2);
  void init();
  void printText(const char* text, uint8_t col, uint8_t row);
  void clear();
  void backlightOn();
  void backlightOff();


private:
  LiquidCrystal_I2C lcd;
};

#endif