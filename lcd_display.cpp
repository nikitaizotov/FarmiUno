#include "lcd_display.h"

LCD_Display::LCD_Display(uint8_t address, uint8_t cols, uint8_t rows)
  : lcd(address, cols, rows) {}

void LCD_Display::init() {
  lcd.init();
  lcd.backlight();
}

void LCD_Display::printText(const char* text, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void LCD_Display::clear() {
  lcd.clear();
}

void LCD_Display::backlightOn() {
  lcd.backlight();
}

void LCD_Display::backlightOff() {
  lcd.noBacklight();
}