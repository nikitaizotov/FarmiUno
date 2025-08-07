#include "lcd_display.h"

/**
 * Constructor: initializes LCD with given parameters.
 * @param address - I2C address of the LCD
 * @param cols - number of columns
 * @param rows - number of rows
 */
LCD_Display::LCD_Display(uint8_t address, uint8_t cols, uint8_t rows)
  : lcd(address, cols, rows) {}

/**
 * Initializes the LCD display.
 */
void LCD_Display::init() {
  lcd.init();
  lcd.backlight();
}

/**
 * Prints text to the display at the given coordinates
 * @param text - text to print
 * @param col - column
 * @param row - row
 */
void LCD_Display::printText(const char* text, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

/**
 * Clears the display.
 */
void LCD_Display::clear() {
  lcd.clear();
}

/**
 * Turns on the backlight.
 */
void LCD_Display::backlightOn() {
  lcd.backlight();
}

/**
 * Turns off the backlight.
 */
void LCD_Display::backlightOff() {
  lcd.noBacklight();
}