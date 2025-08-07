#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/**
 * Class for controlling an LCD display via I2C.
 *
 * Allows you to initialize the display, print text, and control the backlight.
 */
class LCD_Display {
public:
    /**
     * Constructor for LCD_Display.
     *
     * @param address I2C address of the display (default 0x27)
     * @param cols Number of columns (default 16)
     * @param rows Number of rows (default 2)
     */
    LCD_Display(uint8_t address = 0x27, uint8_t cols = 16, uint8_t rows = 2);

    /**
     * Initializes the LCD display (call in setup()).
     */
    void init();

    /**
     * Prints text to the display at the given coordinates.
     *
     * @param text String to print
     * @param col Column (from 0)
     * @param row Row (from 0)
     */
    void printText(const char* text, uint8_t col, uint8_t row);

    /**
     * Clears the display.
     */
    void clear();

    /**
     * Turns on the backlight.
     */
    void backlightOn();

    /**
     * Turns off the backlight.
     */
    void backlightOff();

private:
    LiquidCrystal_I2C lcd; /**< LCD library instance */
};

#endif