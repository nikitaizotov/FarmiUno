# FarmiUno

## Description

FarmiUno is an automated temperature and humidity control system based on Arduino. The project manages heating, cooling, fans, displays data on an LCD and LED matrix, and sends readings to a server via Wi-Fi. An additional Arduino Nano is used for fan control via I2C.

---

## Architecture

- **FarmiUno.ino** — main sketch, initializes all modules, main loop, processes sensor data, sends data to the server.
- **TemperatureControl.h/.cpp** — temperature, heating, cooling, and fan control, interaction with Nano via I2C.
- **lcd_display.h/.cpp** — LCD display control via I2C.
- **WifiManager.h/.cpp** — Wi-Fi connection management.
- **arduino_secrets.h** — stores SSID, Wi-Fi password, and server address (do not commit to public repository).

---

## Wiring Scheme (Mega/Uno)

| Purpose                   | Arduino Pin | Note                      |
|---------------------------|-------------|---------------------------|
| DHT11 Sensor (Data)       | D2          | Temperature/Humidity      |
| Cooler (PWM)              | D3          | FAN_PIN                   |
| Peltier (cooling)         | D9          | RPWM                      |
| Peltier (heating)         | D10         | LPWM                      |
| LCD (I2C)                 | SDA/SCL     | Address 0x27              |
| Nano (I2C)                | SDA/SCL     | Address 0x10              |
| LED matrix                | SPI/I2C     | Via ArduinoLEDMatrix      |

---

## Quick Start

1. **Fill in the `arduino_secrets.h` file:**
   ```cpp
   #define SECRET_SSID "<your Wi-Fi>"
   #define SECRET_PASS "<your password>"
   #define SECRET_SRV_ADDR "<server IP>"
   ```
2. **Upload the sketch to Arduino Mega/Uno.**
3. **Connect all modules according to the table above.**
4. **Make sure Arduino Nano is connected via I2C (address 0x10).**
5. **Open Serial Monitor for debugging.**

---

## Module Description

### TemperatureControl
- Controls heating/cooling via D9/D10 pins (Peltier).
- Controls fans via Nano over I2C.
- Allows you to set the target temperature, implements delays between modes.

### LCD_Display
- Displays text information on the LCD (I2C, address 0x27).
- Used to display current values and statuses.

### WifiManager
- Connects Arduino to Wi-Fi.
- Checks connection status, reconnects if disconnected.
- Outputs debug information to Serial.

### Main Sketch (FarmiUno.ino)
- Initializes all modules.
- Reads data from DHT11.
- Controls temperature and fans.
- Sends data to the server via HTTP POST.
- Updates LCD and LED matrix.

---

## Notes
- To work with the LED matrix and LCD, you need the appropriate libraries (Arduino_LED_Matrix, LiquidCrystal_I2C, etc.).
- Arduino Nano must be flashed with a separate firmware for I2C operation (see separate repository).
- Store all secret data only in `arduino_secrets.h`.

---

## Contacts
Questions and suggestions: [your email or Telegram] 