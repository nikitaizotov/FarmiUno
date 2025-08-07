#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * Class for temperature control, heating, cooling, and fan management.
 *
 * Allows you to set the target temperature, control actuators, and interact with Arduino Nano via I2C.
 */
class TemperatureControl {
public:
    /**
     * Constructor for TemperatureControl.
     *
     * @param coolPWM PWM pin for cooling (Peltier).
     * @param heatPWM PWM pin for heating (Peltier).
     * @param nanoAddr I2C address of Arduino Nano (default 0x10).
     */
    TemperatureControl(uint8_t coolPWM, uint8_t heatPWM, uint8_t nanoAddr = 0x10);

    /**
     * Initializes pins and initial states for temperature control.
     *
     * Call this in setup(). Sets all actuators to off and prepares the system.
     */
    void begin();

    /**
     * Main temperature control logic. Call in loop().
     *
     * - Decides whether to heat, cool, or idle based on current and target temperature.
     * - Handles fan and actuator switching, including cooldown delays.
     */
    void handle();

    /**
     * Turns off all actuators (heating, cooling, fans).
     *
     * Use this to ensure the system is in a safe state.
     */
    void offAll();

    /**
     * Forces cooling mode: disables heating and Peltier, enables fans.
     *
     * Useful for emergency or maintenance situations.
     */
    void forceCooling();

    /**
     * Enables or disables cooling (Peltier).
     *
     * @param enabled true to enable cooling, false to disable.
     */
    void setCooling(bool enabled);

    /**
     * Enables or disables heating (Peltier).
     *
     * @param enabled true to enable heating, false to disable.
     */
    void setHeating(bool enabled);

    /**
     * Enables or disables the Nano fan via I2C.
     *
     * @param enabled true to enable the fan, false to disable.
     */
    void setNanoFan(bool enabled);

    /**
     * Sets the PWM value for the Nano fan via I2C.
     *
     * @param value PWM value (0-255).
     */
    void setNanoFanPWM(uint8_t value);

    /**
     * Sets the Nano fan speed (saved for later use).
     *
     * @param value Speed value (0-255).
     */
    void setNanoFanSpeed(uint8_t value);

    /**
     * Sets the Peltier fan speed via I2C.
     *
     * @param speed Speed value (0-255).
     */
    void setPeltierFanSpeed(uint8_t speed);

    /**
     * Sets the target temperature for the system.
     *
     * @param target Target temperature in degrees Celsius.
     */
    void setTargetTemperature(float target);

    /**
     * Sets the current temperature value (e.g., from a sensor).
     *
     * @param temp Current temperature in degrees Celsius.
     */
    void setCurrentTemperature(float temp);

    /**
     * Checks if heating is currently enabled.
     *
     * @return true if heating is active, false otherwise.
     */
    bool isHeating() const { return _isHeating; }

    /**
     * Checks if cooling is currently enabled.
     *
     * @return true if cooling is active, false otherwise.
     */
    bool isCooling() const { return _isCooling; }

    /**
     * Checks if the system is in idle mode (neither heating nor cooling, but in cooldown delay).
     *
     * @return true if idle, false otherwise.
     */
    bool isIdle() const { return !_isHeating && !_isCooling && _isCooldownDelay; }

private:
    /**
     * Stops the Peltier device (both heating and cooling).
     */
    void stopPeltier();

    uint8_t _coolPWMPin;      ///< PWM pin for cooling
    uint8_t _heatPWMPin;      ///< PWM pin for heating
    uint8_t _nanoAddr;        ///< I2C address of Arduino Nano

    float _targetTemp = 24.0; ///< Target temperature
    float _currentTemp = 0.0; ///< Current temperature
    float _tolerance = 1.0;   ///< Allowed deviation

    bool _isCooling = false;      ///< Cooling is active
    bool _isHeating = false;      ///< Heating is active
    bool _isCooldownDelay = false;///< Delay after heating/cooling

    unsigned long _lastCoolingTime = 0; ///< Last cooling time
    unsigned long _lastHeatingTime = 0; ///< Last heating time
    const unsigned long _fanCooldownDuration = 180000; ///< Passive cooling time (ms)

    // nano fan not used (reserved)
    // TODO: remove nano fan logic
    uint8_t _nanoFanSpeed = 179; ///< Nano fan speed
    uint8_t _peltierFanSpeed = 255; ///< Peltier fan speed
};
