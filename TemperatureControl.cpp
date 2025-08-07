#include "TemperatureControl.h"
#include <Wire.h>

/**
 * Constructor: initializes pins and Nano address.
 * @param coolPWM - PWM pin for cooling (Peltier)
 * @param heatPWM - PWM pin for heating (Peltier)
 * @param nanoAddr - I2C address of Arduino Nano
 */
TemperatureControl::TemperatureControl(uint8_t coolPWM, uint8_t heatPWM, uint8_t nanoAddr)
  : _coolPWMPin(coolPWM), _heatPWMPin(heatPWM), _nanoAddr(nanoAddr) {}

/**
 * Initializes pins, sets initial states.
 */
void TemperatureControl::begin() {
  pinMode(_coolPWMPin, OUTPUT);
  pinMode(_heatPWMPin, OUTPUT);
  setNanoFanPWM(0);
  offAll();

  _isCooldownDelay = true;
  _lastCoolingTime = millis();
  _lastHeatingTime = millis();
}

/**
 * Enable/disable cooling (Peltier).
 * @param enabled - true to enable, false to disable
 */
void TemperatureControl::setCooling(bool enabled) {
  if (enabled && !_isCooling) {
    analogWrite(_coolPWMPin, 255);
    analogWrite(_heatPWMPin, 0);
    _isCooling = true;
    _isHeating = false;
    _isCooldownDelay = false;
    _lastCoolingTime = millis();
  } else if (!enabled && _isCooling) {
    analogWrite(_coolPWMPin, 0);
    _isCooling = false;
    _lastCoolingTime = millis();
    _isCooldownDelay = true;
  }
}

/**
 * Enable/disable heating (Peltier).
 * @param enabled - true to enable, false to disable
 */
void TemperatureControl::setHeating(bool enabled) {
  if (enabled && !_isHeating) {
    analogWrite(_heatPWMPin, 255);
    analogWrite(_coolPWMPin, 0);
    _isHeating = true;
    _isCooling = false;
    _isCooldownDelay = false;
    _lastHeatingTime = millis();
  } else if (!enabled && _isHeating) {
    analogWrite(_heatPWMPin, 0);
    _isHeating = false;
    _lastHeatingTime = millis();
    _isCooldownDelay = true;
  }
}

/**
 * Stop Peltier (heating and cooling).
 */
void TemperatureControl::stopPeltier() {
  analogWrite(_coolPWMPin, 0);
  analogWrite(_heatPWMPin, 0);
}

/**
 * Control Peltier fan speed via I2C.
 * @param speed - speed to set
 */
void TemperatureControl::setPeltierFanSpeed(uint8_t speed) {
  Wire.beginTransmission(_nanoAddr);
  Wire.write(3);
  Wire.write(speed);
  Wire.endTransmission();
}

/**
 * Control Nano fan via I2C (enable/disable).
 * @param enabled - true to enable, false to disable
 */
void TemperatureControl::setNanoFan(bool enabled) {
  setNanoFanPWM(enabled ? _nanoFanSpeed : 0);
}

/**
 * Set PWM for Nano fan via I2C.
 * @param value - PWM value
 */
void TemperatureControl::setNanoFanPWM(uint8_t value) {
  static uint8_t lastValue = 255;
  if (value != lastValue) {
    Wire.beginTransmission(_nanoAddr);
    Wire.write(0);
    Wire.write(value);
    Wire.endTransmission();
    lastValue = value;
  }
}

/**
 * Set Nano fan speed (saved for later use).
 * @param value - speed to set
 */
void TemperatureControl::setNanoFanSpeed(uint8_t value) {
  _nanoFanSpeed = constrain(value, 0, 255);
  Serial.print("Set NanoFan Speed = ");
  Serial.println(_nanoFanSpeed);
}

/**
 * Set target temperature.
 * @param target - target temperature
 */
void TemperatureControl::setTargetTemperature(float target) {
  _targetTemp = target;
}

/**
 * Set current temperature.
 * @param temp - current temperature
 */
void TemperatureControl::setCurrentTemperature(float temp) {
  _currentTemp = temp;
}

/**
 * Main temperature control logic: enables/disables heating/cooling, controls fans, implements delays.
 */
void TemperatureControl::handle() {
  unsigned long now = millis();

  if (_isCooldownDelay) {
    if (now - _lastCoolingTime >= _fanCooldownDuration &&
        now - _lastHeatingTime >= _fanCooldownDuration) {
      _isCooldownDelay = false;
    } else {
      stopPeltier();
      setPeltierFanSpeed(255);
      setNanoFan(false); 
      Serial.println("IDLE — passive cooling!");
      return;
    }
  }

  // Calculating limits.
  const float heatStart = _targetTemp - _tolerance;
  const float coolStart = _targetTemp + _tolerance;
  const float heatStop = _targetTemp - 0.1;
  const float coolStop = _targetTemp + 0.1;

  bool requestHeating = _currentTemp < heatStart;
  bool requestCooling = _currentTemp > coolStart;

  // Stop when we near the target.
  if (_isHeating && _currentTemp >= heatStop) {
    setHeating(false);
  } else if (_isCooling && _currentTemp <= coolStop) {
    setCooling(false);
  }

  // Enable only if there was no opposite mode enabled.
  if (requestHeating && !_isCooling && !_isHeating) {
    setHeating(true);
  } else if (requestCooling && !_isHeating && !_isCooling) {
    setCooling(true);
  }

  bool fansShouldRun = _isCooling || _isHeating ||
    (now - _lastCoolingTime < _fanCooldownDuration) ||
    (now - _lastHeatingTime < _fanCooldownDuration);

  setPeltierFanSpeed(fansShouldRun ? _peltierFanSpeed : 0);
  setNanoFan(fansShouldRun && (_isCooling || _isHeating));

  Serial.print("Temp: ");
  Serial.print(_currentTemp);
  Serial.print(" / Target: ");
  Serial.print(_targetTemp);
  Serial.print(" => Mode: ");
  if (_isHeating) Serial.println("HEAT");
  else if (_isCooling) Serial.println("COOL");
  else Serial.println("IDLE");
}

/**
 * Turn off all actuators.
 */
void TemperatureControl::offAll() {
  stopPeltier();
  setPeltierFanSpeed(0);
  setNanoFanPWM(0);
}

/**
 * Force cooling mode: Peltier off, fans on.
 */
void TemperatureControl::forceCooling() {
  stopPeltier();
  setPeltierFanSpeed(_peltierFanSpeed);
  setNanoFanPWM(0);
  Serial.println("⚠️ Force cooling mode: Peltier OFF, fans ON (Nano fan OFF)");
}