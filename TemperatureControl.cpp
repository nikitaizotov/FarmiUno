#include "TemperatureControl.h"
#include <Wire.h>

TemperatureControl::TemperatureControl(uint8_t coolPWM, uint8_t heatPWM, uint8_t nanoAddr)
  : _coolPWMPin(coolPWM), _heatPWMPin(heatPWM), _nanoAddr(nanoAddr) {}

void TemperatureControl::begin() {
  pinMode(_coolPWMPin, OUTPUT);
  pinMode(_heatPWMPin, OUTPUT);
  setNanoFanPWM(0);
  offAll();

  _isCooldownDelay = true;
  _lastCoolingTime = millis();
  _lastHeatingTime = millis();
}

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

void TemperatureControl::stopPeltier() {
  analogWrite(_coolPWMPin, 0);
  analogWrite(_heatPWMPin, 0);
}

void TemperatureControl::setPeltierFanSpeed(uint8_t speed) {
  Wire.beginTransmission(_nanoAddr);
  Wire.write(3);
  Wire.write(speed);
  Wire.endTransmission();
}

void TemperatureControl::setNanoFan(bool enabled) {
  setNanoFanPWM(enabled ? _nanoFanSpeed : 0);
}

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

void TemperatureControl::setNanoFanSpeed(uint8_t value) {
  _nanoFanSpeed = constrain(value, 0, 255);
  Serial.print("Set NanoFan Speed = ");
  Serial.println(_nanoFanSpeed);
}

void TemperatureControl::setTargetTemperature(float target) {
  _targetTemp = target;
}

void TemperatureControl::setCurrentTemperature(float temp) {
  _currentTemp = temp;
}

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

  // Calcualting limits.
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


void TemperatureControl::offAll() {
  stopPeltier();
  setPeltierFanSpeed(0);
  setNanoFanPWM(0);
}

void TemperatureControl::forceCooling() {
  stopPeltier();
  setPeltierFanSpeed(_peltierFanSpeed);
  setNanoFanPWM(0);
  Serial.println("⚠️ Force cooling mode: Peltier OFF, fans ON (Nano fan OFF)");
}