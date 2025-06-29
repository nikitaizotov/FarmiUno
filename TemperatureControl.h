#pragma once

#include <Arduino.h>
#include <Wire.h>

class TemperatureControl {
public:
  TemperatureControl(uint8_t coolPWM, uint8_t heatPWM, uint8_t nanoAddr = 0x10);

  void begin();
  void handle();
  void offAll();
  void forceCooling();

  void setCooling(bool enabled);
  void setHeating(bool enabled);

  void setNanoFan(bool enabled);
  void setNanoFanPWM(uint8_t value);
  void setNanoFanSpeed(uint8_t value);
  void setPeltierFanSpeed(uint8_t speed);

  void setTargetTemperature(float target);
  void setCurrentTemperature(float temp);

  bool isHeating() const { return _isHeating; }
  bool isCooling() const { return _isCooling; }
  bool isIdle() const { return !_isHeating && !_isCooling && _isCooldownDelay; }

private:
  void stopPeltier();

  uint8_t _coolPWMPin;
  uint8_t _heatPWMPin;
  uint8_t _nanoAddr;

  float _targetTemp = 24.0;
  float _currentTemp = 0.0;
  float _tolerance = 1.0;

  bool _isCooling = false;
  bool _isHeating = false;
  bool _isCooldownDelay = false;

  unsigned long _lastCoolingTime = 0;
  unsigned long _lastHeatingTime = 0;
  const unsigned long _fanCooldownDuration = 180000;

  // nano fan not used
  // TODO: wipe out nano fan logic
  uint8_t _nanoFanSpeed = 179;
  uint8_t _peltierFanSpeed = 255;
};
