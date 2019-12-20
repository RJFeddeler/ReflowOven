#include "Heater.h"

Heater::Heater() {
}

void Heater::init(int8_t relayPin, bool activeLow) {
  _power = 0.0f;
  _maxPower = 100.0f;
  _heaterRatio = 1.0f;
  _relay.init(relayPin, activeLow);
}

void Heater::turnOn() {
  #if defined(DEBUG)
  Serial.println("Heater.cpp: turnOn()");
  #endif
  
  _relay.close();
}

void Heater::turnOff() {
  #if defined(DEBUG)
  Serial.println("Heater.cpp: turnOff()");
  #endif
  
  _relay.open();
}

bool Heater::isActive() {
  return _relay.isClosed();
}

void Heater::setPower(float value) {
  #if defined(DEBUG)
  Serial.print("Heater.cpp: setPower( ");
  Serial.print(value);
  Serial.println(" )");
  #endif
  
  if (value > _maxPower)
    _power = _maxPower;
  else
    _power = value;

  if (_power == 0.0f)
    turnOff();
}

float Heater::getPower() {
  return _power;
}

void Heater::setMaxPower(float value) {
  #if defined(DEBUG)
  Serial.print("Heater.cpp: setMaxPower( ");
  Serial.print(value);
  Serial.println(" )");
  #endif
  
  if (value > 100.0f)
    _maxPower = 100.0f;
  else
    _maxPower = value;
}

float Heater::getMaxPower() {
  return _maxPower;
}

void Heater::setHeaterRatio(float value) {
  #if defined(DEBUG)
  Serial.print("Heater.cpp: setHeaterRatio( ");
  Serial.print(value);
  Serial.println(")");
  #endif
  
  if (value > 1.0f)
    _heaterRatio = 1.0f;
  else if (value < 0.0f)
    _heaterRatio = 0.0f;
  else
    _heaterRatio = value;
}

float Heater::getHeaterRatio() {
  return _heaterRatio;
}
