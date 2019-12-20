#include "Relay.h"

Relay::Relay() {
}

void Relay::init(int8_t pin, bool activeLow) {
  _pin = pin;
  _activeLow = activeLow;
  
  pinMode(_pin, OUTPUT);
  open();
}

void Relay::close() {
  if (_activeLow)
    digitalWrite(_pin, LOW);
  else
    digitalWrite(_pin, HIGH);
  
  _closed = true;
}

void Relay::open() {
  if (_activeLow)
    digitalWrite(_pin, HIGH);
  else
    digitalWrite(_pin, LOW);
  
  _closed = false;
}

bool Relay::isClosed() {
  return _closed;
}
