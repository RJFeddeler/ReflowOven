#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
//#include "globals.h"
#include "Relay.h"


class Heater {
  public:
    
    Heater();
    void init(int8_t relayPin, bool activeLow);

    void turnOn();
    void turnOff();

    bool isActive();

    void setPower(float value);
    float getPower();

    void setMaxPower(float value);
    float getMaxPower();
    
    void setHeaterRatio(float value);
    float getHeaterRatio();
    
  private:

    bool _locked;

    float _power;
    float _maxPower;
    
    float _heaterRatio;

    Relay _relay;
};


#endif
