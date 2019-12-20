#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>


class Relay {
  public:
    Relay();
    void init(int8_t pin, bool activeLow);

    void close();
    void open();
    
    bool isClosed();

  private:
    int8_t _pin;
    bool _activeLow;
    bool _closed;
};


#endif
