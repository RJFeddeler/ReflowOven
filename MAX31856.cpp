#include "MAX31856.h"

#include <SPI.h>

static SPISettings spiSettings = SPISettings(500000, MSBFIRST, SPI_MODE1);

MAX31856::MAX31856() {
}

void MAX31856::init(int8_t cs, int8_t drdy) {
  _cs = cs;
  _drdy = drdy;
  
  _tcTemperature = 0.0f;
  _cjTemperature = 0.0f;
  _fault = 0x00;

  _tcTemperatureWindow[0] = _tcTemperatureWindow[1] = _tcTemperatureWindow[2] = 0.0f;
  _cjTemperatureWindow[0] = _cjTemperatureWindow[1] = _cjTemperatureWindow[2] = 0.0f;
  _windowPos = 0;
  
  pinMode(_drdy, INPUT_PULLUP);
  
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  SPI.begin();
}

void MAX31856::startTemperatureSampling() {
  writeRegister8(MAX31856_CR1_REG, MAX31856_CR1_16SAMPLE | MAX31856_CR1_KTYPE);
  writeRegister8(MAX31856_CR0_REG, MAX31856_CR0_AUTOCONVERT);
}

uint8_t MAX31856::readTemperatures() {
  uint8_t regData[6];
  readRegisters(6, MAX31856_CJTH_REG, &regData[0]);

  uint16_t cj = ((uint16_t)regData[0] << 8) | regData[1];
  _cjTemperatureWindow[_windowPos] = (int16_t)cj / 256.0f;
  
  int32_t tc = ((uint32_t)regData[2] << 24) | ((uint32_t)regData[3] << 16) | ((uint16_t)regData[4] << 8);
  tc >>= 13;
  _tcTemperatureWindow[_windowPos++] = tc * 0.0078125;

  _fault = regData[5];

  if (_windowPos >= WINDOW_SIZE) {
    _windowPos = 0;

    float tmp;
    bool swapped;
    do {
      swapped = false;
      
      for (uint8_t i = 0; i < WINDOW_SIZE-1; i++) {
        if (_cjTemperatureWindow[i] > _cjTemperatureWindow[i+1]) {
          tmp = _cjTemperatureWindow[i];
          _cjTemperatureWindow[i] = _cjTemperatureWindow[i+1];
          _cjTemperatureWindow[i+1] = tmp;
          swapped = true;
        }

        if (_tcTemperatureWindow[i] > _tcTemperatureWindow[i+1]) {
          tmp = _tcTemperatureWindow[i];
          _tcTemperatureWindow[i] = _tcTemperatureWindow[i+1];
          _tcTemperatureWindow[i+1] = tmp;
          swapped = true;
        }
      }
    } while (swapped);

    _cjTemperature = _cjTemperatureWindow[WINDOW_SIZE/2];
    _tcTemperature = _tcTemperatureWindow[WINDOW_SIZE/2];

    return 1;
  }

  return 0;
}

bool MAX31856::dataReady() {
  if (digitalRead(_drdy) == LOW)
    return true;

  return false;
}

uint8_t MAX31856::getFault() {
  return _fault;
}

float MAX31856::getTemperature() {
  return _tcTemperature;
}

float MAX31856::getColdJunctionTemperature() {
  return _cjTemperature;
}

void MAX31856::readRegisters(uint8_t count, uint8_t addr, uint8_t *data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_cs, LOW);
  
  SPI.transfer(addr);
  
  for (uint8_t i = 0; i < count; i++)
    data[i] = SPI.transfer(0xFF);

  SPI.endTransaction();
  digitalWrite(_cs, HIGH);
}

uint8_t MAX31856::readRegister8(uint8_t addr) {
  uint8_t data;
  readRegisters(1, addr, &data);

  return data;
}

uint16_t MAX31856::readRegister16(uint8_t addr) {
  uint8_t data[2];
  readRegisters(2, addr, &data[0]);

  return ((uint16_t)data[0] << 8) | data[1];
}

void MAX31856::writeRegister8(uint8_t addr, uint8_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_cs, LOW);
  
  SPI.transfer(addr | 0x80);
  SPI.transfer(data);

  SPI.endTransaction();
  digitalWrite(_cs, HIGH);
}
