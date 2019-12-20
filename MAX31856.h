#ifndef MAX31856_H
#define MAX31856_H

#define MAX31856_CR0_REG           0x00    ///< Config 0 register
#define MAX31856_CR0_AUTOCONVERT   0x80    ///< Config 0 Auto convert flag
#define MAX31856_CR0_1SHOT         0x40    ///< Config 0 one shot convert flag
#define MAX31856_CR0_OCFAULT1      0x20    ///< Config 0 open circuit fault 1 flag
#define MAX31856_CR0_OCFAULT0      0x10    ///< Config 0 open circuit fault 0 flag
#define MAX31856_CR0_CJ            0x08    ///< Config 0 cold junction disable flag
#define MAX31856_CR0_FAULT         0x04    ///< Config 0 fault mode flag
#define MAX31856_CR0_FAULTCLR      0x02    ///< Config 0 fault clear flag

#define MAX31856_CR1_REG           0x01    ///< Config 1 register
#define MAX31856_CR1_16SAMPLE      0x40
#define MAX31856_CR1_8SAMPLE       0x30
#define MAX31856_CR1_4SAMPLE       0x20
#define MAX31856_CR1_2SAMPLE       0x10
#define MAX31856_CR1_1SAMPLE       0x00
#define MAX31856_CR1_KTYPE         0x03

#define MAX31856_MASK_REG          0x02    ///< Fault Mask register
#define MAX31856_CJHF_REG          0x03    ///< Cold junction High temp fault register
#define MAX31856_CJLF_REG          0x04    ///< Cold junction Low temp fault register
#define MAX31856_LTHFTH_REG        0x05    ///< Linearized Temperature High Fault Threshold Register, MSB
#define MAX31856_LTHFTL_REG        0x06    ///< Linearized Temperature High Fault Threshold Register, LSB
#define MAX31856_LTLFTH_REG        0x07    ///< Linearized Temperature Low Fault Threshold Register, MSB
#define MAX31856_LTLFTL_REG        0x08    ///< Linearized Temperature Low Fault Threshold Register, LSB
#define MAX31856_CJTO_REG          0x09    ///< Cold-Junction Temperature Offset Register 
#define MAX31856_CJTH_REG          0x0A    ///< Cold-Junction Temperature Register, MSB
#define MAX31856_CJTL_REG          0x0B    ///< Cold-Junction Temperature Register, LSB
#define MAX31856_LTCBH_REG         0x0C    ///< Linearized TC Temperature, Byte 2 
#define MAX31856_LTCBM_REG         0x0D    ///< Linearized TC Temperature, Byte 1
#define MAX31856_LTCBL_REG         0x0E    ///< Linearized TC Temperature, Byte 0
#define MAX31856_SR_REG            0x0F    ///< Fault Status Register

#define MAX31856_FAULT_CJRANGE     0x80    ///< Fault status Cold Junction Out-of-Range flag
#define MAX31856_FAULT_TCRANGE     0x40    ///< Fault status Thermocouple Out-of-Range flag
#define MAX31856_FAULT_CJHIGH      0x20    ///< Fault status Cold-Junction High Fault flag
#define MAX31856_FAULT_CJLOW       0x10    ///< Fault status Cold-Junction Low Fault flag
#define MAX31856_FAULT_TCHIGH      0x08    ///< Fault status Thermocouple Temperature High Fault flag
#define MAX31856_FAULT_TCLOW       0x04    ///< Fault status Thermocouple Temperature Low Fault flag
#define MAX31856_FAULT_OVUV        0x02    ///< Fault status Overvoltage or Undervoltage Input Fault flag
#define MAX31856_FAULT_OPEN        0x01    ///< Fault status Thermocouple Open-Circuit Fault flag

#define WINDOW_SIZE                3

#include <Arduino.h>


class MAX31856 {
  public:
    MAX31856();
    void init(int8_t cs, int8_t drdy);
    
    void startTemperatureSampling();
    uint8_t readTemperatures();
    bool dataReady();
    
    uint8_t getFault();
    float getTemperature();
    float getColdJunctionTemperature();

  private:
    uint8_t readFault();
    
    void readRegisters(uint8_t count, uint8_t addr, uint8_t *data);
    uint8_t readRegister8(uint8_t addr);
    uint16_t readRegister16(uint8_t addr);
    void writeRegister8(uint8_t addr, uint8_t data);

    int8_t _cs;
    int8_t _drdy;

    float _tcTemperature;
    float _cjTemperature;
    uint8_t _fault;

    float _tcTemperatureWindow[WINDOW_SIZE];
    float _cjTemperatureWindow[WINDOW_SIZE];
    uint8_t _windowPos;
};


#endif
