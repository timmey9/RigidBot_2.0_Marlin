/*
  adp7142.cpp - Arduino library for NXP Semiconductor ADP7142 i2c LED driver
  For implementation details, please take a look at the datasheet http://www.nxp.com/documents/data_sheet/PCA9551.pdf
*/

#ifndef adp7142_h
#define adp7142_h

//#include <Arduino.h>
#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
#endif
//#include <WProgram.h>
#include <Wire.h>

#define ADDRESS 0xCC // (user manual 6.1)


// Control Register commands (user manual 6.2)
#define INPUT 0x00
#define PSC0  0x01
#define PWM0  0x02
#define PSC1  0x03
#define PWM1  0x04
#define LS0   0x05
#define LS1   0x06

#define AI    0x10 // auto increment

#define OFF    0x00
#define HIGH_Z 0x55
#define BLINK0 0xAA
#define BLINK1 0xFF


class adp7142
{
  public:
    adp7142();
    uint8_t ledsOff();

    uint8_t setPSC0(uint8_t);
    uint8_t setPSC1(uint8_t);
    uint8_t setPWM0(uint8_t);
    uint8_t setPWM1(uint8_t);

    uint8_t setLS0(uint8_t);
    uint8_t setLS1(uint8_t);

  private:
    /*
    uint8_t      fastWrite();
    uint8_t      multiWrite();
    uint8_t      singleWrite(uint8_t);
    uint8_t      seqWrite();
    uint8_t      writeVref();
    uint8_t      writeGain();
    uint8_t      writePowerDown();
    void         writeVout();
    uint8_t      _dev_address;
    uint16_t     _vOut[4];
    uint16_t     _vdd;
    */
    uint8_t      _basicWrite(byte, byte);
    uint8_t      _basicRead(byte);
    
};
#endif

