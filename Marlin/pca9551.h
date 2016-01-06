/*
  pca9551.cpp - Arduino library for NXP Semiconductor pca9551 i2c LED driver
  For implementation details, please take a look at the datasheet http://www.nxp.com/documents/data_sheet/PCA9551.pdf
*/

#ifndef pca9551_h
#define pca9551_h

//#include <Arduino.h>
#if (ARDUINO >= 100)
# include "Arduino.h"
#else
# include "WProgram.h"
#endif
//#include <WProgram.h>
#include <Wire.h>

#define ADDRESS 0x67 // (user manual 6.1)


// Control Register commands (user manual 6.2)
#define INPUT_ 0x00 // INPUT is already defined as something else
#define PSC0   0x01
#define PWM0   0x02
#define PSC1   0x03
#define PWM1   0x04
#define LS0    0x05
#define LS1    0x06

#define AI    0x10 // auto increment

// settings for the setLedSources() function
#define LED_OFF    0
#define LED_ON     1
#define LED_BLINK0 2
#define LED_BLINK1 3


class pca9551
{
  public:
    pca9551();
    uint8_t ledsOff();
    uint8_t ledsOn();
    
    float getPeriod0();
    float getPeriod1();
    void setPeriod0(float);
    void setPeriod1(float);

    float getDutyCycle0();
    float getDutyCycle1();
    void setDutyCycle0(float);
    void setDutyCycle1(float);

    void setLedSources(int red, int green, int blue);
    int getLedSources();

  private:
    uint8_t      _basicWrite(byte, byte);
    uint8_t      _basicRead(byte);

    uint8_t writeLS0(uint8_t);
    uint8_t writeLS1(uint8_t);
    uint8_t writePSC0(uint8_t);
    uint8_t writePSC1(uint8_t);
    uint8_t writePWM0(uint8_t);
    uint8_t writePWM1(uint8_t);

    uint8_t readInput();
    uint8_t readLS0();
    uint8_t readLS1();
    uint8_t readPSC0();
    uint8_t readPSC1();
    uint8_t readPWM0();
    uint8_t readPWM1();
    
};
#endif

