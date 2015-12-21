/*
  adp7142.cpp - Arduino library for NXP Semiconductor ADP7142 i2c LED driver
  For implementation details, please take a look at the datasheet http://www.nxp.com/documents/data_sheet/PCA9551.pdf
*/


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "adp7142.h"


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object. Initialize buffers
*/
adp7142::adp7142()
{
  Wire.begin();
  setPSC0(0); // shortest period
  setPSC1(0); // shortest period
  setLS0(OFF);
}


uint8_t adp7142::ledsOff(){
  return _basicWrite(0x30, 0xe0);
}

uint8_t adp7142::setPSC0(uint8_t period){
  return _basicWrite(PSC0, period);
}
uint8_t adp7142::setPSC1(uint8_t period){
  return _basicWrite(PSC1, period);
}
uint8_t adp7142::setPWM0(uint8_t dutyCycle){
  return _basicWrite(PWM0, dutyCycle);
}
uint8_t adp7142::setPWM1(uint8_t dutyCycle){
  return _basicWrite(PWM1, dutyCycle);
}

uint8_t adp7142::setLS0(uint8_t select){
  return _basicWrite(LS0, select);
}
uint8_t adp7142::setLS1(uint8_t select){
  return _basicWrite(LS1, select);
}



/*
Common function for simple write commands
*/
uint8_t adp7142::_basicWrite(byte simpleCommand, byte data) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(simpleCommand);
  Wire.write(data);
  return Wire.endTransmission();
}

/*
Common function for simple write commands
*/
uint8_t adp7142::_basicRead(byte simpleCommand) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(simpleCommand);
  Wire.read();
  return Wire.endTransmission();
}