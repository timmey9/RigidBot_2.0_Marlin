/*
  pca9551.cpp - Arduino library for NXP Semiconductor pca9551 i2c LED driver
  For implementation details, please take a look at the datasheet http://www.nxp.com/documents/data_sheet/PCA9551.pdf
*/


/* _____PROJECT INCLUDES_____________________________________________________ */
#include "pca9551.h"


/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object. Initialize buffers
*/
pca9551::pca9551()
{
  Wire.begin();
}


uint8_t pca9551::ledsOff(){
  return writeLS0(0);
}

uint8_t pca9551::ledsOn(){
  return writeLS0(0x55);
}

/*
 * Read and write the period for each PWM timer.
 * The period ranges from 0.026 seconds to 6.74 seconds.
 */
float pca9551::getPeriod0(){
  int period = readPSC0();
  return (float)(period+1)/38;
}
float pca9551::getPeriod1(){
  int period = readPSC1();
  return (float)(period+1)/38;
}
void pca9551::setPeriod0(float period){
  int int_period;
  if(period > 6.74) int_period = 0xff;
  else if(period < 0.026) int_period = 0x00;
  else{
    period = period*38-1;
    int_period = (int)period;
  }
  writePSC0(int_period);
}
void pca9551::setPeriod1(float period){
  int int_period;
  if(period > 6.74) int_period = 0xff;
  else if(period < 0.026) int_period = 0x00;
  else{
    period = period*38-1;
    int_period = (int)period;
  }
  writePSC1(int_period);
}

/*
 * Read and write the duty cycle for each PWM timer.
 * The duty cycle is a float between 0.0 and 1.0 representing the percent of the period that the LED is on. 
 */
float pca9551::getDutyCycle0(){
  int duty_cycle = readPWM0();
  return (float)(duty_cycle/256.0);
}
float pca9551::getDutyCycle1(){
  int duty_cycle = readPWM1();
  return (float)(duty_cycle/256.0);
}
void pca9551::setDutyCycle0(float duty_cycle){
  if(duty_cycle > 1.0) duty_cycle = 1.0;
  if(duty_cycle < 0.0) duty_cycle = 0.0;
  duty_cycle = duty_cycle*255;
  writePWM0((int)duty_cycle);
}
void pca9551::setDutyCycle1(float duty_cycle){
  if(duty_cycle > 1.0) duty_cycle = 1.0;
  if(duty_cycle < 0.0) duty_cycle = 0.0;
  duty_cycle = duty_cycle*255;
  writePWM1((int)duty_cycle);
}

/*
 * Read and write the source for the LEDs.
 * The source determines whether the LED is solid on, solid off, or blinking according to either PSC0 or PSC1.
 */
void pca9551::setLedSources(int red, int green, int blue){
  writeLS0( ((blue<<4)|(green<<2)|red) );
  //writeLS0( (red|RED_MASK)|(green|GREEN_MASK)|(blue|BLUE_MASK) );
}
int pca9551::getLedSources(){
  return readLS0();
}

/* _____PRIVATE FUNCTIONS____________________________________________________ */

/*
 * Write command for each register
 */
uint8_t pca9551::writeLS0(uint8_t output){
  return _basicWrite(LS0, output);
}
uint8_t pca9551::writeLS1(uint8_t output){
  return _basicWrite(LS1, output);
}
uint8_t pca9551::writePSC0(uint8_t period){
  return _basicWrite(PSC0, period);
}
uint8_t pca9551::writePSC1(uint8_t period){
  return _basicWrite(PSC1, period);
}
uint8_t pca9551::writePWM0(uint8_t dutyCycle){
  return _basicWrite(PWM0, dutyCycle);
}
uint8_t pca9551::writePWM1(uint8_t dutyCycle){
  return _basicWrite(PWM1, dutyCycle);
}


/*
 * Read commands for each register
 */
uint8_t pca9551::readInput(){
  return _basicRead(INPUT_);
}
uint8_t pca9551::readLS0(){
  return _basicRead(LS0);
}
uint8_t pca9551::readLS1(){
  return _basicRead(LS1);
}
uint8_t pca9551::readPSC0(){
  return _basicRead(PSC0);
}
uint8_t pca9551::readPSC1(){
  return _basicRead(PSC1);
}
uint8_t pca9551::readPWM0(){
  return _basicRead(PWM0);
}
uint8_t pca9551::readPWM1(){
  return _basicRead(PWM1);
}



/*
 * Simple write command
*/
uint8_t pca9551::_basicWrite(byte command, byte data) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(command);
  Wire.write(data);
  return Wire.endTransmission();
}

/*
 * Simple read command
*/
uint8_t pca9551::_basicRead(byte command) {
  Wire.beginTransmission(ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS,1);
  int var = 0xff;
  if(Wire.available()) var = Wire.read();
  return var;
}
