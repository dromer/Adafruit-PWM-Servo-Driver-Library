/***************************************************
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_PWMServoDriver.h>
#include <TinyWireM.h>

Adafruit_PWMServoDriver::Adafruit_PWMServoDriver(uint8_t addr) {
  _i2caddr = addr;
}

void Adafruit_PWMServoDriver::begin(void) {
 TinyWireM.begin();
 reset();
}


void Adafruit_PWMServoDriver::reset(void) {
 write8(PCA9685_MODE1, 0x0);
}

void Adafruit_PWMServoDriver::setPWMFreq(float freq) {
  //Serial.print("Attempting to set freq ");
  //Serial.println(freq);

  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;
  Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
  uint8_t prescale = floor(prescaleval + 0.5);
  Serial.print("Final pre-scale: "); Serial.println(prescale);

  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(PCA9685_MODE1, newmode); // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  delay(5);
  write8(PCA9685_MODE1, oldmode | 0xa1);  //  This sets the MODE1 register to turn on auto increment.
                                          // This is why the beginTransmission below was not working.
  //  Serial.print("Mode now 0x"); Serial.println(read8(PCA9685_MODE1), HEX);
}

void Adafruit_PWMServoDriver::setPWM(uint8_t num, uint16_t on, uint16_t off) {
  //Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);

  TinyWireM.beginTransmission(_i2caddr);
  TinyWireM.send(LED0_ON_L+4*num);
  TinyWireM.send(on);
  TinyWireM.send(on>>8);
  TinyWireM.send(off);
  TinyWireM.send(off>>8);
  TinyWireM.endTransmission();
}

uint8_t Adafruit_PWMServoDriver::read8(uint8_t addr) {
  TinyWireM.beginTransmission(_i2caddr);
  TinyWireM.send(addr);
  TinyWireM.endTransmission();

  TinyWireM.requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return TinyWireM.receive();
}

void Adafruit_PWMServoDriver::write8(uint8_t addr, uint8_t d) {
  TinyWireM.beginTransmission(_i2caddr);
  TinyWireM.send(addr);
  TinyWireM.send(d);
  TinyWireM.endTransmission();
}
