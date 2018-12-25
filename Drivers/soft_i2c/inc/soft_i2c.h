
#ifndef _SOFT_I2C_H_
#define _SOFT_I2C_H_

#include "stm32f0xx_hal_def.h"

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#else
#error "SSD1306 library was tested only on STM32F1 and STM32F4 MCU families. Please modify ssd1306.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif

// Transmission status error, the return value of endTransmission()
#define SOFTWAREWIRE_NO_ERROR       0
#define SOFTWAREWIRE_BUFFER_FULL    1
#define SOFTWAREWIRE_ADDRESS_NACK   2
#define SOFTWAREWIRE_DATA_NACK      3
#define SOFTWAREWIRE_OTHER          4

#define SOFTWAREWIRE_BUFSIZE 32        // same as buffer size of Arduino Wire library

  void SFT_I2C_Init(void);
  void SFT_I2C_UnInit(void);

  void SFT_I2C_setClock(uint32_t clock);
  void SFT_I2C_beginTransmission(uint8_t address);
  uint8_t SFT_I2C_endTransmission(uint8_t sendStop);
  uint8_t SFT_I2C_requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);
  uint8_t SFT_I2C_write(uint8_t* data, uint8_t size);
  int SFT_I2C_read(void);
  int SFT_I2C_readBytes(uint8_t* buf, uint8_t size);
  void SFT_I2C_setTimeout(long timeout);  // timeout to wait for the I2C bus

  // per object data

  uint8_t _sdaPin;
  uint8_t _sclPin;
  uint8_t _sdaBitMask;
  uint8_t _sclBitMask;

  volatile uint8_t *_sdaPortReg;
  volatile uint8_t *_sclPortReg;
  volatile uint8_t *_sdaDirReg;
  volatile uint8_t *_sclDirReg;
  volatile uint8_t *_sdaPinReg;
  volatile uint8_t *_sclPinReg;

  uint8_t _transmission;      // transmission status, returned by endTransmission(). 0 is no error.
  uint16_t _i2cdelay;         // delay in micro seconds for sda and scl bits.
  uint8_t _pullups;           // using the internal pullups or not
  uint8_t _stretch;           // should code handle clock stretching by the slave or not.
  unsigned long _timeout;     // timeout in ms. When waiting for a clock pulse stretch. 2017, Fix issue #6

  uint8_t rxBuf[SOFTWAREWIRE_BUFSIZE];   // buffer inside this class, a buffer per SoftwareWire.
  uint8_t rxBufPut;           // index to rxBuf, just after the last valid byte.
  uint8_t rxBufGet;           // index to rxBuf, the first new to be read byte.

  // private methods

  void i2c_writebit( uint8_t c );
  uint8_t i2c_readbit(void);
  void i2c_init(void);
  uint8_t i2c_start(void);
  void i2c_repstart(void);
  void i2c_stop(void);
  uint8_t i2c_write(uint8_t c);
  uint8_t i2c_read(uint8_t ack);


#endif /* _SOFT_I2C_H_ */

