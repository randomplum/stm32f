
// Signal differences
// ------------------
//    When the AVR microcontroller is set into hardware I2C mode,
//    the pins follow the I2C specifications for voltage levels and current.
//    For example the current to pull SDA or SCL low is maximum 3mA.
//
//    With the Software I2C, a normal pin is used which can sink/source 40mA for
//    a ATmega328P. That could increase the voltage spikes and could increase
//    interference between sda and scl. The voltage levels are different. The
//    timing of the software I2C is also different.
//
//    In most cases the software I2C should work.
//    With longer wires or with non-matching voltage levels, the result is
//    unpredictable.
//
//
//
// Clock pulse stretching
// ----------------------
//    An I2C Slave could stretch the clock signal by keeping the SCL low.
//    This happens for example when a Slave is an Arduino which can be
//    busy doing other things like handling other interrupts.
//    Adding a check for the clock stretching should make the transmission
//    completely reliable without any loss.
//    Only an Arduino as Slave would do clock stretching, normal devices
//    like sensors and I2C EEPROM don't use clock stretching.
//    The extra check for clock stretching slows down the transfer rate.
//
//    Using millis() instead of micros() is faster.
//    That is why millis() is used for the timeout of the clock pulse
//    stretching.
//
//
//
// Arduino Stream class
// --------------------
//    The Arduino Stream class is used by many Arduino objects.
//    For the I2C bus, the benefits are less obvious.
//    For example the parseInt() function is not used with I2C.
//    At this moment the Stream class is not used.
//
//
// Multiple Slaves with the same I2C address
// -----------------------------------------
//    The SoftwareWire can be declared more than once,
//    to create a number of software i2c busses.
//    That makes it possible to use a number of I2C devices,
//    which have the same I2C address.
//    Every software i2c bus requires 2 pins,
//    and every SoftwareWire object requires 59 bytes at the moment.
//


//  added code to i2c_stop(), since a problem was reported here:
//  http://forum.arduino.cc/index.php?topic=348337.0
//  Added lines have keyword "ADDED1".


// Use the next define to run a i2c_scanner inside the printStatus() function.
// #define ENABLE_I2C_SCANNER


#include "soft_i2c.h"


// Sets SDA low and drives output.
// The SDA may not be HIGH output, so first the output register is cleared
// (clearing internal pullup resistor), after that the SDA is set as output.
#define i2c_sda_lo()                                                           \
	*_sdaPortReg &= ~_sdaBitMask;                                          \
	*_sdaDirReg |= _sdaBitMask;


// sets SCL low and drives output.
// The SCL may not be HIGH output, so first the output register is cleared
// (clearing internal pullup resistor), after that the SCL is set as output.
#define i2c_scl_lo()                                                           \
	*_sclPortReg &= ~_sclBitMask;                                          \
	*_sclDirReg |= _sclBitMask;


// Set SDA high and to input (releases pin) (i.e. change to input, turn on
// pullup). The SDA may not become HIGH output. Therefore the pin is first set
// to input, after that, a pullup resistor is switched on if needed.
#define i2c_sda_hi()                                                           \
	*_sdaDirReg &= ~_sdaBitMask;                                           \
	if (_pullups) {                                                        \
		*_sdaPortReg |= _sdaBitMask;                                   \
	}


// set SCL high and to input (releases pin) (i.e. change to input, turn on
// pullup) The SCL may not become HIGH output. Therefore the pin is first set to
// input, after that, a pullup resistor is switched on if needed.
#define i2c_scl_hi()                                                           \
	*_sclDirReg &= ~_sclBitMask;                                           \
	if (_pullups) {                                                        \
		*_sclPortReg |= _sclBitMask;                                   \
	}


// Read the bit value of the pin
// Note that is the pin can also be read when it is an output.
#define i2c_sda_read() ((uint8_t)(*_sdaPinReg & _sdaBitMask) ? 1 : 0)
#define i2c_scl_read() ((uint8_t)(*_sclPinReg & _sclBitMask) ? 1 : 0)


//
// Release the pins of the Software I2C bus for other use.
// Also the internal pullup resistors are removed.
//
void SFT_I2C_UnInit(void)
{
	// Remember the pullups variable.
	// They will be used again when begin() is called.
	uint8_t pullupsCopy = _pullups;

	_pullups = 0;
	i2c_sda_hi(); // release sda, remove any pullup
	i2c_scl_hi(); // release scl, remove any pullup

	_pullups = pullupsCopy;
}


// begin(void) - enter master mode
// The pins are not changed until begin() is called.
void SFT_I2C_Init(void)
{
	rxBufPut = 0; // nothing in the rxBuf
	rxBufGet = 0;

	i2c_init(); // release the sda and scl (the pullup resistors pull them
		    // high)

		HAL_Delay(2); // 1ms didn't always work.
}

//
// beginTransmission starts the I2C transmission immediate.
//
void SFT_I2C_beginTransmission(uint8_t address)
{
	// Reset error returned by endTransmission.
	_transmission = SOFTWAREWIRE_NO_ERROR;

	// check return value of the start condition.
	// It indicates if the i2c bus is okay.
	if (i2c_start()) {
		uint8_t rc = i2c_write((address << 1)
				       | 0); // The r/w bit is zero for write

		if (rc == 0) // a sda zero from Slave for the 9th bit is ack
		{
			_transmission = SOFTWAREWIRE_NO_ERROR;
		} else {
			_transmission = SOFTWAREWIRE_ADDRESS_NACK;
		}
	} else {
		// If the bus was not okay, the scl or sda didn't work.
		_transmission = SOFTWAREWIRE_OTHER;
	}
}

//
uint8_t SFT_I2C_endTransmission(uint8_t sendStop)
{
	if (sendStop)
		i2c_stop();
	else
		i2c_repstart();

	return (_transmission); // return the transmission status that was set
				// during writing address and data
}


//
// The requestFrom() read the data from the I2C bus and stores it in a buffer.
//
uint8_t SFT_I2C_requestFrom(uint8_t address, uint8_t size, uint8_t sendStop)
{
	uint8_t n = 0; // number of valid received bytes. Start with 0 bytes.

	// The transmission status is set, although it is not returned.
	// Start with the status : no error
	_transmission = SOFTWAREWIRE_NO_ERROR;


	// Clear the RX buffer
	rxBufPut = 0;
	rxBufGet = 0;

	boolean bus_okay = i2c_start();

	if (bus_okay) {
		uint8_t rc = i2c_write((address << 1)
				       | 1); // The r/w bit is '1' to read

		if (rc == 0) // a sda zero from Slave for the 9th bit is ack
		{
			_transmission = SOFTWAREWIRE_NO_ERROR;

			// TODO: check if the Slave returns less bytes than
			// requested.

			for (; n < size; n++) {
				if (n < (size - 1))
					rxBuf[n] =
						i2c_read(true); // read with ack
				else
					rxBuf[n] = i2c_read(
						false); // last byte, read with
							// nack
			}
			rxBufPut = n;
		} else {
			_transmission = SOFTWAREWIRE_ADDRESS_NACK;
		}
	} else {
		// There was a bus error.
		_transmission = SOFTWAREWIRE_OTHER;
	}

	if (sendStop || _transmission != SOFTWAREWIRE_NO_ERROR)
		i2c_stop();
	else
		i2c_repstart();

	return (n);
}

// must be called in:
// slave tx event callback
// or after beginTransmission(address)
uint8_t SFT_I2C_write(uint8_t *data, uint8_t quantity)
{
	// When there was an error during the transmission, no more bytes are
	// transmitted.
	if (_transmission == SOFTWAREWIRE_NO_ERROR) {
		if (i2c_write(data)
		    == 0) // a sda zero from Slave for the 9th bit is ack
		{
			_transmission = SOFTWAREWIRE_NO_ERROR;
		} else {
			_transmission = SOFTWAREWIRE_ADDRESS_NACK;
		}
	}

	return (1); // ignore any errors, return the number of bytes that are
		    // written.
}

//
// The read() reads the buffer, not the I2C bus.
//
static int read(void)
{
	int data;

	if (rxBufPut > rxBufGet) {
		data = rxBuf[rxBufGet];
		rxBufGet++;
	} else {
		data = -1;
	}

	return (data);
}


int SFT_I2C_readBytes(uint8_t *buf, uint8_t size)
{
	int data;
	int n;

	for (n = 0; n < size; n++) {
		data = read();
		if (data == -1)
			break;
		else
			buf[n] = (uint8_t)data;
	}

	return (n);
}


//
// Set the clock speed for the I2C bus.
// Default is 100000 (100kHz).
// A speed of 1Hz is possible with this software I2C library (but not with the
// Arduino Wire library). A speed of 200kHz or higher will remove the delay on
// an Arduino Uno. Without the delay, the functions are free running, using the
// execution timing of the code.
//
void SFT_I2C_setClock(uint32_t clock)
{
	// Tested values with an earlier version of this library.
	//   Value 0 is without delay, the result depends on the microcontroller
	//   and the cpu clock. 0=maxspeed=140kHz (tested on 328P@16MHz)
	//   1=120kHz
	//   2=100kHz (default)
	//   7=50kHz
	//   47=10kHz
	//   97=5kHz
	//   500=1kHz
	//   5000=100Hz
	//   16383=minspeed=30Hz  - delayMicroseconds() max value reference
	//   arduino
	//

	// The _i2cdelay is an uint16_t
	_i2cdelay =
		((F_CPU / 32L)
		 / clock); // The delay in microseconds, '32' is for this code.
	unsigned int delayByCode =
		(F_CPU / 5000000L); // Add some delay for the code, just a guess

	if (_i2cdelay > delayByCode)
		_i2cdelay -= delayByCode;
	else
		_i2cdelay = 0;
}


//
// Set the timeout in milliseconds.
// At this moment, it is only used for timeout when the Slave is stretching the
// clock pulse.
//
void SFT_I2C_setTimeout(long timeout)
{
	// 2017, fix issue #6.
	// A signed long as parameter to be compatible with Arduino libraries.
	// A unsigned long internal to avoid compiler warnings.
	_timeout = (unsigned long)timeout;
}

//
// The i2c_writebit and i2c_readbit could be make "inline", but that
// didn't increase the speed, and the code size increases.
//
// The sda is low after the start condition.
// Therefore the sda is low for the first bit.
//
static void i2c_writebit(uint8_t c)
{
	if (c == 0) {
		i2c_sda_lo();
	} else {
		i2c_sda_hi();
	}

	if (_i2cdelay != 0) // This delay is not needed, but it makes it safer
		delayMicroseconds(_i2cdelay); // This delay is not needed, but
					      // it makes it safer

	i2c_scl_hi(); // clock high: the Slave will read the sda signal

	// Check if clock stretching by the Slave should be detected.
	if (_stretch) {
		// If the Slave was stretching the clock pulse, the clock would
		// not go high immediately. For example if the Slave is an
		// Arduino, that has other interrupts running (for example
		// Serial data).
		unsigned long prevMillis = millis();
		while (i2c_scl_read() == 0) {
			if (millis() - prevMillis >= _timeout)
				break;
		};
	}

	// After the clock stretching, the clock must be high for the normal
	// duration. That is why this delay has still to be done.
	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	i2c_scl_lo();

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);
}


//
static uint8_t i2c_readbit(void)
{
	i2c_sda_hi(); // 'hi' is the same as releasing the line
	i2c_scl_hi();

	// Check if clock stretching by the Slave should be detected.
	if (_stretch) {
		// Wait until the clock is high, the Slave could keep it low for
		// clock stretching.
		unsigned long prevMillis = millis();
		while (i2c_scl_read() == 0) {
			if (millis() - prevMillis >= _timeout)
				break;
		};
	}

	// After the clock stretching, this delay has still be done before
	// reading sda.
	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	uint8_t c = i2c_sda_read();

	i2c_scl_lo();

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	return (c);
}


//
// Initializes the Software I2C.
//
// The original i2c_init sets the SDA and SCL high at the same time.
//
// The code has been changed, since the first data to the software i2c did fail
// sometimes. Changed into SCL high first, with a delay. That would send a STOP
// if the SDA happens to be low. Any Slave that was busy, will detect the STOP.
//
// After both lines are high, the delay is changed into 4 times the normal
// delay. That did reduce the error with the first transmission. It was tested
// with Arduino Uno with clock of 100kHz (_i2cdelay=2).
//
static void i2c_init(void)
{
	i2c_scl_hi();

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	i2c_sda_hi();

	for (uint8_t i = 0; i < 4;
	     i++) // 4 times the normal delay, to claim the bus.
	{
		if (_i2cdelay != 0)
			delayMicroseconds(_i2cdelay);
	}
}


//
// Send a START Condition
//
// The SDA and SCL should already be high.
//
// The SDA and SCL will both be low after this function.
// When writing the address, the Master makes them high.
//
// Return value:
//   true  : software i2c bus is okay.
//   false : failed, some kind of hardware bus error.
//
static uint8_t i2c_start(void)
{
	i2c_sda_hi(); // can perhaps be removed some day ? if the rest of the
		      // code is okay
	i2c_scl_hi(); // can perhaps be removed some day ? if the rest of the
		      // code is okay

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	// Both the sda and scl should be high.
	// If not, there might be a hardware problem with the i2c bus signal
	// lines. This check was added to prevent that a shortcut of sda would
	// be seen as a valid ACK from a i2c Slave.
	uint8_t sda_status = i2c_sda_read();
	uint8_t scl_status = i2c_scl_read();
	if (sda_status == 0 || scl_status == 0) {
		return (0);
	} else {
		i2c_sda_lo();

		if (_i2cdelay != 0)
			delayMicroseconds(_i2cdelay);

		i2c_scl_lo();

		if (_i2cdelay != 0)
			delayMicroseconds(_i2cdelay);
	}
	return (1);
}


//
// Repeated START instead of a STOP
//
// TODO: check if the repeated start actually works.
//
static void i2c_repstart(void)
{
	i2c_sda_hi();
	//  i2c_scl_hi();               // ??????

	i2c_scl_lo(); // force SCL low

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	i2c_sda_hi(); // release SDA

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	i2c_scl_hi(); // release SCL

	// Check if clock stretching by the Slave should be detected.
	if (_stretch) {
		// If the Slave was stretching the clock pulse, the clock would
		// not go high immediately. For example if the Slave is an
		// Arduino, that has other interrupts running (for example
		// Serial data).
		unsigned long prevMillis = millis();
		while (i2c_scl_read() == 0) {
			if (millis() - prevMillis >= _timeout)
				break;
		};
	}

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);
}


// Send a STOP Condition
//
// The stop was not recognized by every chip.
// Some code has been added (with comment "ADDED1"),
// to be sure that the levels are okay with delays in between.
static void i2c_stop(void)
{
	i2c_scl_lo(); // ADDED1, it should already be low.
	i2c_sda_lo();

	// ADDED1, wait to be sure that the slave knows that both are low
	if (_i2cdelay != 0)		      // ADDED1
		delayMicroseconds(_i2cdelay); // ADDED1

	// For a stop, make SCL high wile SDA is still low
	i2c_scl_hi();

	// Check if clock stretching by the Slave should be detected.
	if (_stretch) {
		// Wait until the clock is high, the Slave could keep it low for
		// clock stretching. Clock pulse stretching during a stop
		// condition seems odd, but when the Slave is an Arduino, it
		// might happen.
		unsigned long prevMillis = millis();
		while (i2c_scl_read() == 0) {
			if (millis() - prevMillis >= _timeout)
				break;
		};
	}

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	// complete the STOP by setting SDA high
	i2c_sda_hi();

	// A delay after the STOP for safety.
	// It is not known how fast the next START will happen.
	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);
}


//
// Write a byte to the I2C slave device
// The returned bit is 0 for ACK and 1 for NACK
//
static uint8_t i2c_write(uint8_t c)
{
	for (uint8_t i = 0; i < 8; i++) {
		i2c_writebit(c & 0x80); // highest bit first
		c <<= 1;
	}

	return (i2c_readbit());
}


//
// read a byte from the I2C slave device
//
static uint8_t i2c_read(uint8_t ack)
{
	uint8_t res = 0;

	for (uint8_t i = 0; i < 8; i++) {
		res <<= 1;
		res |= i2c_readbit();
	}

	if (ack) {
		i2c_writebit(0);
	} else {
		i2c_writebit(1);
	}

	if (_i2cdelay != 0)
		delayMicroseconds(_i2cdelay);

	return (res);
}
