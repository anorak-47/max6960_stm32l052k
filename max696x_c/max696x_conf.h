#ifndef _MAX696x_CONF_H_
#define _MAX696x_CONF_H_
//-------------------------------------------------------
// USER CONFIGURATIONS FOR MAX696x LIBRARY FOR ARDUINO
//
// Compatible with following devices:
// - MAX6960, MAX6961, MAX6962, MAX6963
//
// Please refer to data sheets for specific limitations
// and features of each device.
//
// The #defines in this section should be set to match your
// display and hardware configurations
//
// NOTE: IF YOU MAKE ANY CHANGES TO THIS FILE THEN YOU
// MUST DELETE THE FOLLOWING FILE AND RECOMPILE/VERIFY:
//
// max696x/max696x.o

//-------------------------------------------------------
// MAX696x_DEBUG
//
// Set to 1 to enable debug output from the max696x library
// Set to 0 to disable debug output

// Debug mode enables printing to the standard serial port of
// various data and status messages.
//
// It also compiles in extra test strings into program memory
// and consumes extra resources and RAM to run.
//
// Setting this to 1 also enables the following functions:
// max696x::reg_dump()

#define MAX696x_DEBUG (1)

//-------------------------------------------------------
// MAX696x_USE_HW_SPI
//
// Set to 1 to use hardware SPI, otherwise set to 0 to
// use bit-bashed SPI driver on what ever pins you've
// set below

#define MAX696x_USE_HW_SPI (1)

// Define the SPI I/O pins for the Arduino Diecimila
#if MAX696x_USE_HW_SPI == 1
// Default pin assignments for hardware SPI
#define DATAOUT (11)     // MOSI
#define DATAIN (12)      // MISO
#define SPICLOCK (13)    // SCK
#define SLAVESELECT (10) // SS
#else
// Select your own pin assignments for bit-bashed SPI
#define DATAOUT (11)     // MOSI
#define DATAIN (12)      // MISO
#define SPICLOCK (13)    // SCK
#define SLAVESELECT (10) // SS
#endif                   // MAX696x_USE_HW_SPI == 1

// select your own RESET and INTERRUPT pin assignments
#define RESET (9)
#define INTERRUPT (8)

//-------------------------------------------------------
// Port and bit definitions for bit bashed SPI
// these must match up with the definitions above

#define SPICLOCK_PORT (PORTB)
#define SPICLOCK_BIT (PB5)
#define DATAIN_DDR (DDRB)
#define DATAIN_PORT (PINB)
#define DATAIN_BIT (PB4)
#define DATAOUT_PORT (PORTB)
#define DATAOUT_BIT (PB3)
#define SLAVESELECT_DDR (DDRB)
#define SLAVESELECT_PORT (PORTB)
#define SLAVESELECT_BIT (PB1)
#define RESET_DDR (DDRB)
#define RESET_PORT (PORTB)
#define RESET_BIT (PB0)
#define INTERRUPT_DDR (DDRD)
#define INTERRUPT_PORT (PIND)
#define INTERRUPT_BIT (PD2)

//-------------------------------------------------------
// User settings
//
// The #defines in this section should be set to match your
// display hardware configuration

// Set the following to the total number of daisy chained
// devices in the display
#define NUMBER_OF_DRIVER_DEVICES (3)

// Set the following to the number of devices per row
#define NUMBER_OF_DRIVERS_PER_ROW (3)

// MAX696x_OSC_FREQ - MAX696x OSC clock frequency in Hz
#define MAX696x_OSC_FREQ (8000000)

// FRAME_AUTO
// (1) -> enable automatic display plane cycling
// (0) -> disable automatic display plane cycling
#define FRAME_AUTO (0)

// FRAME_SPEED
// (1) -> frame rate 1 FPS or greater ie greater than 1 Hz update rate
// (0) -> frame rate less than 1 FPS ie less than 1 Hz update rate
//
// See also FRAME_RATE_FPS below
#define FRAME_SPEED (1)

// FRAME_RATE_FPS - sets the frame rate of the display use whole numbers here.
//
// Note that the interpretation of this value is dependent on the setting
// of FRAME_SPEED above, eg:
//
// FRAME_SPEED = (1)
// FRAME_RATE_FPS = (25) -> 25 frames per sec, ie new frame every 40 msec
//
// FRAME SPEED = (0)
// FRAME_RATE_FPS = (25) -> 0.04 frames per sec, ie new frame every 25 seconds
//
// Note that actual frame rate will not be exactly what is requested
// here due to rounding, but will be fairly close
#define FRAME_RATE_FPS (12)

// Set the following to the mode of operation either:
// mono : MODE_MONO
// RYG  : MODE_RYG
// RGB  : MODE_RGB
#define MAX_696x_MODE (MODE_MONO)

#endif // _MAX696x_CONF_H_
