
#include "max696x_spi_ch.h"
#include "max696x_conf.h"

#if MAX696x_USE_HW_SPI == 0
#error "MAX696x_USE_HW_SPI == 0 not implemented!"
#endif

max696x_spi_ch::max696x_spi_ch()
{

}

//-------------------------------------------------------
// Routine to set up the SPI interface

void max696x_spi_ch::spi_init()
{
	// Initialize I/O pins used in SPI

#ifdef __AVR__
    DATAOUT_PORT |= (_BV(DATAOUT_BIT));
    SPICLOCK_PORT |= (_BV(SPICLOCK_BIT));

    // TODO: move?
    DDRB |= (_BV(PB2));

    // this is the hardware SS pin of the AVR - it
    // needs to be set to output for the spi-interface to work
    // correctly, independently of the CS pin used for the RFM12

    SLAVESELECT_PORT |= (_BV(SLAVESELECT_BIT));
    SLAVESELECT_DDR |= (_BV(SLAVESELECT_BIT));

    DATAIN_DDR &= ~(_BV(DATAIN_BIT));

    byte clr = 0;

    // Configure the hardware SPI interface
    // SPCR = 01011100 0x5C
    // interrupt disabled, spi enabled, msb 1st, master, clk high when idle,
    // sample on trailing (rising) edge of clk, system clock/4 rate
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << CPOL) | (1 << CPHA);
    SPSR &= ~_BV(SPI2X); // Don't use double speed

    clr = SPSR;
    clr = SPDR;
    clr = clr;

    delay(10);
#endif
}

void max696x_spi_ch::io_init()
{
#ifdef __AVR__
#ifdef __ARDUINO__
	digitalWrite(RESET, HIGH);
	pinMode(RESET, OUTPUT);
	pinMode(INTERRUPT, INPUT);
#else
    RESET_PORT |= (_BV(RESET_BIT));
    RESET_DDR |= (_BV(RESET_BIT));
    INTERRUPT_DDR &= ~(_BV(INTERRUPT_BIT));
#endif
#endif
#ifdef __ARM__
#endif
}

//-------------------------------------------------------
// Routine to transmit 8-bits of data via SPI interface
//
// Uses hardware SPI interface or bit-bashed SPI as selected
// by state of MAX696x_USE_HW_SPI ie:
// MAX696x_USE_HW_SPI = 0 => bit bashed SPI
// MAX696x_USE_HW_SPI = 1 => HW SPI
//
// Calling routine controls state of CS for SPI interface
// and final state of clock (LOW).
// Clock initialized to LOW during set up
//
// Parameters are:
// data : 8-bits of data to be sent
//
// Return: nothing

void max696x_spi_ch::spi_send8(uint8_t data)
{
#ifdef __AVR__
    // send the data
    SPDR = data;
    // wait for it to be clocked out
    while (!(SPSR & _BV(SPIF)))
    {
    }
#endif
}

void max696x_spi_ch::spi_send(uint8_t *data, uint8_t length)
{
	cs_low();
	//TODO: write buffer
	cs_high();
}

//-------------------------------------------------------
// Routine to receive 8 bits of data via SPI interface
//
// Uses hardware SPI interface or bit-bashed SPI as selected
// by state of MAX696x_USE_HW_SPI ie:
// MAX696x_USE_HW_SPI = 0 => bit bashed SPI
// MAX696x_USE_HW_SPI = 1 => HW SPI
//
// Calling routine controls state of CS for SPI interface
// and final state of clock (LOW).
// Clock initialized to LOW during set up
//
// Parameters are:
// none
//
// Return: 8-bits of data

uint8_t max696x_spi_ch::spi_receive8()
{
#ifdef __AVR__
    // send some dummy data
    SPDR = 0x00;
    // wait for it to be clocked out
    while (!(SPSR & _BV(SPIF)))
    {
    }
    // read the data clocked in and return it
    return SPDR;
#endif
    return 0;
}

//-------------------------------------------------------
// Routine to take CS low

void max696x_spi_ch::cs_low()
{
#ifdef __AVR__
    // take CS low
    SLAVESELECT_PORT__ = SLAVESELECT_PORT & ~_BV(SLAVESELECT_BIT);
#endif
}

//-------------------------------------------------------
// Routine to take CS high

void max696x_spi_ch::cs_high()
{
#ifdef __AVR__
    // take CS high
    SLAVESELECT_PORT__ = SLAVESELECT_PORT | _BV(SLAVESELECT_BIT);
#endif
}
