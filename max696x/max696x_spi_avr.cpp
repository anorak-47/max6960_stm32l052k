
#include "max696x_conf.h"
#include "max696x_spi_avr.h"

max696x_spi_avr::max696x_spi_avr()
{
}

//-------------------------------------------------------
// Routine to set up the SPI interface

void void max696x_spi_avr::spi_init()
{
// Initialize I/O pins used in SPI

#if MAX696x_USE_HW_SPI == 0
    // Initialize the bit bashed SPI clock level to low
    pinMode(DATAOUT, OUTPUT);
    pinMode(DATAIN, INPUT);
    pinMode(SPICLOCK, OUTPUT);
    pinMode(SLAVESELECT, OUTPUT);
    // set the CS pin high to disable device
    digitalWrite(SLAVESELECT, HIGH); // disable device
    digitalWrite(SPICLOCK, LOW);
#else  // MAX696x_USE_HW_SPI
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
#endif // MAX696x_USE_HW_SPI
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

void max696x_spi_avr::spi_send8(volatile uint8_t data)
{
#if MAX696x_USE_HW_SPI == 0
    byte i;
    for (i = 0; i < 8; i++)
    {
        // set the clock pin low
        sw_spi_clock_pin_low();
        // SPDR = data;
        if (data & 0x80)
        {
            DATAOUT_PORT = DATAOUT_PORT | _BV(DATAOUT_BIT);
        }
        else
        {
            DATAOUT_PORT = DATAOUT_PORT & ~_BV(DATAOUT_BIT);
        }
        data = data << 1;
        // set the clock pin high
        SPICLOCK_PORT = SPICLOCK_PORT | _BV(SPICLOCK_BIT);
    }
#else
    // send the data
    SPDR = data;
    // wait for it to be clocked out
    while (!(SPSR & _BV(SPIF)))
    {
    }
#endif // MAX696x_USE_HW_SPI
}

void max696x_spi_avr::spi_send(uint8_t *data, uint8_t length)
{
    // take CS low
    cs_low();
    // write the data
    for (uint8_t i = 0; i < length; i++)
    {
        spi_send8(*(data + i));
    }
    // take CS high
    cs_high();
#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
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

uint8_t max696x_spi_avr::spi_receive8()
{
#if MAX696x_USE_HW_SPI == 0
    byte i;
    byte data = 0;
    for (i = 0; i < 8; i++)
    {
        // set the clock pin low
        sw_spi_clock_pin_low();
        // set the clock pin high
        SPICLOCK_PORT = SPICLOCK_PORT | _BV(SPICLOCK_BIT);
        // shift data up
        data = data << 1;
        // read in value
        if (DATAIN_PORT & _BV(DATAIN_BIT))
        {
            data = data | 0x01;
        }
    }
    return data;
#else
    // send some dummy data
    SPDR = 0x00;
    // wait for it to be clocked out
    while (!(SPSR & _BV(SPIF)))
    {
    }
    // read the data clocked in and return it
    return SPDR;
#endif // MAX696x_USE_HW_SPI
}

//-------------------------------------------------------
// Routine to take CS low

void max696x_spi_avr::cs_low(void)
{
    // take CS low
    SLAVESELECT_PORT__ = SLAVESELECT_PORT & ~_BV(SLAVESELECT_BIT);
}

//-------------------------------------------------------
// Routine to take CS high

void max696x_spi_avr::cs_high(void)
{
    // take CS high
    SLAVESELECT_PORT__ = SLAVESELECT_PORT | _BV(SLAVESELECT_BIT);
}

void max696x_spi_avr::sw_spi_clock_pin_low()
{
#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    SPICLOCK_PORT = SPICLOCK_PORT & ~_BV(SPICLOCK_BIT);
#endif
}
