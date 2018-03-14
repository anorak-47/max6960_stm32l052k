#ifndef _MAX696X_SPI_MBED_H_
#define _MAX696X_SPI_MBED_H_

#include <inttypes.h>

class max696x_spi_mbed
{
public:
	max696x_spi_mbed();
    virtual ~max696x_spi_mbed(){};

    virtual void spi_init() override;

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
    virtual void spi_send8(uint8_t data) override;

    virtual void spi_send(uint8_t* data, uint8_t length) override;

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

    virtual uint8_t spi_receive8() override;

    //-------------------------------------------------------
    // Routine to take CS high

    virtual void cs_high() override;

    //-------------------------------------------------------
    // Routine to take CS low

    virtual void cs_low() override;
};

#endif
