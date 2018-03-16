#include "max696x.h"
#include "max696x_spi.h"
#include "../debug.h"
#include <string.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#include <util/delay.h>
#define delay_ms _delay_ms
#else
#ifdef __ARM__
#define PROGMEM
#define PGM_P char *
#define strcpy_P strcpy
#define delay_ms wait
#else
#warning "ERROR: unsupported architecture"

#define PROGMEM
#define PGM_P char *
#define strcpy_P strcpy
#define delay_ms(arg)

#endif
#endif

//-------------------------------------------------------------------------------------------------------
// Strings stored in PROGMEM - only required if debug mode used

#if MAX696x_DEBUG == 1

// clang-format off

const char DRIVER_ADDR_STR[] PROGMEM =                "DRIVER address                      (0x00)";
const char PIXEL_INSTENSITY_SCALE_STR[] PROGMEM =     "PIXEL INTENSITY SCALE               (0x01)";
const char PANEL_INTENSITY_STR[] PROGMEM =            "PANEL INTENSITY                     (0x02)";
const char DIGIT_0_INTENSITY_STR[] PROGMEM =          "DIGIT 0 INTENSITY                   (0x03)";
const char DIGIT_1_INTENSITY_STR[] PROGMEM =          "DIGIT 1 INTENSITY                   (0x04)";
const char FAULT_STR[] PROGMEM =                      "FAULT                               (0x05)";
const char UNDEFINED1_STR[] PROGMEM =                 "UNDEFINED                           (0x06)";
const char UNDEFINED2_STR[] PROGMEM =                 "UNDEFINED                           (0x07)";
const char GLOBAL_DRV_INDIR_ADDR_STR[] PROGMEM =      "GLOBAL DRIVER INDIRECT address      (0x08)";
const char GLOBAL_DISP_INDIR_ADDR_LSB_STR[] PROGMEM = "GLOBAL DISPLAY INDIRECT address LSB (0x09)";
const char GLOBAL_DISP_INDIR_ADDR_MSB_STR[] PROGMEM = "GLOBAL DISPLAY INDIRECT address MSB (0x0a)";
const char GLOBAL_PLANE_COUNTER_STR[] PROGMEM =       "GLOBAL PLANE COUNTER                (0x0b)";
const char GLOBAL_CLEAR_PLANES_STR[] PROGMEM =        "CLEAR PLANES                        (0x0c)";
const char GLOBAL_PANEL_CONFIG_STR[] PROGMEM =        "GLOBAL PANEL CONFIG                 (0x0d)";
const char GLOBAL_DRIVER_DEVICES_STR[] PROGMEM =      "GLOBAL DRIVER DEVICES               (0x0e)";
const char GLOBAL_DRIVER_ROWS_STR[] PROGMEM =         "GLOBAL DRIVER ROWS                  (0x0f)";

// clang-format on

PGM_P commandRegisterStringTable[] = {DRIVER_ADDR_STR,
                                      PIXEL_INSTENSITY_SCALE_STR,
                                      PANEL_INTENSITY_STR,
                                      DIGIT_0_INTENSITY_STR,
                                      DIGIT_1_INTENSITY_STR,
                                      FAULT_STR,
                                      UNDEFINED1_STR,
                                      UNDEFINED2_STR,
                                      GLOBAL_DRV_INDIR_ADDR_STR,
                                      GLOBAL_DISP_INDIR_ADDR_LSB_STR,
                                      GLOBAL_DISP_INDIR_ADDR_MSB_STR,
                                      GLOBAL_PLANE_COUNTER_STR,
                                      GLOBAL_CLEAR_PLANES_STR,
                                      GLOBAL_PANEL_CONFIG_STR,
                                      GLOBAL_DRIVER_DEVICES_STR,
                                      GLOBAL_DRIVER_ROWS_STR};
#endif // MAX696x_DEBUG

//------------------------------------------------------------------------------------
// max696x initialization constructor

max696x::max696x(max696x_spi *spi, int16_t w, int16_t h) : Adafruit_GFX(w, h), _spi(spi)
{
#if MAX696x_DEBUG == 1
    memset(stringBuffer, 0x00, 64);
#endif // MAX696x_DEBUG
}

max696x::~max696x()
{
}

//------------------------------------------------------------------------------------
// max696x initialization routine

void max696x::init(uint8_t gpc_p, uint8_t pi_p, uint8_t pis_p, uint8_t gdd_p, uint8_t gdr_p, uint8_t gplc_p)
{
// set up general I/O pins
#if MAX696x_DEBUG == 1
    LS_("Setting up I/O lines...");
#endif // MAX696x_DEBUG == 1

    _spi->io_init();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

// Setting up SPI interface pins
#if MAX696x_DEBUG == 1
    LS_("Setting up SPI interface...");
#endif // MAX696x_DEBUG == 1

    _spi->spi_init();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

    // Reset the display driver ICs
    delay_ms(50);

#if MAX696x_DEBUG == 1
    LS_("Resetting display drivers...");
#endif // MAX696x_DEBUG == 1

    display_reset();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

    delay_ms(100);

// setting up display drivers
#if MAX696x_DEBUG == 1
    LS_("Setting up display drivers...");
#endif // MAX696x_DEBUG == 1

    // Global Panel Configuration Register
    command_wr(GLOBAL_PANEL_CONFIG, gpc_p, GLOBAL);

    // Panel Intensity Register
    command_wr(PANEL_INTENSITY, pi_p, GLOBAL);

    // Pixel Intensity Scale Register
    command_wr(PIXEL_INSTENSITY_SCALE, pis_p, GLOBAL);

    // Global driver devices register
    command_wr(GLOBAL_DRIVER_DEVICES, (gdd_p - 1), GLOBAL);

    // Global driver rows register
    command_wr(GLOBAL_DRIVER_ROWS, (gdr_p - 1), GLOBAL);

    // Global Plane Counter register
    // a gplc rate of 0 disables auto plane switching, user software
    // loop must take care of manually switching display planes.
    if ((gplc_p & 0x3F) == 0x00)
    {
        gplc_p = 0x00;
    }
    command_wr(GLOBAL_PLANE_COUNTER, gplc_p, GLOBAL);

    // Clear Faults on all display drivers
    command_wr(GLOBAL | FAULT, 0x00, GLOBAL);

    // set global driver indirect register to 0
    command_wr(GLOBAL_DRV_INDIR_ADDR, 0, GLOBAL);

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

    delay_ms(450);
}

//-------------------------------------------------------
// Routine to reset the display ICs

void max696x::display_reset(void)
{
#ifdef __AVR__
#ifdef ARDUINO
    digitalWrite(RESET, LOW);
    delay_ms(2.5);
    digitalWrite(RESET, HIGH);
#else
    RESET_PORT &= ~(_BV(RESET_BIT));
    delay_ms(2.5);
    RESET_PORT |= (_BV(RESET_BIT));
#endif
#endif
#ifdef __ARM__

// TODO

#endif
}

//-------------------------------------------------------
// Routine to perform indirect display memory addressing
// 8-bit write to the MAX 6960
//
// Parameters are:
// data : 8-bits of display data
//
// Return : nothing

void max696x::indirect_mem_wr(volatile byte data)
{
    // take CS low
    _spi->cs_low();
    // send the data
    _spi->spi_send8(data);
    // take CS high
    _spi->cs_high();

#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
}

//-------------------------------------------------------
// Routine to write to the command registers in the
// MAX6960
//
// Parameters are:
// reg : 8-bit address of register to write to
// data : 8-bit data to write to the register
// scope : perform global or local write
//         GLOBAL = 0x20
//         LOCAL = 0x00
//
// Return : nothing

void max696x::command_wr(volatile byte reg, volatile byte data, volatile byte scope)
{
    // check range of scope
    if (scope != GLOBAL)
    {
        scope = LOCAL;
    }

    // take CS low
    _spi->cs_low();
    // start the transmission by sending address
    _spi->spi_send8((reg & (~READ16)) | scope);
    // now send data
    _spi->spi_send8(data);
    // take CS high
    _spi->cs_high();

#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
}

//-------------------------------------------------------
// Routine to read from the command registers in the
// MAX6960
//
// Parameters are:
// reg : 8-bit address of register to write to
// scope : perform global or local read
//         GLOBAL = 0x20
//         LOCAL = 0x00
//
// Return : 8-bit data from the selected register

uint8_t max696x::command_rd(volatile byte reg, byte scope)
{
    byte data = 0;

    // check range of scope
    if (scope != GLOBAL)
    {
        scope = LOCAL;
    }

    // take CS low
    _spi->cs_low();
    // start the transmission by sending address
    _spi->spi_send8(reg | READ16 | scope);
    // now send some dummy data
    _spi->spi_send8(0x00);
    // take CS high
    _spi->cs_high();
    // send some more dummy data. read in data at the same time
    data = _spi->spi_receive8();

#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    _spi->sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
    return data;
}

//-------------------------------------------------------
// Routine to write 8 bits of data to a 24-bit direct
// display memory address.
//
// Parameters are:
// plane   : for 1 BPP : PLANE0_1BPP, PLANE1_1BPP, PLANE2_1BPP or PLANE3_1BPP
//         : for 2 BPP : PLANE0_2BPP, PLANE1_2BPP
// address : for 1 BPP : 12-bit address
//         : for 2 BPP : 13-bit address
// data    : 8-bits of data
//
// Return  : nothing

void max696x::direct_mem_wr(volatile byte plane, volatile unsigned int address, volatile byte data)
{
    // take CS low
    _spi->cs_low();
    // start the transmission by sending plane and top 4 bits of address
    _spi->spi_send8(((byte)((address & 0x0F00) >> 8) | plane) & 0xB01111111);
    // send lower 8 bits of address
    _spi->spi_send8((byte)(address & 0x00FF));
    // send 8 bits of data
    _spi->spi_send8((byte)(data & 0x00FF));
    // take CS high
    _spi->cs_high();

#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    _spi->sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
}

//-------------------------------------------------------
// Routine to read 8 bits of display data back from a 24-bit
// direct display memory address
//
// Parameters are:
// plane   : for 1 BPP : PLANE0_1BPP, PLANE1_1BPP, PLANE2_1BPP or PLANE3_1BPP
//         : for 2 BPP : PLANE0_2BPP, PLANE1_2BPP
// address : for 1 BPP : 12-bit address
//         : for 2 BPP : 13-bit address
//
// Return  : 8-bits of display data

byte max696x::direct_mem_rd(volatile byte plane, volatile unsigned int address)
{
    byte data = 0;

    // take CS low
    _spi->cs_low();
    // start the transmission by sending plane and top 4 bits of address
    _spi->spi_send8((byte)((address & 0x0F00) >> 8) | plane | READ24);
    // send lower 8 bits of address
    _spi->spi_send8((byte)(address & 0x00FF));
    // send 8 bits of dummy data
    _spi->spi_send8(0xFF);
    // take CS high
    _spi->cs_high();
    // send some more dummy data. read in data at the same time
    data = _spi->spi_receive8();

#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
    return data;
}

//-------------------------------------------------------
// Routine to write to display memory from RAM using
// indirect addressing

void max696x::mem_update(byte *buffer, unsigned int bufferSize)
{
    unsigned int i = 0;
    // read the current memory plane being displayed
    i = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    i = i & 0x8C; // mask out bits we're not interested in
    // clear the global indirect address register LSB
    command_wr(GLOBAL_DISP_INDIR_ADDR_LSB, 0x00, GLOBAL); // LSB

    if ((i & GPC_PI) == GPC_PIXEL_INTENSITY_1BPP)
    {
        // 1 BPP
        // LS_("1 BPP");
        // set the global indirect address register MSB
        if (i == GPC_PLANE3_1BPP)
        {
            // set indirect address to 0x0000 - 1BPP
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x00, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE0_1BPP)
        {
            // set indirect address to 0x0800 - 1BPP
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x10, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE1_1BPP)
        {
            // set indirect address to 0x1000 - 1BPP
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x20, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE2_1BPP)
        {
            // set indirect address to 0x2000 - 1BPP
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x30, GLOBAL); // MSB
        }

        _spi->spi_send(buffer, bufferSize);

        /*
        // write the data
        for (i = 0; i < bufferSize; i++)
        {
            max696x::indirect_mem_wr(*(buffer + i));
        }
        */
    }
    else
    {
        // 2 BPP
        // LS_("2 BPP");
        if (i == GPC_PLANE1_2BPP)
        {
            // LS_("PLANE 1");
            // set indirect address to 0x0000 - 2BPP
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x00, GLOBAL); // MSB

            _spi->spi_send(buffer, bufferSize);

            /*
            for (i = 0; i < bufferSize; i++)
            {
                max696x::indirect_mem_wr(*(buffer + i));
            }
            */
        }
        else if (i == GPC_PLANE0_2BPP)
        {
            // set indirect address to 0x2000 - 2BPP
            // LS_("PLANE 0");
            command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x20, GLOBAL); // MSB

            _spi->spi_send(buffer, bufferSize);

            /*
            for (i = 0; i < bufferSize; i++)
            {
                max696x::indirect_mem_wr(*(buffer + i));
            }
            */
        }
    }
}

// ------------------------------------------------------
// Routine to dump the configuration registers of all devices

uint8_t max696x::fault_check(void)
{
    uint8_t global_fault = 0;
#if MAX696x_DEBUG == 1
    bool has_fault = display_fault_test();

    if (has_fault)
    {
        global_fault = command_rd(FAULT, GLOBAL);

        for (uint8_t i = 0; i < NUMBER_OF_DRIVER_DEVICES; i++)
        {
            // select device i
            command_wr(GLOBAL_DRV_INDIR_ADDR, i, LOCAL);

            // read back device local registers
            uint8_t data = command_rd(FAULT, LOCAL);

            strcpy_P(stringBuffer, commandRegisterStringTable[FAULT]);
            LV_("%s: 0x%u", stringBuffer, data);

            if (data & FDI_FAULT)
            {
                // Clear fault
                command_wr(FAULT, 0x00, LOCAL);
            }
            delay_ms(10);
        }
    }
#endif
    return global_fault;
}

// ------------------------------------------------------
// Routine to dump the configuration registers of all devices

void max696x::reg_dump(void)
{
#if MAX696x_DEBUG == 1
    byte data;
    byte i;
    byte j;

    LS_("GLOBAL REGS");

    for (i = 0x08; i <= 0x0f; i++)
    {
        data = command_rd(i, GLOBAL);
        // strcpy_P (stringBuffer, (char *)pgm_read_word(&(commandRegisterStringTable[i])));
        strcpy_P(stringBuffer, commandRegisterStringTable[i]);
        LV_("%s: 0x%x / %u", stringBuffer, data, data);
    }

    LS_("");
    LS_("LOCAL REGS");

    for (i = 0; i < NUMBER_OF_DRIVER_DEVICES; i++)
    {
        // select device i
        command_wr(GLOBAL_DRV_INDIR_ADDR, i, LOCAL);

        for (j = 0; j < 6; j++)
        {
            // read back device local registers
            data = command_rd(j, LOCAL);
            // strcpy_P(stringBuffer, (char *)pgm_read_word(&(commandRegisterStringTable[j])));
            strcpy_P(stringBuffer, commandRegisterStringTable[j]);
            LV_("%s: 0x%x / %u", stringBuffer, data, data);

            if ((j == FAULT) && (data & FDI_FAULT))
            {
                // Clear fault
                command_wr(FAULT, 0x00, LOCAL);
            }
        }
        delay_ms(100);
    }
#endif // MAX696x_DEBUG
}

//-------------------------------------------------------
// Routine to check display pixels by turning all LEDs on
// and then reading the fault register.
// parameter - none
// returns - 0 -> for no errors
//         - 1 -> for 1 or more errors

uint8_t max696x::display_fault_test(void)
{
    byte gpc;

    // clear all the display driver memory
    command_wr(GLOBAL_CLEAR_PLANES, GCP_ALL, GLOBAL);

    // store the GPC register setting of the display
    gpc = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);

    // set the display to invert the pixels.
    command_wr(GLOBAL_PANEL_CONFIG, (gpc | GPC_PIXEL_INVERT), GLOBAL);

    // wait a short while
    delay_ms(100);

    // restore original GPC register setting
    command_wr(GLOBAL_PANEL_CONFIG, gpc, GLOBAL);

    // check for faults and return status
    return (command_rd(FAULT, GLOBAL) & FDI_FAULT);
}

//-------------------------------------------------------

void max696x::set_intensity(uint8_t value)
{
    command_wr(PANEL_INTENSITY, value, GLOBAL);
}

//-------------------------------------------------------

void max696x::invertDisplay(bool i)
{
    byte gpc;

    // store the GPC register setting of the display
    gpc = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);

    if (i)
        gpc |= GPC_PIXEL_INVERT;
    else
        gpc &= ~GPC_PIXEL_INVERT;

    // set the display to invert the pixels.
    command_wr(GLOBAL_PANEL_CONFIG, gpc, GLOBAL);
}

//-------------------------------------------------------

void max696x::shutdown()
{
    uint8_t data = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    command_wr(GLOBAL_PANEL_CONFIG, data & ~(GPC_ENABLE), GLOBAL);
}

//-------------------------------------------------------

void max696x::enable()
{
    uint8_t data = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    command_wr(GLOBAL_PANEL_CONFIG, data | GPC_ENABLE, GLOBAL);
}
