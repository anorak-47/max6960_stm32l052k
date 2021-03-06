#include "../max696x_c/max696x.h"

#include "debug.h"
#include <string.h>

#ifdef __AVR__
#include <avr/pgmspace.h>
#include <util/delay.h>
#define delay _delay_ms
#else
#define PROGMEM
#define PGM_P char *
#define strcpy_P strcpy
#define delay TODO
#endif

#if MAX696x_DEBUG == 1 && !defined(DEBUG)
#undef MAX696x_DEBUG
#endif

#if MAX696x_DEBUG == 1
char stringBuffer[64];
#endif

#define DISPLAY_PAGE_HEIGHT ((NUMBER_OF_DRIVERS_PER_ROW * 8) / 2)

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

//-------------------------------------------------------
// Routine to set up the SPI interface - private

static void spi_init(void)
{
	// Initialize I/O pins used in SPI

#ifdef __AVR__
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
#else
#ifdef __ARM__
// TODO: ARM STM32 SPI stuff
#else
#error "ERROR: unsupported architecture"
#endif
#endif
}

void io_init(void)
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
#else
#error "ERROR: unsupported architecture"
#endif
}

//------------------------------------------------------------------------------------
// max696x initialization routine

void max696x_init(uint8_t gpc_p, uint8_t pi_p, uint8_t pis_p, uint8_t gdd_p, uint8_t gdr_p, uint8_t gplc_p)
{
#if MAX696x_DEBUG == 1
    memset(stringBuffer, 0x00, 64);
#endif // MAX696x_DEBUG

// set up general I/O pins
#if MAX696x_DEBUG == 1
    LS_("Setting up I/O lines...");
#endif // MAX696x_DEBUG == 1

    io_init();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

// Setting up SPI interface pins
#if MAX696x_DEBUG == 1
    LS_("Setting up SPI interface...");
#endif // MAX696x_DEBUG == 1

    spi_init();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

    // Reset the display driver ICs
    delay(50);

#if MAX696x_DEBUG == 1
    LS_("Resetting display drivers...");
#endif // MAX696x_DEBUG == 1

    max696x_display_reset();

#if MAX696x_DEBUG == 1
    LS_(" done");
#endif // MAX696x_DEBUG == 1

    delay(100);

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

    delay(450);
}

//-------------------------------------------------------
// Routine to reset the display ICs

void max696x_display_reset(void)
{
#ifdef __AVR__
#ifdef ARDUINO

    digitalWrite(RESET, LOW);
    delay(2.5);
    digitalWrite(RESET, HIGH);

#else

    RESET_PORT &= ~(_BV(RESET_BIT));
    delay(2.5);
    RESET_PORT |= (_BV(RESET_BIT));

#endif
#else

#ifdef __ARM__

// TODO

#else
#error "ERROR: unsupported architecture"
#endif

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

static void spi_send8(volatile byte data)
{
#ifdef __AVR__
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
#else

#ifdef __ARM__

// TODO

#else
#error "ERROR: unsupported architecture"
#endif

#endif
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

static byte spi_receive8(void)
{
#ifdef __AVR__
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
#else

#ifdef __ARM__

// TODO

#else
#error "ERROR: unsupported architecture"
#endif

#endif

    return 0;
}

//-------------------------------------------------------
// Routine to take CS low

inline void cs_low(void)
{
#ifdef __AVR__
    // take CS low
    SLAVESELECT_PORT__ = SLAVESELECT_PORT & ~_BV(SLAVESELECT_BIT);
#else

#endif
}

//-------------------------------------------------------
// Routine to take CS high

inline void cs_high(void)
{
#ifdef __AVR__
    // take CS high
    SLAVESELECT_PORT__ = SLAVESELECT_PORT | _BV(SLAVESELECT_BIT);
#else

#endif
}

inline void sw_spi_clock_pin_low(void)
{
#if MAX696x_USE_HW_SPI == 0
#ifdef __AVR__
    // set the clock pin low
    SPICLOCK_PORT = SPICLOCK_PORT & ~_BV(SPICLOCK_BIT);
#else

#errror "implement sw_spi_clock_pin_low"

#endif
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

void max696x_indirect_mem_wr(volatile byte data)
{
    // take CS low
    cs_low();
    // send the data
    spi_send8(data);
    // take CS high
    cs_high();
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

void max696x_command_wr(volatile byte reg, volatile byte data, volatile byte scope)
{
    // check range of scope
    if (scope != GLOBAL)
    {
        scope = LOCAL;
    }

    // take CS low
    cs_low();
    // start the transmission by sending address
    spi_send8((reg & (~READ16)) | scope);
    // now send data
    spi_send8(data);
    // take CS high
    cs_high();
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

uint8_t max696x_command_rd(volatile byte reg, byte scope)
{
    byte data = 0;

    // check range of scope
    if (scope != GLOBAL)
    {
        scope = LOCAL;
    }

    // take CS low
    cs_low();
    // start the transmission by sending address
    spi_send8(reg | READ16 | scope);
    // now send some dummy data
    spi_send8(0x00);
    // take CS high
    cs_high();
    // send some more dummy data. read in data at the same time
    data = spi_receive8();
#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
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

void max696x_direct_mem_wr(volatile byte plane, volatile unsigned int address, volatile byte data)
{
    // take CS low
    cs_low();
    // start the transmission by sending plane and top 4 bits of address
    spi_send8(((byte)((address & 0x0F00) >> 8) | plane) & 0xB01111111);
    // send lower 8 bits of address
    spi_send8((byte)(address & 0x00FF));
    // send 8 bits of data
    spi_send8((byte)(data & 0x00FF));
    // take CS high
    cs_high();
#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
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

byte max696x_direct_mem_rd(volatile byte plane, volatile unsigned int address)
{
    byte data = 0;

    // take CS low
    cs_low();
    // start the transmission by sending plane and top 4 bits of address
    spi_send8((byte)((address & 0x0F00) >> 8) | plane | READ24);
    // send lower 8 bits of address
    spi_send8((byte)(address & 0x00FF));
    // send 8 bits of dummy data
    spi_send8(0xFF);
    // take CS high
    cs_high();
    // send some more dummy data. read in data at the same time
    data = spi_receive8();
#if MAX696x_USE_HW_SPI == 0
    // set the clock pin low
    sw_spi_clock_pin_low();
#endif // MAX696x_USE_HW_SPI
    return data;
}

//-------------------------------------------------------
// Routine to write to display memory from RAM using
// indirect addressing

void max696x_mem_update(byte *buffer, unsigned int bufferSize)
{
    unsigned int i = 0;
    // read the current memory plane being displayed
    i = max696x_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    i = i & 0x8C; // mask out bits we're not interested in
    // clear the global indirect address register LSB
    max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_LSB, 0x00, GLOBAL); // LSB

    if ((i & GPC_PI) == GPC_PIXEL_INTENSITY_1BPP)
    {
        // 1 BPP
        // LS_("1 BPP");
        // set the global indirect address register MSB
        if (i == GPC_PLANE3_1BPP)
        {
            // set indirect address to 0x0000 - 1BPP
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x00, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE0_1BPP)
        {
            // set indirect address to 0x0800 - 1BPP
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x10, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE1_1BPP)
        {
            // set indirect address to 0x1000 - 1BPP
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x20, GLOBAL); // MSB
        }
        else if (i == GPC_PLANE2_1BPP)
        {
            // set indirect address to 0x2000 - 1BPP
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x30, GLOBAL); // MSB
        }
        // write the data
        for (i = 0; i < bufferSize; i++)
        {
            max696x_indirect_mem_wr(*(buffer + i));
        }
    }
    else
    {
        // 2 BPP
        // LS_("2 BPP");
        if (i == GPC_PLANE1_2BPP)
        {
            // LS_("PLANE 1");
            // set indirect address to 0x0000 - 2BPP
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x00, GLOBAL); // MSB
            for (i = 0; i < bufferSize; i++)
            {
                max696x_indirect_mem_wr(*(buffer + i));
            }
        }
        else if (i == GPC_PLANE0_2BPP)
        {
            // set indirect address to 0x2000 - 2BPP
            // LS_("PLANE 0");
            max696x_command_wr(GLOBAL_DISP_INDIR_ADDR_MSB, 0x20, GLOBAL); // MSB
            for (i = 0; i < bufferSize; i++)
            {
                max696x_indirect_mem_wr(*(buffer + i));
            }
        }
    }
}

// ------------------------------------------------------
// A function to convert 1 Bit per pixel data to 2 Bit per
// pixel data.  This function converts to 2 bpp full intensity
// pixel shading from 1 bpp intensity.
//

unsigned char max696x_convert_to_2bpp(unsigned char dataIn)
{
    unsigned char dataOut = 0;
    switch (dataIn)
    {
    case 0x01:
        dataOut = 0x03;
        break;
    case 0x02:
        dataOut = 0x0c;
        break;
    case 0x03:
        dataOut = 0x0f;
        break;
    case 0x04:
        dataOut = 0x30;
        break;
    case 0x05:
        dataOut = 0x33;
        break;
    case 0x06:
        dataOut = 0x3c;
        break;
    case 0x07:
        dataOut = 0x3f;
        break;
    case 0x08:
        dataOut = 0xc0;
        break;
    case 0x09:
        dataOut = 0xc3;
        break;
    case 0x0A:
        dataOut = 0xcc;
        break;
    case 0x0B:
        dataOut = 0xcf;
        break;
    case 0x0C:
        dataOut = 0xf0;
        break;
    case 0x0D:
        dataOut = 0xf3;
        break;
    case 0x0E:
        dataOut = 0xfc;
        break;
    case 0x0F:
        dataOut = 0xff;
        break;
    default:
        break;
    }
    return dataOut;
}

// ------------------------------------------------------
// Routine to dump the configuration registers of all devices

uint8_t max696x_fault_check(void)
{
    uint8_t global_fault = 0;
    bool has_fault = max696x_display_test();

    if (has_fault)
    {
        global_fault = max696x_command_rd(FAULT, GLOBAL);

        for (uint8_t i = 0; i < NUMBER_OF_DRIVER_DEVICES; i++)
        {
            // select device i
            max696x_command_wr(GLOBAL_DRV_INDIR_ADDR, i, LOCAL);

            // read back device local registers
            uint8_t data = max696x_command_rd(FAULT, LOCAL);

            strcpy_P(stringBuffer, commandRegisterStringTable[FAULT]);
            LV_("%s: 0x%u", stringBuffer, data);

            if (data & FDI_FAULT)
            {
                // Clear fault
                max696x_command_wr(FAULT, 0x00, LOCAL);
            }
            delay(10);
        }
    }

    return global_fault;
}

// ------------------------------------------------------
// Routine to dump the configuration registers of all devices

void max696x_reg_dump(void)
{
#if MAX696x_DEBUG == 1
    byte data;
    byte i;
    byte j;

    LS_("GLOBAL REGS");

    for (i = 0x08; i <= 0x0f; i++)
    {
        data = max696x_command_rd(i, GLOBAL);
        // strcpy_P (stringBuffer, (char *)pgm_read_word(&(commandRegisterStringTable[i])));
        strcpy_P(stringBuffer, commandRegisterStringTable[i]);
        LV_("%s: 0x%x / %u", stringBuffer, data, data);
    }

    LS_("");
    LS_("LOCAL REGS");

    for (i = 0; i < NUMBER_OF_DRIVER_DEVICES; i++)
    {
        // select device i
        max696x_command_wr(GLOBAL_DRV_INDIR_ADDR, i, LOCAL);

        for (j = 0; j < 6; j++)
        {
            // read back device local registers
            data = max696x_command_rd(j, LOCAL);
            // strcpy_P(stringBuffer, (char *)pgm_read_word(&(commandRegisterStringTable[j])));
            strcpy_P(stringBuffer, commandRegisterStringTable[j]);
            LV_("%s: 0x%x / %u", stringBuffer, data, data);

            if ((j == FAULT) && (data & FDI_FAULT))
            {
                // Clear fault
                max696x_command_wr(FAULT, 0x00, LOCAL);
            }
        }
        delay(100);
    }
#endif // MAX696x_DEBUG
}

//-------------------------------------------------------
// Routine to check display pixels by turning all LEDs on
// and then reading the fault register.
// parameter - none
// returns - 0 -> for no errors
//         - 1 -> for 1 or more errors

uint8_t max696x_display_fault_test(void)
{
    byte gpc;

    // clear all the display driver memory
    max696x_command_wr(GLOBAL_CLEAR_PLANES, GCP_ALL, GLOBAL);

    // store the GPC register setting of the display
    gpc = max696x_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);

    // set the display to invert the pixels.
    max696x_command_wr(GLOBAL_PANEL_CONFIG, (gpc | GPC_PIXEL_INVERT), GLOBAL);

    // wait a short while
    delay(100);

    // restore original GPC register setting
    max696x_command_wr(GLOBAL_PANEL_CONFIG, gpc, GLOBAL);

    // check for faults and return status
    return (max696x_command_rd(FAULT, GLOBAL) & FDI_FAULT);
}

//-------------------------------------------------------

void max696x_set_intensity(uint8_t value)
{
    max6960_command_wr(PANEL_INTENSITY, value, GLOBAL);
}

//-------------------------------------------------------

void max696x_shutdown()
{
    uint8_t data = max6960_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    max6960_command_wr(GLOBAL_PANEL_CONFIG, data & ~(GPC_ENABLE), GLOBAL);
}

//-------------------------------------------------------

void max696x_enable()
{
    uint8_t data = max6960_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    max6960_command_wr(GLOBAL_PANEL_CONFIG, data | GPC_ENABLE, GLOBAL);
}

//-------------------------------------------------------

void max696x_switch_active_plane_2bpp(void)
{
    // read the current memory plane being displayed
    uint8_t panel = max6960_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    panel = panel & 0x8C; // mask out bits we're not interested in

    if (panel == GPC_PLANE1_2BPP)
    {
#if MAX696x_DEBUG == 1
        LS_("--> 0");
#endif
        max6960_command_wr(GLOBAL_PLANE_COUNTER, 0, GLOBAL);
    }
    else if (panel == GPC_PLANE0_2BPP)
    {
#if MAX696x_DEBUG == 1
        LS_("--> 1");
#endif
        max6960_command_wr(GLOBAL_PLANE_COUNTER, 1, GLOBAL);
    }
}

//-------------------------------------------------------

void max696x_show_active_plane_2bpp(void)
{
#if MAX696x_DEBUG == 1
    // read the current memory plane being displayed
    uint8_t panel = max6960_command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    panel = panel & 0x8C; // mask out bits we're not interested in

    if (panel == GPC_PLANE1_2BPP)
    {
        LS_("active plane: 1");
    }
    else if (panel == GPC_PLANE0_2BPP)
    {
        LS_("active plane: 0");
    }
#endif
}

//-------------------------------------------------------

void u8g_pb8v2_set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color_index)
{
    register uint8_t mask;
    uint8_t *ptr = b;
    // y -= b->p.page_y0;
    // TODO: this limited to a display height of 8 pixel!
    // if (y > 3)
    if (y >= DISPLAY_PAGE_HEIGHT)
        ptr += MAX696X_DISPLAY_WIDTH;
    mask = 0x03;
    y &= 0x03;
    y <<= 1;
    mask <<= y;
    mask ^= 0xff;
    color_index &= 3;
    color_index <<= y;
    ptr += x;
    *ptr &= mask;
    *ptr |= color_index;
}

void max696x_set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color)
{
    if (y >= MAX696X_DISPLAY_HEIGHT)
        return;
    if (x >= MAX696X_DISPLAY_WIDTH)
        return;
    u8g_pb8v2_set_pixel_2bpp(b, x, y, color);
}
