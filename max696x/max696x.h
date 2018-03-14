#ifndef _MAX696x_H_
#define _MAX696x_H_
//----------------------------------------------------------------------------------------------
// MAX696x driver
//----------------------------------------------------------------------------------------------
// Based on http://expat.dyndns.org/arduino/library-for-max6960/
//
// Customization of this driver for the user's application (settings etc)
// should be made in conf.h.  Refer to that file for more
// information.
//
// NOTE:
// In order for the display to show correct information, the user
// software must write data to the display memory correctly.
// This is dependent on the mode of operation, the number and
// arrangement of MAX696x drivers ICs and LEDs matrices.
//
// The user should check the MAX696x data sheet for information on how
// the display driver memory maps to pixels.
//
// As an example:
//
// A display uses two MAX696x ICs and four 8x8 monochrome displays
// arranged in order from left to right.  The display consists of 32
// columns (0 to 31,left to right) and 8 rows (0 to 7, top to bottom).
//
// This puts the pixel at the top left at (0,0) and bottom right pixel
// at (31,7)
//
// 1 BIT PER PIXEL
//---------------------
// When in 1 bit per pixel mode there are 4 memory planes (0, 1, 2 & 3)
// of 32 bytes each.
//
// For each memory plane:
//
// Addresses 0x00 (0 dec) to 0x1F (31 dec) correspond directly to the
// pixels in all 8 rows across the length of the display, from left to
// right, ie writing to address 0x00 effects pixels in column 0, and
// writing to address 0x1F effects pixels in column 31.
//
// 8 bit data written to each address maps to the display as follows:
//
//  Bit 0 -> row 0
//  Bit 1 -> row 1
//  Bit 2 -> row 2
//  Bit 3 -> row 3
//  Bit 4 -> row 4
//  Bit 5 -> row 5
//  Bit 6 -> row 6
//  Bit 7 -> row 7
//
// ie there's a direct 1 to 1 mapping of pixels to memory addresses
//
// 2 BITS PER PIXEL
//---------------------
// When in 2 bit per pixel mode there are 2 memory planes
// (0 and 1) of 64 bytes each.
//
// For each memory plane:
//
// Addresses 0x00 (0 dec) to 0x1F (31 dec) correspond to the pixels
// in the first 4 rows across the length of the display from left to right.
// ie, writing to address 0x00 effects pixels in column 0, and writing to
// address 0x1F effects pixels in column 31.
//
// 8 bit data written to each address maps to the  display as follows:
//
//  Bits 0 and 1 -> row 0
//  Bits 2 and 3 -> row 1
//  Bits 4 and 5 -> row 2
//  Bits 6 and 7 -> row 3
//
// Addresses 0x20 (32 dec) to 0x3F (63 dec) correspond to the pixels
// in the last 4 rows across the length of the display from left to right.
// ie, writing to address 0x20 effects pixels in column 0, and writing to
// address 0x3F effects pixels in column 31.
//
// 8 bit data is written to each address and it maps to the
// display as follows:
//
//  Bits 0 and 1 -> row 4
//  Bits 2 and 3 -> row 5
//  Bits 4 and 5 -> row 6
//  Bits 6 and 7 -> row 7
//
// ie the pixels in a display are split across 2 different addresses ranges in the
// display memory, separated by 32 bytes (in this example)
//
// The value of the bit pairs determines the corresponding pixel intensity:
//
//  b00 -> pixel off
//  b01 -> pixel on low
//  b10 -> pixel on mid
//  b11 -> pixel on full
//
//

//----------------------------------------------------------------------------------------------
// Include user application settings

#include "max696x_conf.h"
#include <inttypes.h>
#include <stdbool.h>

typedef uint8_t byte;

#if MAX696x_DEBUG == 1 && !defined(DEBUG)
#undef MAX696x_DEBUG
#endif

#define MAX696X_DISPLAY_WIDTH (NUMBER_OF_DRIVERS_PER_ROW * 16)
#define MAX696X_DISPLAY_HEIGHT ((NUMBER_OF_DRIVER_DEVICES / NUMBER_OF_DRIVERS_PER_ROW) * 8)

#define MAX696X_PIXELBUFFER_SIZE_1BPP (MAX696X_DISPLAY_WIDTH)
#define MAX696X_PIXELBUFFER_SIZE_2BPP (MAX696X_DISPLAY_WIDTH * 2)

//----------------------------------------------------------------------------------------------
// Include all the definitions for the MAX696x now that we've set the user settings.
//
// System settings - DO NOT MODIFY THIS FILE
//
// Used to define the modes of operation of the MAX696x based
#define MODE_MONO (1)
#define MODE_RYG (2)
#define MODE_RGB (3)

// Maximum pixel counts
#define MAX_PIXEL_COUNT_MONO (32768)
#define MAX_PIXEL_COUNT_RGY (16384)

// default recommended oscillator frequency
#define DEFAULT_OSC_FREQ (4194304)
#define MHZ (1000000)

//----------------------------------------------------------------------------------------------
// MAX696x Register Descriptions

//----------------------------------------------------------------------------------------------
// for 16 bit device addressing - register access
// FIELD      BITS   FUNCTION
// R/nW       15     read = 1, write = 0
// AI         14     Address autoincrement enabled = 1
// L/G        13     local = 0, global = 1
// X          12     unused
// ADDRESS    11-8   4-bit address
// DATA       7-0    8-bit register data
#define READ16 (0x80)
#define WRITE16 (0x00)
#define AUTOINC (0x40)
#define LOCAL (0x00)
#define GLOBAL (0x20)

//----------------------------------------------------------------------------------------------
// 16-bit command address values
#define DRIVER_ADDR (0x00)                // Driver address (read only) 0 - 255
#define PIXEL_INSTENSITY_SCALE (0x01)     // Arithmetic for red and green 0 - 3
#define PANEL_INTENSITY (0x02)            // Panel intensity 0 - 255
#define DIGIT_0_INTENSITY (0x03)          // Digit 0 intensity 0 - 255
#define DIGIT_1_INTENSITY (0x04)          // Digit 1 intensity 0 - 255
#define FAULT (0x05)                      // Fault indication
#define GLOBAL_DRV_INDIR_ADDR (0x08)      // Global driver indirect address 0 - 255
#define GLOBAL_DISP_INDIR_ADDR_LSB (0x09) // Global display indirect address LSB 0 - 255
#define GLOBAL_DISP_INDIR_ADDR_MSB (0x0a) // Global display indirect address MSB 0 - 63
#define GLOBAL_PLANE_COUNTER (0x0b)       // Global plane counter 0 - 255
#define GLOBAL_CLEAR_PLANES (0x0c)        // Global clear planes
#define GLOBAL_PANEL_CONFIG (0x0d)        // Global panel configuration 0 - 255
#define GLOBAL_DRIVER_DEVICES (0x0e)      // Global driver devices
#define GLOBAL_DRIVER_ROWS (0x0f)         // Global driver rows

//----------------------------------------------------------------------------------------------
// Pixel intensity scale (PIS)
#define PIS_FULL (0x03)
#define PIS_ARITH_066 (0x02)
#define PIS_GEO_050 (0x02)
#define PIS_ARITH_033 (0x01)
#define PIS_GEO_025 (0x01)
#define PIS_OFF (0x00)

//----------------------------------------------------------------------------------------------
// Panel Intensity Scale Register (PIR)

//----------------------------------------------------------------------------------------------
// Plane Counter Register (PCR)

#if FRAME_AUTO == 1
// Work out the setting for the PCR register for automatic display plane
// cycling at a the specified frame rate

// some bounds checking
// Must set FRAME_RATE_FPS to a value greater than 0
#if FRAME_RATE_FPS == 0
#error FRAME_RATE_FPS must be set greater than 0, please set FRAME_RATE_FPS
#endif

// check the maximum and minimum frequencies of operation for the MAX696x
#if MAX696x_OSC_FREQ > 8500000
#error User setting for MAX696x_OSC_FREQ is greater than max of 8.5 MHz
#endif // MAX696x_OSC_FREQ > 8500000

#if MAX696x_OSC_FREQ < 1000000
#error User setting for MAX696x_OSC_FREQ is lower than min 1.0 MHz
#endif // MAX696x_OSC_FREQ < 1000000

// define the maximum and minimum FPS possible given the user MAX696x_OSC_FREQ setting
#define FRAME_RATE_MAX_FPS ((63 * MAX696x_OSC_FREQ) / DEFAULT_OSC_FREQ)
// Note that FRAME_RATE_MIN_FPS is x 100000 for resolution
#define FRAME_RATE_MIN_FPS (DEFAULT_OSC_FREQ * 100000 / MAX696x_OSC_FREQ / 63)

// Check the user setting for FRAME_RATE_FPS against the maximum and minimum frame rates
// that are possible with the user's MAX696x_OSC_FREQ
#if FRAME_SPEED == 1
// Fast
#if (FRAME_RATE_FPS) > FRAME_RATE_MAX_FPS
#error User setting for FRAME_RATE_FPS is faster than the maximum possible using selected MAX696x_OSC_FREQ
#endif // (FRAME_RATE_FPS) > FRAME_RATE_MAX_FPS

#else
// Slow
#if (100000 / FRAME_RATE_FPS) < FRAME_RATE_MIN_FPS // FRAME_RATE_MIN_FPS is x 100000 for resolution
#error User setting for FRAME_RATE_FPS is slower than the minimum possible using selected MAX696x_OSC_FREQ
#endif // (100000/FRAME_RATE_FPS) < FRAME_RATE_MIN_FPS
#endif // FRAME_SPEED == 1

// Define the value of the PCR register based on FRAME_RATE_FPS and MAX696x_OSC_FREQ
// Note we always must set the LSB of the PCR_RATE register to get a valid rate.
#if FRAME_SPEED == 0
// FAST = 0
#define PCR_RATE ((byte)(DEFAULT_OSC_FREQ / MAX696x_OSC_FREQ / FRAME_RATE_FPS) | 0x41)
#else
// FAST = 1
#define PCR_RATE ((byte)((FRAME_RATE_FPS * DEFAULT_OSC_FREQ) / MAX696x_OSC_FREQ) | 0xC1)
#endif // FRAME_SPEED == 0

#else
// set display for manual plane cycling and set to plane 0
#define PCR_RATE (0x00)
#endif // FRAME_AUTO

//----------------------------------------------------------------------------------------------
// Fault & Device ID (FDI)

#define FDI_OK (0x00)
#define FDI_OPEN (0x01)
#define FDI_SHORT (0x02)
#define FDI_OVERTEMP (0x04)
#define FDI_ID (0x60)
#define FDI_FAULT (0x80)

#define FDI_MAX6960 (0x00)
#define FDI_MAX6961 (0x20)
#define FDI_MAX6962 (0x40)
#define FDI_MAX6963 (0x60)

//----------------------------------------------------------------------------------------------
// Global Clear Planes (GCP) bits
// FIELD     BITS   FUNCTION
// GREEN P3  7      Clear all green plane 3 - colour & 2 bpp only
// GREEN P2  6      Clear all green plane 2 - colour & 2 bpp only
// GREEN P1  5      Clear all green plane 1 - colour & 2 bpp only
// GREEN P0  4      Clear all green plane 0 - colour & 2 bpp only
// RED P3    3      Clear all red plane 3 - 1 bpp only
// RED P2    2      Clear all red plane 2 - 1 bpp only
// RED P1    1      Clear all red plane 1
// RED P0    0      Clear all red plane 0

#define GCP_GRNP3 (0x80)
#define GCP_GRNP2 (0x40)
#define GCP_GRNP1 (0x20)
#define GCP_GRNP0 (0x10)
#define GCP_REDP3 (0x08)
#define GCP_REDP2 (0x04)
#define GCP_REDP1 (0x02)
#define GCP_REDP0 (0x01)
#define GCP_ALL (0xff)

//----------------------------------------------------------------------------------------------
// Global Panel configuration (GPC) bits
// FIELD      BITS   FUNCTION
// PI         7      pixel intensity control, 4 planes 1bit/pixel = 0, 2 planes 2bit/pixel = 1
// C          6      Colour Control mono/RGB = 0, RGY = 1
// F          5      mux flip enabled = 1, disabled = 0
// R          4      ripple sync enabled = 1, disabled = 0
// DP1-DP0    3-2    current display plane (0 - 3)
// IP         1      logic 1 lights LED = 0, logic 0 lights LED = 1
// S          0      shutdown = 0, normal operation = 1
#define GPC_PI (0x80)
#define GPC_C (0x40)
#define GPC_F (0x20)
#define GPC_R (0x10)
#define GPC_DP1 (0x08)
#define GPC_DP0 (0x04)
#define GPC_IP (0x02)
#define GPC_S (0x01)

#define GPC_PIXEL_INTENSITY_1BPP (0x00)
#define GPC_PIXEL_INTENSITY_2BPP (GPC_PI)

#define GPC_DISPLAY_MONO (0x00)
#define GPC_DISPLAY_RGB (0x00)
#define GPC_DISPLAY_RYG (GPC_C)

#define GPC_MUXFLIP_DISABLED (0x00) // note that the MUXFLIP function in the MAX696x has a bug
#define GPC_MUXFLIP_ENABLED (GPC_F) // that causes incorrect display of data so don't enable it

#define GPC_RIPPLESYNC_DISABLED (0x00)
#define GPC_RIPPLESYNC_ENABLED (GPC_R)

#define GPC_PLANE0_1BPP (0x00)
#define GPC_PLANE1_1BPP (GPC_DP0)
#define GPC_PLANE2_1BPP (GPC_DP1)
#define GPC_PLANE3_1BPP (GPC_DP1 | GPC_DP0)

#define GPC_PLANE0_2BPP (GPC_PI)
#define GPC_PLANE1_2BPP (GPC_PI | GPC_DP0)

#define GPC_PIXEL_NORMAL (0x00)
#define GPC_PIXEL_INVERT (GPC_IP)

#define GPC_SHUTDOWN (0x00)
#define GPC_ENABLE (GPC_S)

//----------------------------------------------------------------------------------------------
// for 24-bit direct display addressing, 12-bit address, 1 bit per pixel, mono & RGY
// FIELD      BITS   FUNCTION
// R/nW       23     read = 1, write = 0;
// X          22     unused
// PLANE      21-20  memory plane: 0, 1, 2, 3
// ADDRESS    19-8   12-bit address, 4096 x 8 pixels mono, 2048 x 8 red + 2048 x 8 green RGY
// DATA       7-0    display memory, 1 bit per pixel
#define READ24 (0x80)
#define WRITE24 (0x00)
#define PLANE0_1BPP (0x00)
#define PLANE1_1BPP (0x10)
#define PLANE2_1BPP (0x20)
#define PLANE3_1BPP (0x30)

//----------------------------------------------------------------------------------------------
// for 24-bit diect display addressing, 13 bit address, 2 bit per pixel, mono & RGY
// FIELD      BITS   FUNCTION
// R/nW       23     read = 1, write = 0;
// X          22     unused
// PLANE      21     memory plane: 0, 1
// ADDRESS    20-8   13-bit address, 4096 x 4 pixels mono, 2048 x 4 red + 2048 x 4 green RGY
// DATA       7-0    display memory, 1 bit per pixel
#define PLANE0_2BPP (0x00)
#define PLANE1_2BPP (0x02)

//----------------------------------------------------------------------------------------------
// Function prototypes

class max696x_spi;

class max696x
{
public:
	max696x();

    //-------------------------------------------------------
    // Routine to set up MAX696x display drivers
    //
    // gpc_p: Global Panel Configuration Register
    // pi_p:  Panel Intensity Register
    // pis_p: Pixel Intensity Scale Register
    // gdd_p: Global driver devices register
    // gdr_p: Global driver rows register
    // gplc_p: Global Plane Counter register
    //
    // a gplc_p rate of 0 disables auto plane switching, user software
    // loop must take care of manually switching display planes.
    //
    // example call: 2bpp, mono color display, ripple sync enabled, display enabled, logic 1 lights LED
    //
    // max6960_init((GPC_PIXEL_INTENSITY_2BPP | GPC_RIPPLESYNC_ENABLED | GPC_DISPLAY_MONO | GPC_ENABLE |
    //               GPC_PIXEL_NORMAL),        // Global Panel Config register setting
    //               0x80,                      // Panel Intensity
    //               PIS_FULL,                  // Pixel Intensity Scale
    //               NUMBER_OF_DRIVER_DEVICES,  // Number of driver devices
    //               NUMBER_OF_DRIVERS_PER_ROW, // Number of drivers per row
    //               PCR_RATE);                 // PCR_RATE is computed based on FRAME_AUTO and FRAME_RATE_FPS
    //
    //
    void init(max696x_spi *spi, uint8_t gpc_p, uint8_t pi_p, uint8_t pis_p, uint8_t gdd_p, uint8_t gdr_p, uint8_t gplc_p);

    //-------------------------------------------------------
    // Routine to write to display memory from RAM using indirect addressing
    void mem_update(uint8_t *buffer, unsigned int bufferSize);

    //-------------------------------------------------------
    // Routine to convert 1BPP display data to 2BPP display data
    unsigned char convert_to_2bpp(unsigned char dataIn);

    // ------------------------------------------------------
    // Run a device check
    uint8_t fault_check(void);

    // ------------------------------------------------------
    // Routine to dump the configuration registers of all devices
    // (MAX696x_DEBUG)
    void reg_dump(void);

    //-------------------------------------------------------
    // Routine to check display pixels by turning all LEDs on
    // and then reading the fault register.
    // parameter - none
    // returns - 0 -> for no errors
    //         - 1 -> for 1 or more errors
    uint8_t display_fault_test(void);

    //-------------------------------------------------------
    // Routine to reset the display driver ICs
    void display_reset(void);

    //-------------------------------------------------------
    // Routine to set global intensity
    void set_intensity(uint8_t value);

    //-------------------------------------------------------
    // Routine to set max696x into shutdown mode
    void shutdown();

    //-------------------------------------------------------
    // Routine to set max696x into enabled mode
    void enable();

    //-------------------------------------------------------
    // Routine to switch plane in 2bpp mode
    void switch_active_plane_2bpp(void);

    //-------------------------------------------------------
    // Routine to show the active plane in 2bpp mode
    // (MAX696x_DEBUG)
    void show_active_plane_2bpp(void);

    //-------------------------------------------------------
    // Routine to set pixel in given buffer with given color
    // in 2bpp mode
    void set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color);

protected:
    //-------------------------------------------------------
    // Routine to perform indirect display memory addressing  8-bit write to the MAX696x
    void indirect_mem_wr(volatile uint8_t data);

    //-------------------------------------------------------
    // Routine to write to the command registers in the MAX696x
    void command_wr(volatile uint8_t reg, volatile uint8_t data, volatile uint8_t scope);

    //-------------------------------------------------------
    // Routine to read from the command registers in the MAX696x
    uint8_t command_rd(volatile uint8_t reg, uint8_t scope);

    //-------------------------------------------------------
    // Routine to write 8 bits of data to a 24-bit direct display memory address.
    void direct_mem_wr(volatile uint8_t plane, volatile unsigned int address, volatile uint8_t data);

    //-------------------------------------------------------
    // Routine to read 8 bits of display data back from a 24-bit direct display memory address
    uint8_t direct_mem_rd(volatile uint8_t plane, volatile unsigned int address);

private:
    void io_init();
    void u8g_pb8v2_set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color_index);

    max696x_spi *_spi;

#if MAX696x_DEBUG == 1
    char stringBuffer[64];
#endif
};

#endif // __MAX696x_H__
