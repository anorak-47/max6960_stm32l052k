#include "max696x_2bpp.h"
#include "../debug.h"
#include <string.h>

#define DISPLAY_PAGE_HEIGHT ((NUMBER_OF_DRIVERS_PER_ROW * 8) / 2)

//------------------------------------------------------------------------------------
// max696x initialization constructor

max696x_2bpp::max696x_2bpp(max696x_spi *spi) : max696x(spi, MAX696X_DISPLAY_WIDTH, MAX696X_DISPLAY_HEIGHT)
{
	buffer.resize(MAX696X_PIXELBUFFER_SIZE_2BPP);
}

//-------------------------------------------------------

max696x_2bpp::~max696x_2bpp()
{
}

//-------------------------------------------------------

void max696x_2bpp::clearDisplay()
{
	std::fill(buffer.begin(),buffer.end(),0);
}

//-------------------------------------------------------

void max696x_2bpp::switch_active_plane(void)
{
    // read the current memory plane being displayed
    uint8_t panel = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
    panel = panel & 0x8C; // mask out bits we're not interested in

    if (panel == GPC_PLANE1_2BPP)
    {
#if MAX696x_DEBUG == 1
        LS_("--> 0");
#endif
        command_wr(GLOBAL_PLANE_COUNTER, 0, GLOBAL);
    }
    else if (panel == GPC_PLANE0_2BPP)
    {
#if MAX696x_DEBUG == 1
        LS_("--> 1");
#endif
        command_wr(GLOBAL_PLANE_COUNTER, 1, GLOBAL);
    }
}

//-------------------------------------------------------

void max696x_2bpp::show_active_plane(void)
{
#if MAX696x_DEBUG == 1
    // read the current memory plane being displayed
    uint8_t panel = command_rd(GLOBAL_PANEL_CONFIG, GLOBAL);
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
// a function to set one pixel

void max696x_2bpp::u8g_pb8v2_set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color_index)
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

void max696x_2bpp::set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color)
{
    if (y >= _rawHeight)
        return;
    if (x >= _rawWidth)
        return;
    u8g_pb8v2_set_pixel_2bpp(b, x, y, color);
}

void max696x_2bpp::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
        return;

    // check rotation, move pixel around if necessary
    switch (getRotation())
    {
    case 1:
        swap(x, y);
        x = _rawWidth - x - 1;
        break;
    case 2:
        x = _rawWidth - x - 1;
        y = _rawHeight - y - 1;
        break;
    case 3:
        swap(x, y);
        y = _rawHeight - y - 1;
        break;
    }

    u8g_pb8v2_set_pixel_2bpp(buffer.data(), x, y, color);
}

// ------------------------------------------------------
// A function to convert 1 Bit per pixel data to 2 Bit per
// pixel data.  This function converts to 2 bpp full intensity
// pixel shading from 1 bpp intensity.
//

unsigned char max696x_2bpp::convert_to_2bpp(unsigned char dataIn)
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
