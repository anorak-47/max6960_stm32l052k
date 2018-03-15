#ifndef _MAX696x_2bpp_H_
#define _MAX696x_2bpp_H_

#include "max696x.h"
#include <vector>

class max696x_2bpp : public max696x
{
public:
    max696x_2bpp(max696x_spi *spi);
    virtual ~max696x_2bpp();

    void display();
    void clearDisplay();
    virtual void invertDisplay(bool i) override;

    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) override;

    //-------------------------------------------------------
    // Routine to switch plane in 2bpp mode
    void switch_active_plane(void);

    //-------------------------------------------------------
    // Routine to show the active plane in 2bpp mode
    // (MAX696x_DEBUG)
    void show_active_plane(void);

    //-------------------------------------------------------
    // Routine to convert 1BPP display data to 2BPP display data
    unsigned char convert_to_2bpp(unsigned char dataIn);

private:
    void u8g_pb8v2_set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color_index);

    //-------------------------------------------------------
    // Routine to set pixel in given buffer with given color
    // in 2bpp mode
    void set_pixel_2bpp(uint8_t *b, uint8_t x, uint8_t y, uint8_t color);

    // the memory buffer for the LCD
    std::vector<uint8_t> buffer;
};

#endif // __MAX696x_H__
