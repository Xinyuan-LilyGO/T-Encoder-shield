#include "Arduino.h"
#include <SPI.h>

// 7 led
#define APA102_BRIGHTNESS 5
#define APA102_QUANTITY 7
#define APA102_COL 5
#define APA102_COL_QUANTITY 2 // Quantity per column

char apa102_buf[] = {0x00, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XE1, 0x00, 0x00, 0x00,
                     0XFF, 0xFF, 0XFF, 0XFF};

char row_table[] = {2, 2, 1, 1, 1}; // How many lights are in each column from left to right

char col_table[APA102_COL * APA102_COL_QUANTITY] =
    {
        3,
        7,
        4,
        6,
        5,
        0,
        2,
        0,
        1,
        0,
};

void change_led(uint8_t num, uint8_t brightness, uint32_t rgb)
{
    uint8_t r, g, b = 0;
    uint8_t head = (num * 4) + 4;
    r = (rgb & 0xFF0000) >> 16;
    g = (rgb & 0xFF00) >> 8;
    b = rgb & 0xFF;

    apa102_buf[head] = (brightness & 0x1F) | 0xE0;
    apa102_buf[head + 1] = b;
    apa102_buf[head + 2] = g;
    apa102_buf[head + 3] = r;
}

uint32_t rainbow(uint32_t value)
{
    uint8_t red = 0;   // Red is the top 5 bits of a 16 bit colour value
    uint8_t green = 0; // Green is the middle 6 bits, but only top 5 bits used here
    uint8_t blue = 0;  // Blue is the bottom 5 bits
    uint8_t sector = 0;
    uint8_t amplit = 0;
    uint8_t tmp = value % 192;
    uint32_t res = 0;
    sector = tmp >> 5;
    amplit = tmp & 0x1F;

    switch (sector)
    {
    case 0:
        red = 0x1F;
        green = amplit; // Green ramps up
        blue = 0;
        break;

    case 1:
        red = 0x1F - amplit; // Red ramps down
        green = 0x1F;
        blue = 0;
        break;

    case 2:
        red = 0;
        green = 0x1F;
        blue = amplit; // Blue ramps up
        break;

    case 3:
        red = 0;
        green = 0x1F - amplit; // Green ramps down
        blue = 0x1F;
        break;

    case 4:
        red = amplit; // Red ramps up
        green = 0;
        blue = 0x1F;
        break;

    case 5:
        red = 0x1F;
        green = 0;
        blue = 0x1F - amplit; // Blue ramps down
        break;
    }

    res = red << 8;
    res = res << 8;
    res |= green << 8 | blue;
    return res;
}
void setup()
{
    SPI_begin();
    SPI_beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
}

void loop()
{
    static int j;
    static uint32_t color;
    j++;
    for (int i = 0; i < APA102_QUANTITY; i++)
    {
        color = rainbow(j + (i * 5));
        change_led(i, APA102_BRIGHTNESS, color);
    }

    /* Send APA102_BUF parameter */
    for (int i = 0; i < (sizeof(apa102_buf) / sizeof(char)); i++)
    {
        SPI_transfer(apa102_buf[i]);
    }
    delay(20);
}
