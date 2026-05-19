
// Standard includes
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "gpio.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

#include <stdint.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "gpio.h"
#include "spi.h"
#include "prcm.h"
#include "utils.h"
#include "string.h"

// Common interface includes
#include "uart_if.h"
#include "pinmux.h"

#include "Adafruit_SSD1351.h"

#define DC_GPIO_BASE   GPIOA2_BASE
#define DC_PIN         0x40

#define CS_GPIO_BASE   GPIOA1_BASE
#define CS_PIN         0x10

#define RST_GPIO_BASE  GPIOA1_BASE
#define RST_PIN        0x20

#define SCK_GPIO_BASE  GPIOA1_BASE
#define SCK_PIN        0x40        // PIN_05 → GPIO14
#define MOSI_GPIO_BASE GPIOA2_BASE
#define MOSI_PIN       0x02        // PIN_08 → GPIO17

static void spi_send(uint8_t byte) {
    int i;
    for (i = 7; i >= 0; i--) {
        if (byte & (1 << i))
            GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, MOSI_PIN);
        else
            GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, 0);
        GPIOPinWrite(SCK_GPIO_BASE, SCK_PIN, SCK_PIN);  // clock high
        GPIOPinWrite(SCK_GPIO_BASE, SCK_PIN, 0);         // clock low
    }
}

//*****************************************************************************

void writeCommand(uint8_t cmd) {
    GPIOPinWrite(DC_GPIO_BASE,  DC_PIN,  0);       // DC low  = command
    GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  0);       // CS low  = select
    spi_send(cmd);
    GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  CS_PIN);  // CS high = deselect
}

//*****************************************************************************

void writeData(uint8_t data) {
    GPIOPinWrite(DC_GPIO_BASE,  DC_PIN,  DC_PIN);  // DC high = data
    GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  0);       // CS low
    spi_send(data);
    GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  CS_PIN);  // CS high
}

//*****************************************************************************
void Adafruit_Init() {
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

    GPIODirModeSet(DC_GPIO_BASE,   DC_PIN,   GPIO_DIR_MODE_OUT);
    GPIODirModeSet(CS_GPIO_BASE,   CS_PIN,   GPIO_DIR_MODE_OUT);
    GPIODirModeSet(RST_GPIO_BASE,  RST_PIN,  GPIO_DIR_MODE_OUT);
    GPIODirModeSet(SCK_GPIO_BASE,  SCK_PIN,  GPIO_DIR_MODE_OUT);
    GPIODirModeSet(MOSI_GPIO_BASE, MOSI_PIN, GPIO_DIR_MODE_OUT);

    GPIOPinWrite(CS_GPIO_BASE,   CS_PIN,   CS_PIN);
    GPIOPinWrite(DC_GPIO_BASE,   DC_PIN,   DC_PIN);
    GPIOPinWrite(RST_GPIO_BASE,  RST_PIN,  RST_PIN);
    GPIOPinWrite(SCK_GPIO_BASE,  SCK_PIN,  0);
    GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, 0);

    // Hardware reset
    GPIOPinWrite(RST_GPIO_BASE, RST_PIN, 0);
    UtilsDelay(3200000);
    GPIOPinWrite(RST_GPIO_BASE, RST_PIN, RST_PIN);
    UtilsDelay(3200000);

    writeCommand(0xFD); writeData(0x12); // Unlock driver
    writeCommand(0xFD); writeData(0xB1); // Unlock commands

    writeCommand(0xAE);                  // Display off

    writeCommand(0xB3); writeData(0xF1); // Front clock divider
    writeCommand(0xCA); writeData(0x7F); // Mux ratio = 128
    writeCommand(0xA2); writeData(0x00); // Display offset = 0
    writeCommand(0xA1); writeData(0x00); // Start line = 0  (was 32 — bug)
    writeCommand(0xA0); writeData(0x74); // Remap

    writeCommand(0xAB); writeData(0x01); // Function select: internal VDD

    writeCommand(0xB1); writeData(0x32); // Phase length
    writeCommand(0xB4); writeData(0xA0); writeData(0xB5); writeData(0x55); // Set VSL
    writeCommand(0xC1); writeData(0xC8); writeData(0x80); writeData(0xC8); // Contrast
    writeCommand(0xC7); writeData(0x0F); // Master contrast
    writeCommand(0xB6); writeData(0x01); // Second precharge period
    writeCommand(0xBE); writeData(0x05); // VCOMH
    writeCommand(0xA6);                  // Normal display mode

    writeCommand(0xAF);                  // Display on
}

/***********************************/

void goTo(int x, int y) {
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;

  // set x and y coordinate
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(SSD1351WIDTH-1);

  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(SSD1351HEIGHT-1);

  writeCommand(SSD1351_CMD_WRITERAM);
}

unsigned int Color565(unsigned char r, unsigned char g, unsigned char b) {
  unsigned int c;
  c = r >> 3;
  c <<= 6;
  c |= g >> 2;
  c <<= 5;
  c |= b >> 3;

  return c;
}

void fillScreen(unsigned int fillcolor) {
  fillRect(0, 0, SSD1351WIDTH, SSD1351HEIGHT, fillcolor);
}

/**************************************************************************/
/*!
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor)
{
  unsigned int i;

  // Bounds check
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  // Y bounds check
  if (y+h > SSD1351HEIGHT)
  {
    h = SSD1351HEIGHT - y - 1;
  }

  // X bounds check
  if (x+w > SSD1351WIDTH)
  {
    w = SSD1351WIDTH - x - 1;
  }

  // set location
  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  // fill!
  writeCommand(SSD1351_CMD_WRITERAM);

  for (i=0; i < w*h; i++) {
    writeData(fillcolor >> 8);
    writeData(fillcolor);
  }
}

void drawPixel(int x, int y, unsigned int color)
{
  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if ((x < 0) || (y < 0)) return;

  goTo(x, y);

  writeData(color >> 8);
  writeData(color);
}


void  invert(char v) {
   if (v) {
     writeCommand(SSD1351_CMD_INVERTDISPLAY);
   } else {
        writeCommand(SSD1351_CMD_NORMALDISPLAY);
   }
 }


void SetAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    writeCommand(0x15); // Set column address
    writeData(x0);
    writeData(x1);

    writeCommand(0x75); // Set row address
    writeData(y0);
    writeData(y1);

    writeCommand(0x5C); // Write RAM
}

void FillScreen(uint16_t color) {
    int x, y;

    SetAddrWindow(0, 0, 127, 127);

    for (y = 0; y < 128; y++) {
        for (x = 0; x < 128; x++) {
            writeData(color >> 8);   // high byte
            writeData(color & 0xFF); // low byte
        }
    }
}


void VerticalBands(){
    SetAddrWindow(0, 0, 127, 127);

    int y, x;
    unsigned int color;

    for (y = 0; y < 128; y++) {
        for (x = 0; x < 128; x++) {
            if (x % 16 == 0){

                int band = x / 16;

                switch(band) {
                    case 0: color = 0xF000; break;
                    case 1: color = 0xFAA0; break;
                    case 2: color = 0x0AA0; break;
                    case 3: color = 0x0AAF; break;
                    case 4: color = 0x000F; break;
                    case 5: color = 0xF00F; break;
                    case 6: color = 0xFFFF; break;
                    case 7: color = 0xABCD; break;
                }

                writeData(color >> 8);
                writeData(color);
            }
            else{
                writeData(0x0000 >> 8);
                writeData(0x0000);
            }
        }
    }
}

void HorizontalBands(){
    SetAddrWindow(0, 0, 127, 127);

    int y, x;
    unsigned int color;

    for (y = 0; y < 128; y++) {
        for (x = 0; x < 128; x++) {
            if (y % 16 == 0){

                int band = y / 16;

                switch(band) {
                    case 0: color = 0xF000; break;
                    case 1: color = 0xFAA0; break;
                    case 2: color = 0x0AA0; break;
                    case 3: color = 0x0AAF; break;
                    case 4: color = 0x000F; break;
                    case 5: color = 0xF00F; break;
                    case 6: color = 0xFFFF; break;
                    case 7: color = 0xABCD; break;
                }

                writeData(color >> 8);
                writeData(color);
            }
            else{
                writeData(0x0000 >> 8);
                writeData(0x0000);
            }
        }
    }
}

void drawBall(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor)
{
  unsigned int i, j;

  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT))
    return;

  if (y + h > SSD1351HEIGHT)
    h = SSD1351HEIGHT - y;

  if (x + w > SSD1351WIDTH)
    w = SSD1351WIDTH - x;

  // Center and radius
  int cx = w / 2;
  int cy = h / 2;
  int r = (w < h ? w : h) / 2;
  int r2 = r * r;

  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x + w - 1);

  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y + h - 1);

  writeCommand(SSD1351_CMD_WRITERAM);

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++) {

      int dx = i - cx;
      int dy = j - cy;

      if ((dx * dx + dy * dy) <= r2) {
        writeData(fillcolor >> 8);
        writeData(fillcolor);
      } else {
        // background (black)
        writeData(0x00);
        writeData(0x00);
      }
    }
  }
}



