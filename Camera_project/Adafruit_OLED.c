
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
#define SCK_PIN        0x40
#define MOSI_GPIO_BASE GPIOA2_BASE
#define MOSI_PIN       0x02

static void spi_send(uint8_t byte) {
    int i;
    for (i = 7; i >= 0; i--) {
        if (byte & (1 << i))
            GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, MOSI_PIN);
        else
            GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, 0);
        //MAP_UtilsDelay(100);
        GPIOPinWrite(SCK_GPIO_BASE, SCK_PIN, SCK_PIN);  // clock high
        //MAP_UtilsDelay(100);
        GPIOPinWrite(SCK_GPIO_BASE, SCK_PIN, 0);         // clock low
        //MAP_UtilsDelay(100);
    }
}

//*****************************************************************************

void writeCommand(uint8_t cmd) {
    MAP_GPIOPinWrite(DC_GPIO_BASE,  DC_PIN,  0);       // DC low  = command
    MAP_GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  0);       // CS low  = select
    spi_send(cmd);
    MAP_GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  CS_PIN);  // CS high = deselect
}

//*****************************************************************************

void writeData(uint8_t data) {
    MAP_GPIOPinWrite(DC_GPIO_BASE,  DC_PIN,  DC_PIN);  // DC high = data
    MAP_GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  0);       // CS low
    spi_send(data);
    MAP_GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  CS_PIN);  // CS high
}

//*****************************************************************************
void Adafruit_Init() {
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);

    GPIODirModeSet(DC_GPIO_BASE,  DC_PIN,  GPIO_DIR_MODE_OUT);
    GPIODirModeSet(CS_GPIO_BASE,  CS_PIN,  GPIO_DIR_MODE_OUT);
    GPIODirModeSet(RST_GPIO_BASE, RST_PIN, GPIO_DIR_MODE_OUT);

    GPIOPinWrite(CS_GPIO_BASE,  CS_PIN,  CS_PIN);
    GPIOPinWrite(DC_GPIO_BASE,  DC_PIN,  DC_PIN);
    GPIOPinWrite(RST_GPIO_BASE, RST_PIN, RST_PIN);

    GPIODirModeSet(SCK_GPIO_BASE,  SCK_PIN,  GPIO_DIR_MODE_OUT);
    GPIODirModeSet(MOSI_GPIO_BASE, MOSI_PIN, GPIO_DIR_MODE_OUT);
    GPIOPinWrite(SCK_GPIO_BASE,  SCK_PIN,  0);
    GPIOPinWrite(MOSI_GPIO_BASE, MOSI_PIN, 0);

    GPIOPinWrite(RST_GPIO_BASE, RST_PIN, 0);
    UtilsDelay(3200000);
    GPIOPinWrite(RST_GPIO_BASE, RST_PIN, RST_PIN);
    UtilsDelay(3200000);

    writeCommand(0xAE); // display off

    writeCommand(0xA0); writeData(0x74); // remap
    writeCommand(0xA1); writeData(32); // start line
    writeCommand(0xA2); writeData(0x00); // offset
    writeCommand(0xB1); writeData(0x32); // phase length
    writeCommand(0xB3); writeData(0xF1); // clock
    writeCommand(0xB5); writeData(0x00); // GPIO
    writeCommand(0xB6); writeData(0x0F); // precharge 2
    writeCommand(0xC1); // contrast
    writeData(0xC8); writeData(0x80); writeData(0xC8);
    writeCommand(0xC7); writeData(0x0F); // master contrast
    writeCommand(0xBE); writeData(0x05); // VCOMH

    FillScreen(0x0000);
    writeCommand(0xAF); // display ON
}

void forceAllPixelsOn(void) {
    writeCommand(0xA5);  // All pixels ON — no RAM, no init needed
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

/**************************************************************************/
/*!
    @brief  Draws a filled rectangle using HW acceleration
*/
/**************************************************************************/
static void writeDataRaw(uint8_t data){
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, DC_PIN);
    spi_send(data);
}

void fillRect(unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int fillcolor)
{
  unsigned int i;

  if ((x >= SSD1351WIDTH) || (y >= SSD1351HEIGHT)) return;
  if (y+h > SSD1351HEIGHT) h = SSD1351HEIGHT - y - 1;
  if (x+w > SSD1351WIDTH) w = SSD1351WIDTH - x - 1;

  writeCommand(SSD1351_CMD_SETCOLUMN);
  writeData(x);
  writeData(x+w-1);
  writeCommand(SSD1351_CMD_SETROW);
  writeData(y);
  writeData(y+h-1);
  writeCommand(SSD1351_CMD_WRITERAM);

  MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
  for (i=0; i < w*h; i++) {
    writeDataRaw(fillcolor >> 8);
    writeDataRaw(fillcolor);
  }
  MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);
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

void fillScreenRaw(uint16_t color) {
    int i;
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    // Set column 0-127
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, 0);     // DC low = command
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    spi_send(0x15);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, DC_PIN); // DC high = data
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    spi_send(0);    // start col
    spi_send(127);  // end col
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);

    // Set row 0-127
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, 0);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    spi_send(0x75);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, DC_PIN);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    spi_send(0);    // start row
    spi_send(127);  // end row
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);

    // Write RAM command
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, 0);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    spi_send(0x5C);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);

    // Pixel data — CS stays low for entire burst
    MAP_GPIOPinWrite(DC_GPIO_BASE, DC_PIN, DC_PIN);
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, 0);
    for (i = 0; i < 128 * 128; i++) {
        spi_send(hi);
        spi_send(lo);
    }
    MAP_GPIOPinWrite(CS_GPIO_BASE, CS_PIN, CS_PIN);
}
