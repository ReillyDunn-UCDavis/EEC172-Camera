//*****************************************************************************
//
// main.c - Accelerometer ball game that sends an AWS IoT notification
//          whenever the ball hits a wall.
//
//*****************************************************************************

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Simplelink includes
#include "simplelink.h"

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "spi.h"
#include "gpio.h"
#include "Adafruit_SSD1351.h"

// Common interface includes
#include "pinmux.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"
#include "i2c_if.h"


// Custom includes
#include "utils/network_utils.h"


//*****************************************************************************
// Date/time for TLS certificate validation — keep current!
//*****************************************************************************
#define DATE                19
#define MONTH               5
#define YEAR                2026
#define HOUR                9
#define MINUTE              29
#define SECOND              0

//*****************************************************************************
// AWS IoT connection settings
//*****************************************************************************
#define APPLICATION_NAME      "BallGame"
#define APPLICATION_VERSION   "SQ24"
#define SERVER_NAME           "abktwrys0k548-ats.iot.us-east-2.amazonaws.com"
#define GOOGLE_DST_PORT       8443

#define POSTHEADER "POST /things/Reilly_CC3200Board/shadow HTTP/1.1\r\n"
#define HOSTHEADER "Host: abktwrys0k548-ats.iot.us-east-2.amazonaws.com\r\n"
#define CHEADER    "Connection: Keep-Alive\r\n"
#define CTHEADER   "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1  "Content-Length: "
#define CLHEADER2  "\r\n\r\n"

#define ARDUCHIP_FIFO        0x04
#define ARDUCHIP_TRIG        0x41
#define FIFO_CLEAR_MASK      0x01
#define FIFO_START_MASK      0x02
#define CAP_DONE_MASK        0x08

#define CAM_WIDTH   320
#define CAM_HEIGHT  240
#define OLED_WIDTH  128
#define OLED_HEIGHT 96

#define FIFO_SIZE1           0x42
#define FIFO_SIZE2           0x43
#define FIFO_SIZE3           0x44

#define BURST_FIFO_READ      0x3C

#define ARDUCHIP_MODE      0x02
#define MCU2LCD_MODE       0x00
#define CAM2LCD_MODE       0x01

#define RED_SCALE    1.0f
#define GREEN_SCALE  1.0f
#define BLUE_SCALE   1.0f

// Camera GPIO mapping

#define CAM_CS_BASE    GPIOA0_BASE
#define CAM_CS_PIN     0x01    // PIN_50 -> GPIO0

#define CAM_MOSI_BASE  GPIOA3_BASE
#define CAM_MOSI_PIN   0x80    // PIN_45 -> GPIO31

#define CAM_MISO_BASE  GPIOA3_BASE
#define CAM_MISO_PIN   0x40    // PIN_53 -> GPIO30

#define CAM_CLK_BASE   GPIOA0_BASE
#define CAM_CLK_PIN    0x40    // PIN_61 -> GPIO6

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

//*****************************************************************************
// Cooldown: minimum loop iterations between successive POSTs.
// At ~40 ms per iteration, 25 iterations ≈ 1 second.
//*****************************************************************************
#define POST_COOLDOWN_TICKS  25

//*****************************************************************************
// Game constants
//*****************************************************************************
#define BALL_RADIUS   7
#define SCREEN_MAX_X  120
#define SCREEN_MAX_Y  120

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


//*****************************************************************************
//
//! Board Initialization & Configuration
//
//*****************************************************************************
static void BoardInit(void)
{
#ifndef USE_TIRTOS
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);
    PRCMCC3200MCUInit();
}

static void cam_select(void)
{
    MAP_GPIOPinWrite(CAM_CS_BASE,
                     CAM_CS_PIN,
                     0);
    //MAP_SPICSEnable(GSPI_BASE);
    MAP_UtilsDelay(2000);
}

static void cam_deselect(void)
{
    MAP_UtilsDelay(2000);
    //MAP_SPICSDisable(GSPI_BASE);
    MAP_GPIOPinWrite(CAM_CS_BASE,
                     CAM_CS_PIN,
                     CAM_CS_PIN);
}

static unsigned char spi_transfer(unsigned char data)
{
    unsigned char rx = 0;
    int i;
    for (i = 7; i >= 0; i--){
        MAP_GPIOPinWrite(CAM_MOSI_BASE, CAM_MOSI_PIN,
                         (data & (1 << i)) ? CAM_MOSI_PIN : 0);
        //MAP_UtilsDelay(5000);

        MAP_GPIOPinWrite(CAM_CLK_BASE, CAM_CLK_PIN, CAM_CLK_PIN);
        //MAP_UtilsDelay(5000);

        rx <<= 1;
        if (MAP_GPIOPinRead(CAM_MISO_BASE, CAM_MISO_PIN))
            rx |= 1;

        MAP_GPIOPinWrite(CAM_CLK_BASE, CAM_CLK_PIN, 0);
        //MAP_UtilsDelay(5000);
    }

    return rx;
}

//*****************************************************************************
//
//! Set the device clock so TLS certificates can be validated
//
//*****************************************************************************
static int set_time(void)
{
    long retVal;

    g_time.tm_day  = DATE;
    g_time.tm_mon  = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_hour = HOUR;
    g_time.tm_min  = MINUTE;
    g_time.tm_sec  = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                       SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                       sizeof(SlDateTime), (unsigned char *)(&g_time));
    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

static void arducam_write_reg(unsigned char addr,
                              unsigned char data)
{
    cam_select();

    spi_transfer(addr | 0x80);
    spi_transfer(data);

    cam_deselect();
}

static unsigned char arducam_read_reg(unsigned char addr)
{
    unsigned char value;

    cam_select();

    spi_transfer(addr & 0x7F);
    value = spi_transfer(0x00);

    cam_deselect();

    return value;
}

static void flush_fifo(){
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

static void start_capture(){
    arducam_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

static int capture_done(){
    return (arducam_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK);
}

static unsigned long read_fifo_length(){
    unsigned long len1, len2, len3;

    len1 = arducam_read_reg(FIFO_SIZE1);
    len2 = arducam_read_reg(FIFO_SIZE2);
    len3 = arducam_read_reg(FIFO_SIZE3) & 0x7F;

    return (len3 << 16) | (len2 << 8) | len1;
}

static unsigned char fifo_read_byte(){
    return spi_transfer(0x00);
}

static void oled_begin_image(){
    goTo(0, 0);

    writeCommand(0x15);
    writeData(0);
    writeData(127);

    writeCommand(0x75);
    writeData(16);
    writeData(127);

    writeCommand(0x5C);
}

static void dump_fifo_bytes()
{
    int i;

    cam_select();

    spi_transfer(BURST_FIFO_READ);

    UART_PRINT("FIFO bytes: ");

    for(i = 0; i < 16; i++)
    {
        unsigned char b = fifo_read_byte();
        UART_PRINT("%02X ", b);
    }

    UART_PRINT("\n\r");

    cam_deselect();
}

void fill_screen_color(unsigned char hi, unsigned char lo)
{
    int i;
    goTo(0, 0);
    writeCommand(0x15); writeData(0); writeData(127);
    writeCommand(0x75); writeData(0); writeData(127);
    writeCommand(0x5C);
    for (i = 0; i < 128*128; i++) {
        writeData(hi);
        writeData(lo);
    }
}

static void display_camera_frame()
{
    unsigned long length;

    UART_PRINT("Flushing FIFO...\n\r");

    arducam_write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);

    flush_fifo();
    start_capture();

    while (!capture_done());

    length = read_fifo_length();

    UART_PRINT("FIFO length: %lu\n\r", length);

    if (length == 0 || length > 200000)
    {
        UART_PRINT("Bad FIFO size\n\r");
        return;
    }

    cam_select();
    spi_transfer(BURST_FIFO_READ);

    oled_begin_image();

    int row, col;
    for (row = 0; row < CAM_HEIGHT; row++)
    {
        int out_row = row * OLED_HEIGHT / CAM_HEIGHT;
        int prev_out_row = (row > 0) ? (row-1) * OLED_HEIGHT / CAM_HEIGHT : -1;
        int row_written = (out_row != prev_out_row);

        for (col = 0; col < CAM_WIDTH; col++)
        {
            unsigned char hi = fifo_read_byte();
            unsigned char lo = fifo_read_byte();

            int out_col = col * OLED_WIDTH / CAM_WIDTH;
            int prev_out_col = (col > 0) ? (col-1) * OLED_WIDTH / CAM_WIDTH : -1;

            if (row_written && out_col != prev_out_col)
            {
                unsigned char red = (hi >> 3) & 0x1F;
                unsigned char green = ((hi & 0x07) >> 3) | ((lo >> 5) & 0x07);
                unsigned char blue = lo & 0x1F;

                red = (unsigned char)(red * RED_SCALE > 31 ? 31 : red * RED_SCALE);
                green = (unsigned char)(green * GREEN_SCALE > 63 ? 63 : green * GREEN_SCALE);
                blue = (unsigned char)(blue * BLUE_SCALE  > 31 ? 31 : blue * BLUE_SCALE);

                hi = (red << 3) | (green >> 3);
                lo = (green << 5) | blue;

                writeData(hi);
                writeData(lo);
            }
        }
    }

    cam_deselect();

    UART_PRINT("Frame displayed\n\r");
}

static int wrSensorReg8_8(unsigned char regID,
                           unsigned char regDat)
{
    unsigned char data[2] = {regID, regDat};
    int ret = I2C_IF_Write(0x30, data, 2, 1);
    if (ret != 0)
        UART_PRINT("I2C write failed: reg=0x%02X ret=%d\n\r", regID, ret);

    return ret;
}

struct sensor_reg {
    unsigned char reg;
    unsigned char val;
};

static const struct sensor_reg ov2640_rgb565_regs[] =
{
    // select sensor bank
    {0xff, 0x01},

    // RGB mode
    {0x12, 0x00},

    // clock
    {0x11, 0x01},

    // output format RGB565
    {0xff, 0x00},
    {0xda, 0x08},

    // disable JPEG
    {0xe0, 0x00},

    // QQVGA
    {0xc0, 0x32},
    {0xc1, 0x1e},
    {0x8c, 0x00},

    // scaling
    {0x50, 0x89},

    // DSP
    {0x51, 0xc8},
    {0x52, 0x96},
    {0x53, 0x00},
    {0x54, 0x00},
    {0x55, 0x00},
    {0x57, 0x00},

    {0x00, 0x00}
};

static void ov2640_init_rgb565()
{
    int i = 0;

    UART_PRINT("Initializing OV2640...\n\r");

    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    MAP_UtilsDelay(8000000);

    while (!(ov2640_rgb565_regs[i].reg == 0x00 &&
             ov2640_rgb565_regs[i].val == 0x00))
    {
        wrSensorReg8_8(
            ov2640_rgb565_regs[i].reg,
            ov2640_rgb565_regs[i].val
        );

        MAP_UtilsDelay(10000);

        i++;
    }

    UART_PRINT("OV2640 init done\n\r");
}

//*****************************************************************************
//
//! main
//
//*****************************************************************************
void main(void)
{
    long lRetVal;

    //-------------------------------------------------------------------------
    // Hardware init
    //-------------------------------------------------------------------------
    BoardInit();
    PinMuxConfig();
    InitTerm();
    ClearTerm();
    cam_deselect();
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    ov2640_init_rgb565();

    unsigned char da_val;
    wrSensorReg8_8(0xff, 0x00);  // make sure we're in DSP bank
    // read back 0xDA
    unsigned char reg = 0xda;
    I2C_IF_Write(0x30, &reg, 1, 0);
    I2C_IF_Read(0x30, &da_val, 1);
    UART_PRINT("0xDA = 0x%02X\n\r", da_val);

    arducam_write_reg(ARDUCHIP_MODE, CAM2LCD_MODE);
    unsigned char mode = arducam_read_reg(ARDUCHIP_MODE);

    Adafruit_Init();

    display_camera_frame();

    while(1);

}
