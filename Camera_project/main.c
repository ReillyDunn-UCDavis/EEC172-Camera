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

// Wall names embedded in the JSON payload — one per wall
#define WALL_TOP    "top"
#define WALL_BOTTOM "bottom"
#define WALL_LEFT   "left"
#define WALL_RIGHT  "right"

// Camera GPIO mapping

#define CAM_CS_BASE    GPIOA0_BASE
#define CAM_CS_PIN     0x01    // PIN_50 -> GPIO0

#define CAM_MOSI_BASE  GPIOA1_BASE
#define CAM_MOSI_PIN   0x20    // PIN_45 -> GPIO13

#define CAM_MISO_BASE  GPIOA3_BASE
#define CAM_MISO_PIN   0x40    // PIN_53 -> GPIO30

#define CAM_CLK_BASE   GPIOA0_BASE
#define CAM_CLK_PIN    0x40    // PIN_61 -> GPIO6

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

//static void SPIInit()
//{
//    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,
//                                PRCM_RUN_MODE_CLK);
//
//    MAP_PRCMPeripheralReset(PRCM_GSPI);
//
//    MAP_SPIReset(GSPI_BASE);
//
//    MAP_SPIConfigSetExpClk(
//        GSPI_BASE,
//        MAP_PRCMPeripheralClockGet(PRCM_GSPI),
//        1000000,
//        SPI_MODE_MASTER,
//        SPI_SUB_MODE_0,
//        (SPI_SW_CTRL_CS |
//         SPI_4PIN_MODE |
//         SPI_TURBO_OFF |
//         SPI_CS_ACTIVELOW |
//         SPI_WL_8));
//
////    MAP_SPIFIFOEnable(GSPI_BASE, SPI_TX_FIFO | SPI_RX_FIFO);
////    MAP_SPIFIFOLevelSet(GSPI_BASE, 1, 1);
//
//    MAP_SPIEnable(GSPI_BASE);
//}

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
        MAP_UtilsDelay(5000);


        MAP_GPIOPinWrite(CAM_CLK_BASE, CAM_CLK_PIN, CAM_CLK_PIN);
        MAP_UtilsDelay(5000);

        MAP_GPIOPinWrite(CAM_CLK_BASE, CAM_CLK_PIN, 0);
        MAP_UtilsDelay(5000);

        rx <<= 1;
        if (MAP_GPIOPinRead(CAM_MISO_BASE, CAM_MISO_PIN))
            rx |= 1;

        MAP_GPIOPinWrite(CAM_CLK_BASE, CAM_CLK_PIN, 0);
        MAP_UtilsDelay(5000);
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
    UART_PRINT("Before Cam Select...\n\r");
    cam_select();

    UART_PRINT("Before SPI Transfer...\n\r");
    spi_transfer(addr | 0x80);
    spi_transfer(data);

    UART_PRINT("Before Cam Deselect...\n\r");
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
    //SPIInit();


    cam_deselect();
    InitTerm();
    ClearTerm();
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    arducam_write_reg(0x07, 0x80);   // ARDUCHIP_FIFO, reset
    MAP_UtilsDelay(100000);
    arducam_write_reg(0x07, 0x00);
    MAP_UtilsDelay(100000);

    UART_PRINT("Testing ArduCAM SPI...\n\r");

    arducam_write_reg(0x00, 0x56);
    unsigned char test = arducam_read_reg(0x00);
    UART_PRINT("Wrote 0x56, read back: 0x%02X\n\r", test);

    arducam_write_reg(0x00, 0xAA);
    test = arducam_read_reg(0x00);
    UART_PRINT("Wrote 0xAA, read back: 0x%02X\n\r", test);

    while(1);
}
