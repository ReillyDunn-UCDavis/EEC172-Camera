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


//*****************************************************************************
//
//! Send a POST to the AWS IoT device shadow reporting which wall was hit.
//!
//! \param wall   String identifying the wall: "top", "bottom", "left", "right"
//!
//! \return 0 on success, negative on failure
//
//*****************************************************************************
static int notify_wall_hit(const char *wall)
{
    // Build the JSON body dynamically so we can embed the wall name
    char body[256];
    snprintf(body, sizeof(body),
             "{\"state\":{\"desired\":{\"var\":\"Ball hit the %s wall!\"}}}\r\n\r\n",
             wall);

    // Open a fresh TLS connection for this POST
    int sock = tls_connect();
    if (sock < 0)
    {
        UART_PRINT("notify_wall_hit: TLS connect failed (%d)\n\r", sock);
        return sock;
    }

    // Assemble the full HTTP request
    char request[512];
    char *p = request;

    strcpy(p, POSTHEADER);  p += strlen(POSTHEADER);
    strcpy(p, HOSTHEADER);  p += strlen(HOSTHEADER);
    strcpy(p, CHEADER);     p += strlen(CHEADER);
    strcpy(p, CTHEADER);    p += strlen(CTHEADER);
    strcpy(p, CLHEADER1);   p += strlen(CLHEADER1);

    char lenStr[16];
    snprintf(lenStr, sizeof(lenStr), "%d", (int)strlen(body));
    strcpy(p, lenStr);      p += strlen(lenStr);

    strcpy(p, CLHEADER2);   p += strlen(CLHEADER2);
    strcpy(p, body);

    UART_PRINT("Sending wall-hit notification (%s)...\n\r", wall);

    // Send
    int ret = sl_Send(sock, request, strlen(request), 0);
    if (ret < 0)
    {
        UART_PRINT("sl_Send failed: %d\n\r", ret);
        sl_Close(sock);
        return ret;
    }

    // Receive (and discard) the HTTP response
    char recvBuf[1460];
    ret = sl_Recv(sock, recvBuf, sizeof(recvBuf) - 1, 0);
    if (ret > 0)
    {
        recvBuf[ret] = '\0';
        UART_PRINT("Response: %s\n\r", recvBuf);
    }

    sl_Close(sock);
    return 0;
}


//*****************************************************************************
//
//! Read a single signed byte from the BMA222 accelerometer via I2C
//
//*****************************************************************************
static int readRegister(unsigned char reg)
{
    unsigned char data = 0;
    I2C_IF_Write(0x18, &reg, 1, 0);
    I2C_IF_Read(0x18, &data, 1);
    return (int)(signed char)data;
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
    UART_PRINT("Ball game + AWS IoT wall notifier starting...\n\r");

    I2C_IF_Open(I2C_MASTER_MODE_FST);

    // Configure the BMA222 accelerometer
    unsigned char resetCmd[2] = {0x14, 0xB6};
    I2C_IF_Write(0x18, resetCmd, 2, 1);
    MAP_UtilsDelay(800000);

    unsigned char rangeCmd[2] = {0x0F, 0x03};   // ±2 g range
    I2C_IF_Write(0x18, rangeCmd, 2, 1);

    unsigned char bwCmd[2] = {0x10, 0x0C};       // 31.25 Hz bandwidth
    I2C_IF_Write(0x18, bwCmd, 2, 1);

    // Display init
    Adafruit_Init();
    FillScreen(0x0000);

    //-------------------------------------------------------------------------
    // Network init
    //-------------------------------------------------------------------------
    g_app_config.host = SERVER_NAME;
    g_app_config.port = GOOGLE_DST_PORT;

    lRetVal = connectToAccessPoint();
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to connect to AP\n\r");
        LOOP_FOREVER();
    }

    lRetVal = set_time();
    if (lRetVal < 0)
    {
        UART_PRINT("Unable to set time\n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Network ready. Starting game loop.\n\r");

    //-------------------------------------------------------------------------
    // Game state
    //-------------------------------------------------------------------------
    int x = 64, y = 64;
    int prev_x = 64, prev_y = 64;
    float vx = 0.0f, vy = 0.0f;

    // Cooldown counter — starts at 0 so the first collision is reported
    int post_cooldown = 0;

    //-------------------------------------------------------------------------
    // Game loop
    //-------------------------------------------------------------------------
    while (1)
    {
        //--- Read accelerometer ---
        int yRaw = readRegister(0x03);
        int xRaw = readRegister(0x05);

        float raw_to_accel = 32.0f;
        float ax = -(xRaw / raw_to_accel);
        float ay = -(yRaw / raw_to_accel);

        //--- Physics ---
        vx += ax;
        vy += ay;

        float friction = 0.9f;
        vx *= friction;
        vy *= friction;

        x += (int)vx;
        y += (int)vy;

        //--- Cooldown tick ---
        if (post_cooldown > 0)
            post_cooldown--;

        //--- Wall collisions ---
        // Each block: clamp position, reverse (& boost) velocity, then POST
        // if the cooldown has expired.

        if (x < 0)
        {
            x = 0;
            vx = -vx * 1.1f;
            if (post_cooldown == 0)
            {
                notify_wall_hit(WALL_LEFT);
                post_cooldown = POST_COOLDOWN_TICKS;
            }
        }
        else if (x > SCREEN_MAX_X)
        {
            x = SCREEN_MAX_X;
            vx = -vx * 1.1f;
            if (post_cooldown == 0)
            {
                notify_wall_hit(WALL_RIGHT);
                post_cooldown = POST_COOLDOWN_TICKS;
            }
        }

        if (y < 0)
        {
            y = 0;
            vy = -vy * 1.1f;
            if (post_cooldown == 0)
            {
                notify_wall_hit(WALL_TOP);
                post_cooldown = POST_COOLDOWN_TICKS;
            }
        }
        else if (y > SCREEN_MAX_Y)
        {
            y = SCREEN_MAX_Y;
            vy = -vy * 1.1f;
            if (post_cooldown == 0)
            {
                notify_wall_hit(WALL_BOTTOM);
                post_cooldown = POST_COOLDOWN_TICKS;
            }
        }

        //--- Draw ---
        drawBall(prev_x, prev_y, BALL_RADIUS, BALL_RADIUS, 0x0000); // erase
        drawBall(x,      y,      BALL_RADIUS, BALL_RADIUS, 0xFFFF); // draw

        prev_x = x;
        prev_y = y;

        MAP_UtilsDelay(40000);
    }
}
