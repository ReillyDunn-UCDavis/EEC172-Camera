//*****************************************************************************
// pinmux.c  —  Combined ball-game + AWS IoT notifier
//
// PIN assignments
// ───────────────
// PIN_01  I2C SCL  (BMA222 accelerometer)
// PIN_02  I2C SDA  (BMA222 accelerometer)   ← DO NOT touch after I2C init
// PIN_03  GPIO     OLED CS
// PIN_04  GPIO     OLED RST
// PIN_05  GPIO     OLED SCK
// PIN_08  GPIO     OLED MOSI
// PIN_15  GPIO     OLED DC
// PIN_55  UART0 TX (console)
// PIN_57  UART0 RX (console)
// PIN_64  GPIO     Red LED (MCU_RED_LED_GPIO, used by SSL error reporting)
//
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

void PinMuxConfig(void)
{
    //-------------------------------------------------------------------------
    // Peripheral clocks
    // Enable all clocks up front so every MAP_PinType* call below succeeds.
    //-------------------------------------------------------------------------
    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0,  PRCM_RUN_MODE_CLK);  // I2C
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,   PRCM_RUN_MODE_CLK);  // SPI (OLED)
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);  // UART console
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);  // PIN_50..PIN_64
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);  // PIN_02 group / LED
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);  // PIN_08 (OLED MOSI)
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);  // used by SimpleLink

    //-------------------------------------------------------------------------
    // I2C  (MUST be configured before anything else touches PIN_01 / PIN_02)
    //-------------------------------------------------------------------------
    MAP_PinTypeI2C(PIN_01, PIN_MODE_1);   // SCL
    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);   // SDA

    //-------------------------------------------------------------------------
    // OLED display — bit-banged SPI via GPIO
    //-------------------------------------------------------------------------
    MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);   // CS
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);   // RST
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);   // DC
    MAP_PinTypeGPIO(PIN_05, PIN_MODE_0, false);   // SCK
    MAP_PinTypeGPIO(PIN_08, PIN_MODE_0, false);   // MOSI

    //-------------------------------------------------------------------------
    // UART console
    //-------------------------------------------------------------------------
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);   // TX
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);   // RX

    //-------------------------------------------------------------------------
    // Red LED — used by the SSL/network layer (GPIO_IF_LedOn / ERR_PRINT).
    // PIN_64 lives on GPIOA1; configure as output but do NOT touch PIN_02.
    //-------------------------------------------------------------------------
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);
}
