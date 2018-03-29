/*
 * SysConfiguration.c
 *
 *  Created on: Mar 5, 2018
 *      Author:
 */

#include "board.h"
#include "SysConfiguration.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "SPI_driver.h"
#include "LCDNokia5110.h"
#include "semphr.h"
#include "NVIC.h"
#include "fsl_debug_console.h"
#include "UART_personal.h"
#include "I2C_no_bloqueante.h"

//////////////////**user types definitions*////////////////////////////
typedef enum {
    PTA1 = 1, PTA2
} PORTA_buttons_pins_t;

typedef enum {
    PTB9 = 9, PTB23 = 23
} PORTB_buttons_pins_t;

typedef enum {
    PTC2 = 2, PTC3, PTC4, PTC5, PTC16 = 16, PTC17
} PORTC_buttons_pins_t;

typedef enum {
    PTB3_RX = 3, PTB4_TX
} PORTC_UART1_pins_t;

//////////////////**function prototypes*////////////////////////////
void SYSconfig_ButtonsConfiguration();
void SYSconfig_SPIConfiguration();
void SYSconfig_UARTConfiguration();
void SYSconfig_I2CConfiguration();








//////////////////**mechanisms definitions*/////////////////////////
void SystemConfiguration(void* args) {
    for (;;)
    {

        /**ports A,B,C clock will be enabled, as pins from all those will be used*/
        CLOCK_EnableClock(kCLOCK_PortA);
        CLOCK_EnableClock(kCLOCK_PortB);
        CLOCK_EnableClock(kCLOCK_PortC);
        /**modules configuration*/
        SYSconfig_ButtonsConfiguration(); /**buttons configuration*/
        SYSconfig_UARTConfiguration(); /**UART module configuration*/

        SYSconfig_SPIConfiguration(); /**SPI module configuration (including device initialization)*/
        SYSconfig_I2CConfiguration(); /**I2C module configuration*/

        //vTaskSuspend(NULL); /**the function auto suspends itself, as it won't be used again*/
        vTaskDelete(NULL);
    }
}

void SYSconfig_ButtonsConfiguration() { /**this function responsibility is to configure everything related to the buttons which will be used*/
    /**button configuration definition*/
    port_pin_config_t button_configuration = { kPORT_PullDown,
        kPORT_SlowSlewRate, kPORT_PassiveFilterEnable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
    /**button pins configuration*/
    PORT_SetPinConfig(PORTA, PTA2, &button_configuration); /**B2 configuration*/
    PORT_SetPinConfig(PORTB, PTB23, &button_configuration);/**B3 configuration*/
    PORT_SetPinConfig(PORTA, PTA1, &button_configuration); /**B4 configuration*/
    PORT_SetPinConfig(PORTB, PTB9, &button_configuration); /**B5 configuration*/

    /**interrupts configuration*/
    PORT_SetPinInterruptConfig(PORTA, PTA2, kPORT_InterruptFallingEdge); /**B2 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTB, PTB23, kPORT_InterruptFallingEdge); /**B3 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTA, PTA1, kPORT_InterruptFallingEdge); /**B4 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTB, PTB9, kPORT_InterruptFallingEdge); /**B5 configured to interrupt when a logic 1 is read*/

    NVIC_enableInterruptAndPriotity(PORTA_IRQn, 6);
    NVIC_enableInterruptAndPriotity(PORTB_IRQn, 6);
    NVIC_enableInterruptAndPriotity(PORTC_IRQn, 6);
}

void SYSconfig_SPIConfiguration() {
    SPI_configuration();
    LCDNokia_init(); /*! Configuration function for the LCD */
    LCDNokia_clear();/*! It clears the information printed in the LCD*/
}

void SYSconfig_UARTConfiguration() {

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    /* config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    config.enableTx = true;
    config.enableRx = true;

    UART_Initialization(&config);
    UART_tasks();

    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
    NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
}

void SYSconfig_I2CConfiguration() {
    inicializacion_I2C();
}
