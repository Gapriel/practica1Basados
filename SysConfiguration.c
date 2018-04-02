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
#include "terminal_menus.h"
#include "event_groups.h"

#define FREE_MEM_EVENT (1<<0)
#define FREE_RTC_EVENT (1<< 1)


volatile EventGroupHandle_t* pSubTasks_Events;
SemaphoreHandle_t* pInterface_mutex;

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
        /* Gets the references of the Interface mutex for the UARTS and the subtaks events from the terminal menus.*/
        pInterface_mutex = (SemaphoreHandle_t*) pGetInterfaceMutex();
        pSubTasks_Events = (EventGroupHandle_t*) pGetSubTasksEvents();

        /* Initialize the Interface menu mutex and the Events for the SubTaks*/
        *pSubTasks_Events = xEventGroupCreate();
        *pInterface_mutex = xSemaphoreCreateMutex();
        SYSconfig_ButtonsConfiguration(); /**buttons configuration*/
        SYSconfig_SPIConfiguration(); /**SPI module configuration (including device initialization)*/
        SYSconfig_I2CConfiguration(); /**I2C module configuration*/

        /* Sets the FREE MEMORY AND FREE RTC Events*/
       xEventGroupSetBits(*pSubTasks_Events, FREE_MEM_EVENT|FREE_RTC_EVENT);

       /*Because it's just for comfiguration*/
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



void SYSconfig_I2CConfiguration() {

    //clock enabling
    CLOCK_EnableClock(kCLOCK_PortC); //I2C_1 pins port clock enabling
    CLOCK_EnableClock(kCLOCK_I2c1); //I2C_1 clock enabling

    //I2C_0 pins configuration
    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTC, 10, &config_i2c);  //I2C_0 SCL pin configuration
    PORT_SetPinConfig(PORTC, 11, &config_i2c);  //I2C_0 SDA pin configuration

    //I2C_0 master configuration
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig); //I2C_0 master default config. obtanined
    masterConfig.baudRate_Bps = 100000;
    I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

    NVIC_EnableIRQ(I2C1_IRQn);
    NVIC_SetPriority(I2C1_IRQn, 6);
}
