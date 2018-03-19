/*
 * SysConfiguration.c
 *
 *  Created on: Mar 5, 2018
 *      Author:
 */

#include "SysConfiguration.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "SPI_driver.h"
#include "LCDNokia5110.h"
#include "semphr.h"
#include "I2C_driver.h"

//////////////////**user types definitions*////////////////////////////
typedef enum {
    PTA1 = 1, PTA2
} PORTA_buttons_pins_t;

typedef enum {
    PTB9 = 9, PTB23 = 23
} PORTB_buttons_pins_t;

typedef enum {
    PTC2 = 2, PTC3, PTC16 = 16, PTC17
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
void SystemConfiguration(void* args)
{
    for(;;){

        /**ports A,B,C clock will be enabled, as pins from all those will be used*/
        CLOCK_EnableClock(kCLOCK_PortA);
        CLOCK_EnableClock(kCLOCK_PortB);
        CLOCK_EnableClock(kCLOCK_PortC);
        /**modules configuration*/
        SYSconfig_ButtonsConfiguration(); /**buttons configuration*/
      //  SYSconfig_SPIConfiguration(); /**SPI module configuration (including device initialization)*/
        SYSconfig_UARTConfiguration(); /**UART module configuration*/
        SYSconfig_I2CConfiguration(); /**I2C module configuration*/

        vTaskSuspend(NULL); /**the function auto suspends itself, as it won't be used again*/
        //vTaskDelete(NULL);
    }
}

void SYSconfig_ButtonsConfiguration()
{ /**this function responsibility is to configure everything related to the buttons which will be used*/
    /**button configuration definition*/
    port_pin_config_t button_configuration = { kPORT_PullDisable,
        kPORT_SlowSlewRate, kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
    /**button pins configuration*/
    PORT_SetPinConfig(PORTC, PTC3, &button_configuration); /**B0 configuration*/
    PORT_SetPinConfig(PORTC, PTC2, &button_configuration); /**B1 configuration*/
    PORT_SetPinConfig(PORTA, PTA2, &button_configuration); /**B2 configuration*/
    PORT_SetPinConfig(PORTB, PTB23, &button_configuration);/**B3 configuration*/
    PORT_SetPinConfig(PORTA, PTA1, &button_configuration); /**B4 configuration*/
    PORT_SetPinConfig(PORTB, PTB9, &button_configuration); /**B5 configuration*/
    /**interrupts configuration*/
    PORT_SetPinInterruptConfig(PORTC, PTC3, kPORT_InterruptLogicOne); /**B0 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTC, PTC2, kPORT_InterruptLogicOne); /**B1 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTA, PTA2, kPORT_InterruptLogicOne); /**B2 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTB, PTB23, kPORT_InterruptLogicOne); /**B3 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTA, PTA1, kPORT_InterruptLogicOne); /**B4 configured to interrupt when a logic 1 is read*/
    PORT_SetPinInterruptConfig(PORTB, PTB9, kPORT_InterruptLogicOne); /**B5 configured to interrupt when a logic 1 is read*/
}

void SYSconfig_SPIConfiguration()
{
    SPI_configuration();
    LCDNokia_init(); /*! Configuration function for the LCD */
    LCDNokia_clear();/*! It clears the information printed in the LCD*/
}

void SYSconfig_UARTConfiguration()
{

}

void SYSconfig_I2CConfiguration()
{
    I2C_configuration();
}

////////////**I2C Bug fixing functions provided by NXP*/////////////
//static void i2c_release_bus_delay(void)
//{
//    uint32_t i = 0;
//    for (i = 0; i < 100; i++)
//    {
//        __NOP();
//    }
//}
//void i2c_ReleaseBus()
//{
//    uint8_t i = 0;
//    gpio_pin_config_t pin_config;
//    port_pin_config_t i2c_pin_config = { 0 };
//
//    /* Config pin mux as gpio */
//    i2c_pin_config.pullSelect = kPORT_PullUp;
//    i2c_pin_config.mux = kPORT_MuxAsGpio;
//
//    pin_config.pinDirection = kGPIO_DigitalOutput;
//    pin_config.outputLogic = 1U;
//    CLOCK_EnableClock(kCLOCK_PortE);
//    PORT_SetPinConfig(PORTE, 24, &i2c_pin_config);
//    PORT_SetPinConfig(PORTE, 25, &i2c_pin_config);
//
//    GPIO_PinInit(GPIOE, 24, &pin_config);
//    GPIO_PinInit(GPIOE, 25, &pin_config);
//
//    GPIO_PinWrite(GPIOE, 25, 0U);
//    i2c_release_bus_delay();
//
//    for (i = 0; i < 9; i++)
//    {
//        GPIO_PinWrite(GPIOE, 24, 0U);
//        i2c_release_bus_delay();
//
//        GPIO_PinWrite(GPIOE, 25, 1U);
//        i2c_release_bus_delay();
//
//        GPIO_PinWrite(GPIOE, 24, 1U);
//        i2c_release_bus_delay();
//        i2c_release_bus_delay();
//    }
//
//    GPIO_PinWrite(GPIOE, 24, 0U);
//    i2c_release_bus_delay();
//
//    GPIO_PinWrite(GPIOE, 25, 0U);
//    i2c_release_bus_delay();
//
//    GPIO_PinWrite(GPIOE, 24, 1U);
//    i2c_release_bus_delay();
//
//    GPIO_PinWrite(GPIOE, 25, 1U);
//    i2c_release_bus_delay();
//}
