/*
 * terminal_menus.c
 *
 *  Created on: Mar 24, 2018
 *      Author: Paco
 */

#include "terminal_menus.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"
#include "fsl_debug_console.h"

#define PTA1 (1<< 1)
#define PTA2 (1<< 2)
#define PTB23 (1 << 23)
#define PTB9 (1 << 9)

void PORTA_IRQHandler() {
    uint32_t InterruptFlags;

    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOA);
    if (PTA1 == (InterruptFlags & PTA1))
    {

        GPIO_ClearPinsInterruptFlags(GPIOA, PTA1);
        /*
         * prueba
         */
        PRINTF("PUTO del A1");
    } else if (PTA2 == (InterruptFlags & PTA2))
    {
        PRINTF("MAS PUTO del A2");

        GPIO_ClearPinsInterruptFlags(GPIOA, PTA2);
    }

}


void PORTB_IRQHandler() {
    uint32_t InterruptFlags;

    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOB);
    if (PTB9 == (InterruptFlags & PTB9))
    {

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB9);
        /*
         * prueba
         */
        PRINTF("PUTO del B9");
    } else if (PTB23 == (InterruptFlags & PTB23))
    {
        PRINTF("MAS PUTO del B23");

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB23);
    }

}


