/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    Practica1_Embebidos_UART_SPI_I2C.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "FreeRTOS.h"
#include "fsl_i2c.h"
#include "fsl_dspi.h"
#include "event_groups.h"
#include "semphr.h"
#include "task.h"
#include "fsl_port.h"
#include "FreeRTOSConfig.h"
#include "UART_personal.h"
#include "I2C_no_bloqueante.h"
#include "SPI_driver.h"
#include "SysConfiguration.h"
#include "LCDNokia5110.h"
#include "terminal_menus.h"
#define STACK_SIZE 150

QueueHandle_t UART0_send_Queue;
QueueHandle_t UART0_receive_Queue;
uart_handle_t g_uart0Handle;
EventGroupHandle_t g_UART0_Events;

QueueHandle_t UART1_send_Queue;
QueueHandle_t UART1_receive_Queue;
uart_handle_t g_uart1Handle;
EventGroupHandle_t g_UART1_Events;

chatStates_t TerminalChatStates = {
    {pdFALSE,pdFALSE},
    {"\033[12;10H\0"},
    &UART0_send_Queue,
    &UART1_send_Queue,
    {pdFALSE,pdFALSE},
    {{"\033[05;10H\0"},{"\033[05;10H\0"}}
};

static uart_struct UART_0_struct = {
        UART0,
        &g_uart0Handle,
        &g_UART0_Events,
        &UART0_receive_Queue,
        &UART0_send_Queue,
        UART_0,
        &TerminalChatStates
};


static uart_struct UART_1_struct = {
        UART1,
        &g_uart1Handle,
        &g_UART1_Events,
        &UART1_receive_Queue,
        &UART1_send_Queue,
        UART_1,
        &TerminalChatStates
};



int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    uart_struct* p_UART_0_struct = &UART_0_struct;
    uart_struct* p_UART_1_struct = &UART_1_struct;


    xTaskCreate(SystemConfiguration, "CONFIG",configMINIMAL_STACK_SIZE,NULL,5,NULL);
    xTaskCreate(I2CInit, "Init I2C", configMINIMAL_STACK_SIZE , NULL, 5,
    NULL);
    SYSconfig_UARTConfiguration(p_UART_0_struct);
    UART_tasks((void*) p_UART_0_struct);
    SYSconfig_UARTConfiguration(p_UART_1_struct);
    UART_tasks((void*) p_UART_1_struct);
    xTaskCreate(TerminalMenus_MainMenu, "test menu 0", configMINIMAL_STACK_SIZE - 30 , (void*)p_UART_0_struct, 4 , NULL);
    xTaskCreate(TerminalMenus_MainMenu, "test menu 1", configMINIMAL_STACK_SIZE -30 , (void*)p_UART_1_struct, 4 , NULL);
   vTaskStartScheduler();
    while(1) {

    }
    return 0 ;
}
