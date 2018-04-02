/*
 * UART_personal.h
 *
 *  Created on: Mar 24, 2018
 *      Author: Paco
 */

#ifndef UART_PERSONAL_H_
#define UART_PERSONAL_H_


/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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
 * o Neither the name of the copyright holder nor the names of its
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

#include "board.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define ECHO_BUFFER_LENGTH 1
#define txOnOffGoing (1 << 0)
#define rxOnOffGoing (1 << 1)
#define send_event (1 << 2)


typedef enum {UART_0,UART_1,UART_2}UART_NUMBER;

typedef struct{
    uint8_t terminalChatStates[2];
    uint8_t chatPosition[8];
    QueueHandle_t* UART0_send_Queue;
    QueueHandle_t* UART1_send_Queue;
    uint8_t FirstEntry[2];
}chatStates_t;

typedef struct{

    UART_Type *base;
    uart_handle_t* handle;
    EventGroupHandle_t* g_UART_Events_personal;
    QueueHandle_t* UART_receive_Queue;
    QueueHandle_t* UART_send_Queue;
    UART_NUMBER uart_number;
    chatStates_t * ChatStates;
}uart_struct;

/*
 * Tarea encargada de enviar datos a la UART
 */
void uart_send_task(void*) ;

/*
 * Tarea encargada de recibir datos de la UART
 */

void uart_receive_task(void*) ;

/*
 * Funcion de inicializacion de la UART
 */
void UART_tasks(void*) ;




void SYSconfig_UARTConfiguration(uart_struct *UART_struct) ;

#endif /* UART_PERSONAL_H_ */
