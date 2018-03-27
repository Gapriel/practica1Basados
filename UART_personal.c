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
#include "UART_personal.h"
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
#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART_CLK_FREQ CLOCK_GetFreq(UART0_CLK_SRC)
#define ECHO_BUFFER_LENGTH 1
#define txOnOffGoing (1 << 0)
#define rxOnOffGoing (1 << 1)
#define send_event (1 << 2)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
                       void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
uart_handle_t g_uartHandle;

QueueHandle_t UART_receive_Queue;
QueueHandle_t UART_send_Queue;

EventGroupHandle_t g_UART_Events_personal;

/*******************************************************************************
 * Code
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
                       void *userData) {
    userData = userData;
    BaseType_t pxHigherPriorityTaskWoken;
    pxHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {
        xEventGroupSetBitsFromISR(g_UART_Events_personal, txOnOffGoing,
                                  &pxHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
        xEventGroupSetBitsFromISR(g_UART_Events_personal,
        rxOnOffGoing | send_event,
                                  &pxHigherPriorityTaskWoken);

    }
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void Uart_putChar(uint8_t data){
    static uart_transfer_t receiveXfer_function;
    receiveXfer_function.dataSize = 1;
    receiveXfer_function.data = &data;
    UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle,
                                                     &receiveXfer_function);
                        xEventGroupWaitBits(g_UART_Events_personal, txOnOffGoing,
                        pdTRUE,
                                            pdTRUE,
                                            portMAX_DELAY);
}


void uart_send_task() {

    static uart_transfer_t *receiveXfer_function;

    while (1)
    {
        /*
         * Cuando otra tarea envia un dato por medio de esta Queue, se envía a la UART
         */
        xQueueReceive(UART_send_Queue, &receiveXfer_function, portMAX_DELAY);

        while(*receiveXfer_function->data){
            Uart_putChar(*receiveXfer_function->data++);
        }
    }
}

//void uart_send_task() {
//
//    static uart_transfer_t *receiveXfer_function;
//
//    while (1)
//    {
//        /*
//         * Cuando otra tarea envia un dato por medio de esta Queue, se envía a la UART
//         */
//        xQueueReceive(UART_send_Queue, &receiveXfer_function, portMAX_DELAY);
//
//        UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle,
//                                     receiveXfer_function);
//        xEventGroupWaitBits(g_UART_Events_personal, txOnOffGoing,
//        pdTRUE,
//                            pdTRUE,
//                            portMAX_DELAY);
//
//    }
//
//}

void uart_receive_task() {
    static uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = { 0 };
    uart_transfer_t *sendXfer_function;
    uart_transfer_t prueba_receive;

    sendXfer_function = pvPortMalloc(sizeof(uart_transfer_t*));
    prueba_receive.data = g_rxBuffer;
    prueba_receive.dataSize = ECHO_BUFFER_LENGTH;
    while (1)
    {
        /*
         *
         */
        UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle,
                                        &prueba_receive, NULL);
        /*
         * Espera un dato en cualquier momento
         */
        xEventGroupWaitBits(g_UART_Events_personal, rxOnOffGoing,
        pdTRUE,
                            pdTRUE, portMAX_DELAY);
        sendXfer_function->data = prueba_receive.data;
        sendXfer_function->dataSize = prueba_receive.dataSize;
        /*
         * Lo envia por medio de una Queue para que cualquier tarea pueda leerlo
         */
        xQueueSend(UART_receive_Queue, &sendXfer_function, 0);

    }
}

void UART_tasks(void) {
     g_UART_Events_personal = xEventGroupCreate();

    xTaskCreate(uart_send_task, "send_uart", configMINIMAL_STACK_SIZE + 100,
                NULL, 1, NULL);
    xTaskCreate(uart_receive_task, "receive_uart",
    configMINIMAL_STACK_SIZE + 100,
                NULL, 1, NULL);

    UART_receive_Queue = xQueueCreate(1, sizeof(uart_transfer_t*));
    UART_send_Queue = xQueueCreate(1, sizeof(uart_transfer_t*));
}


void UART_Initialization(uart_config_t *config){
    UART_Init(UART0, config, CLOCK_GetFreq(UART0_CLK_SRC));
    UART_TransferCreateHandle(UART0, &g_uartHandle, UART_UserCallback,
                              NULL);

}
