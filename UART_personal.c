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
#include "fsl_port.h"
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


/*******************************************************************************
 * Code
 ******************************************************************************/


/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
                       void *userData) {

    uart_struct* UART_struct = (uart_struct*) userData;
    BaseType_t pxHigherPriorityTaskWoken;
    pxHigherPriorityTaskWoken = pdFALSE;

    if (kStatus_UART_TxIdle == status)
    {

        xEventGroupSetBitsFromISR(*UART_struct->g_UART_Events_personal,
        txOnOffGoing,
                                  &pxHigherPriorityTaskWoken);
    }

    if (kStatus_UART_RxIdle == status)
    {
        xEventGroupSetBitsFromISR(*UART_struct->g_UART_Events_personal,
        rxOnOffGoing | send_event,
                                  &pxHigherPriorityTaskWoken);

    }
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void Uart_putChar(uart_struct* UART_struct, uint8_t data) {
    static uart_transfer_t receiveXfer_function;
    receiveXfer_function.dataSize = 1;
    receiveXfer_function.data = &data;
    UART_TransferSendNonBlocking(UART_struct->base, UART_struct->handle,
                                 &receiveXfer_function);
    xEventGroupWaitBits(*UART_struct->g_UART_Events_personal, txOnOffGoing,
    pdTRUE,
                        pdTRUE,
                        portMAX_DELAY);
}

void uart_send_task(void* args) {

    uart_struct* UART_struct = (uart_struct*) args;

    static uart_transfer_t *receiveXfer_function;
    while (1)
    {
        /*
         * Cuando otra tarea envia un dato por medio de esta Queue, se envía a la UART
         */

        xQueueReceive(*UART_struct->UART_send_Queue, &receiveXfer_function,
                      portMAX_DELAY);

        while (*receiveXfer_function->data)
        {
            Uart_putChar(UART_struct, *receiveXfer_function->data++);
        }
        vPortFree(receiveXfer_function);
    }
}

void uart_receive_task(void* args) {
    uart_struct* UART_struct = (uart_struct*) args;

    static uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = { 0 };
    uart_transfer_t *sendXfer_function;
    uart_transfer_t receiveXfer;


    while (1)
    {

        sendXfer_function = pvPortMalloc(sizeof(uart_transfer_t*));
        receiveXfer.data = g_rxBuffer;
        receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
        /*
         *
         */
        UART_TransferReceiveNonBlocking(UART_struct->base, UART_struct->handle,
                                        &receiveXfer, NULL);
        /*
         * Espera un dato en cualquier momento
         */
        xEventGroupWaitBits(*UART_struct->g_UART_Events_personal, rxOnOffGoing,
                            pdTRUE, pdTRUE, portMAX_DELAY);
        sendXfer_function->data = receiveXfer.data;
        sendXfer_function->dataSize = receiveXfer.dataSize;
        /*
         * Lo envia por medio de una Queue para que cualquier tarea pueda leerlo
         */
        xQueueSend(*UART_struct->UART_receive_Queue, &sendXfer_function, portMAX_DELAY);

    }
}


void UART_tasks(void*args) {
    uart_struct* UART_struct = (uart_struct*) (args);
    *UART_struct->g_UART_Events_personal = xEventGroupCreate();

    xTaskCreate(uart_send_task, "send_uart", configMINIMAL_STACK_SIZE + 100,
                UART_struct, 2, NULL);
    xTaskCreate(uart_receive_task, "receive_uart",
    configMINIMAL_STACK_SIZE + 100,
                UART_struct, 2, NULL);

    *UART_struct->UART_receive_Queue = xQueueCreate(1,
                                                    sizeof(uart_transfer_t*));
    *UART_struct->UART_send_Queue = xQueueCreate(1, sizeof(uart_transfer_t*));
}


void SYSconfig_UARTConfiguration(uart_struct *UART_struct) {

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    port_pin_config_t uart1_configuration = { kPORT_PullDisable,
                  kPORT_SlowSlewRate, kPORT_PassiveFilterEnable, kPORT_OpenDrainDisable,
                  kPORT_LowDriveStrength, kPORT_MuxAlt3, kPORT_UnlockRegister };

    switch(UART_struct->uart_number){
        case UART_0:
            config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
            config.enableTx = true;
            config.enableRx = true;

            UART_Init(UART_struct->base, &config, CLOCK_GetFreq(UART0_CLK_SRC));

            UART_TransferCreateHandle(UART_struct->base, UART_struct->handle,
                                      UART_UserCallback, (void*) UART_struct);

            NVIC_EnableIRQ(UART0_RX_TX_IRQn);
            NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
            break;
        case UART_1:

            CLOCK_EnableClock(kCLOCK_PortC);
            CLOCK_EnableClock(kCLOCK_Uart1);
            config.baudRate_Bps = 9600U;

            config.enableTx = true;
            config.enableRx = true;
            PORT_SetPinConfig(PORTC, 3, &uart1_configuration);
            PORT_SetPinConfig(PORTC, 4, &uart1_configuration);
            UART_Init(UART_struct->base, &config, CLOCK_GetFreq(UART0_CLK_SRC));

            UART_TransferCreateHandle(UART_struct->base, UART_struct->handle,
                                      UART_UserCallback, (void*) UART_struct);

            NVIC_EnableIRQ(UART1_RX_TX_IRQn);
            NVIC_SetPriority(UART1_RX_TX_IRQn, 5);
            break;
        default:
            break;
    }

}
