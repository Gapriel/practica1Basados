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
#define txBufferFull_or_Empty (1 << 0)
#define rxBufferFull_or_Empty (1 << 1)
#define txOnOffGoing (1 << 2)
#define rxOnOffGoing (1 << 3)
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

EventGroupHandle_t g_UART_Events_personal;
uint8_t g_tipString[] =
        "Uart interrupt example\r\nBoard receives 1 characters then sends them out\r\nNow please input:\r\n";

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = { 0 };
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = { 0 };

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void UART0_DriverIRQHandler(void);

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
                       void *userData) {
    userData = userData;

    if (kStatus_UART_TxIdle == status)
    {
        xEventGroupClearBits(g_UART_Events_personal, txBufferFull_or_Empty);
        xEventGroupClearBits(g_UART_Events_personal, txOnOffGoing);

    }

    if (kStatus_UART_RxIdle == status)
    {
        xEventGroupSetBits(g_UART_Events_personal, rxBufferFull_or_Empty);
        xEventGroupClearBits(g_UART_Events_personal, rxOnOffGoing);

    }
}

/*!
 * @brief Main function
 */
int main(void) {
    uart_config_t config;
    uart_transfer_t xfer;
    uart_transfer_t sendXfer;
    uart_transfer_t receiveXfer;
    g_UART_Events_personal = xEventGroupCreate();
    BOARD_InitPins();
    BOARD_BootClockRUN();

    BOARD_InitDebugConsole();

    UART_GetDefaultConfig(&config);

    config.baudRate_Bps = 115200U;
    /* config.parityMode = kUART_ParityDisabled;
     * config.stopBitCount = kUART_OneStopBit;
     * config.txFifoWatermark = 0;
     * config.rxFifoWatermark = 1;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    config.enableTx = true;
    config.enableRx = true;

    UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
    UART_TransferCreateHandle(DEMO_UART, &g_uartHandle, UART_UserCallback,
    NULL);

    /*******************************************************************************
     * Codigo para limpiar la terminal y reiniciar el cursor.
     ******************************************************************************/
    uint8_t limpiar[] = "\033[2J";
    xfer.data = limpiar;
    xfer.dataSize = sizeof(limpiar) - 1;
    xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
    UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

    /* Wait send finished */
    while (txOnOffGoing
            == (xEventGroupGetBits(g_UART_Events_personal) & (txOnOffGoing)))
    {
    }
    uint8_t reiniciar_consola[] = "\033[1;1H";
    xfer.data = reiniciar_consola;
    xfer.dataSize = sizeof(reiniciar_consola) - 1;
    xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
    UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

    /* Wait send finished */
    while (txOnOffGoing
            == (xEventGroupGetBits(g_UART_Events_personal) & (txOnOffGoing)))
    {
    }

    /*******************************************************************************
     * Code
     ******************************************************************************/
    /* Send g_tipString out. */
    xfer.data = g_tipString;
    xfer.dataSize = sizeof(g_tipString) - 1;
    xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
    UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

    /* Wait send finished */
    while (txOnOffGoing
            == (xEventGroupGetBits(g_UART_Events_personal) & (txOnOffGoing)))
    {
    }

    /* Start to echo. */
    sendXfer.data = g_txBuffer;
    sendXfer.dataSize = ECHO_BUFFER_LENGTH;
    receiveXfer.data = g_rxBuffer;
    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;

    while (1)
    {
        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
        if ((0)
                == (xEventGroupGetBits(g_UART_Events_personal)
                        & (rxOnOffGoing | rxBufferFull_or_Empty)))
        {
            xEventGroupSetBits(g_UART_Events_personal, rxOnOffGoing);
            UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle,
                                            &receiveXfer, NULL);
        }

        /* If TX is idle and g_txBuffer is full, start to send data. */

        if ((txBufferFull_or_Empty)
                == (xEventGroupGetBits(g_UART_Events_personal)
                        & (txOnOffGoing | txBufferFull_or_Empty)))
        {
            xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
            UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &sendXfer);
        }

        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
        if ((rxBufferFull_or_Empty)
                == (xEventGroupGetBits(g_UART_Events_personal)
                        & (rxBufferFull_or_Empty | txBufferFull_or_Empty)))
        {
            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
            xEventGroupClearBits(g_UART_Events_personal, rxBufferFull_or_Empty);
            xEventGroupSetBits(g_UART_Events_personal, txBufferFull_or_Empty);
        }
    }
}
