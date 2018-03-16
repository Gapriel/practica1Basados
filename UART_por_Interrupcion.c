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
uart_config_t config;

EventGroupHandle_t g_UART_Events_personal;

static QueueHandle_t UART_Queue;

typedef struct {

} uart_;

uint8_t g_tipString[] =
		"Uart interrupt example\r\nBoard receives 1 characters then sends them out\r\nNow please input:\r\n";

/*******************************************************************************
 * Code
 ******************************************************************************/
extern void UART0_DriverIRQHandler(void);

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status,
		void *userData) {
	userData = userData;
	BaseType_t pxHigherPriorityTaskWoken;
	pxHigherPriorityTaskWoken = pdFALSE;

	if (kStatus_UART_TxIdle == status) {
		xEventGroupSetBitsFromISR(g_UART_Events_personal, txOnOffGoing,
				&pxHigherPriorityTaskWoken);
	}

	if (kStatus_UART_RxIdle == status) {
		xEventGroupSetBitsFromISR(g_UART_Events_personal,
		rxOnOffGoing | send_event, &pxHigherPriorityTaskWoken);

	}
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void uart_function() {

	uart_transfer_t xfer;

	/*******************************************************************************
	 * Codigo para limpiar la terminal y reiniciar el cursor.
	 ******************************************************************************/
	uint8_t limpiar[] = "\033[2J";
	xfer.data = limpiar;
	xfer.dataSize = sizeof(limpiar) - 1;
	xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
	UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

	/*******************************************************************************
	 * Code
	 ******************************************************************************/
	/* Send g_tipString out. */
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;
	xEventGroupSetBits(g_UART_Events_personal, txOnOffGoing);
	UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle, &xfer);

	/* Wait send finished */
	xEventGroupWaitBits(g_UART_Events_personal, txOnOffGoing, pdFALSE, pdTRUE,
	portMAX_DELAY);

	while (1) {
		vTaskDelay(50000);
	}
}

void uart_send_function() {

	uart_transfer_t *receiveXfer_function;
	while (1) {
		/*
		 xEventGroupWaitBits(g_UART_Events_personal, txOnOffGoing | send_event,
		 pdTRUE,
		 pdTRUE,
		 portMAX_DELAY);*/
		xQueueReceive(UART_Queue, &receiveXfer_function, portMAX_DELAY);

		UART_TransferSendNonBlocking(DEMO_UART, &g_uartHandle,
				receiveXfer_function);
		vPortFree(receiveXfer_function);

	}

}

void uart_receive_function() {
	static uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = { 0 };
	uart_transfer_t *sendXfer_function;
	uart_transfer_t prueba_receive;

	prueba_receive.data = g_rxBuffer;
	prueba_receive.dataSize = ECHO_BUFFER_LENGTH;
	while (1) {

		UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle,
				&prueba_receive,
				NULL);
		xEventGroupWaitBits(g_UART_Events_personal, rxOnOffGoing,
		pdTRUE, pdTRUE, portMAX_DELAY);
		sendXfer_function = pvPortMalloc(sizeof(uart_transfer_t));
		sendXfer_function->data = prueba_receive.data;
		sendXfer_function->dataSize = prueba_receive.dataSize;
		xQueueSend(UART_Queue, &sendXfer_function, portMAX_DELAY);

	}
}

int main(void) {
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();

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

	UART_Init(DEMO_UART, &config, DEMO_UART_CLK_FREQ);
	UART_TransferCreateHandle(DEMO_UART, &g_uartHandle, UART_UserCallback,
	NULL);

	NVIC_EnableIRQ(UART0_RX_TX_IRQn);
	NVIC_SetPriority(UART0_RX_TX_IRQn, 5);
	g_UART_Events_personal = xEventGroupCreate();
	xTaskCreate(uart_function, "UART_TASK", configMINIMAL_STACK_SIZE + 100,
	NULL,
	configMAX_PRIORITIES - 1, NULL);

	xTaskCreate(uart_send_function, "send", configMINIMAL_STACK_SIZE + 100,
	NULL, configMAX_PRIORITIES - 2, NULL);
	xTaskCreate(uart_receive_function, "receive",
	configMINIMAL_STACK_SIZE + 100, NULL, configMAX_PRIORITIES - 3,
	NULL);

	UART_Queue = xQueueCreate(1, sizeof(uart_transfer_t));

	vTaskStartScheduler();
}
