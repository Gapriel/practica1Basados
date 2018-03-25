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

#define STACK_SIZE 150
extern QueueHandle_t UART_send_Queue;
extern QueueHandle_t UART_receive_Queue;
extern QueueHandle_t I2C_write_queue;
extern QueueHandle_t I2C_read_queue;
extern QueueHandle_t SPI_queue;


void inicializacion_spi(void)
{
    xTaskCreate(SystemConfiguration, "System initial configuration", STACK_SIZE,
                (void*) NULL, 4, NULL); /**System configuration task creation*/
 //   xTaskCreate(probandoSPI, "prueba", STACK_SIZE, (void*) NULL, 2, NULL);

}

void print_task(){
    SPI_msg_t *message;


    i2c_master_transfer_t *masterXfer_prueba;
    masterXfer_prueba = pvPortMalloc(sizeof(i2c_master_transfer_t*));
  volatile  uint8_t buffer_prueba[1] = {0};

    uart_transfer_t* received_UART;
    uart_transfer_t* toSend_UART;

    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
    while(1){
        message = pvPortMalloc(sizeof(SPI_msg_t*)); /**memory is reserved for the message to be sent*/

        /*
         * Se recibe un dato por medio de la UART
         */
        xQueueReceive(UART_receive_Queue,&received_UART,portMAX_DELAY);
        /*
         * Configuracion del masterXfer para guardar en I2C
         */
        masterXfer_prueba->data = received_UART->data;
        masterXfer_prueba->dataSize =1;
        masterXfer_prueba->slaveAddress =0x50;
        masterXfer_prueba->subaddress = 0x04;
        masterXfer_prueba->subaddressSize =1;
        masterXfer_prueba->flags = kI2C_TransferDefaultFlag;
        masterXfer_prueba->direction = kI2C_Write;
#if 1
        /*
         * Se envía a la memoria por medio de I2C y se espera confirmacion de escritura
         */
        xQueueSend(I2C_write_queue ,&masterXfer_prueba,portMAX_DELAY);
        xQueueReceive(I2C_read_queue,&masterXfer_prueba,portMAX_DELAY);

        /*
         * Se modifica el masterXfer para realizar una lectura en otra ubicacion para comprobar la correcta escritura/lectura
         * del dispositivo
         */

        masterXfer_prueba->data = buffer_prueba;
        masterXfer_prueba->direction = kI2C_Read;
        /*
         * Se lee el dato y se espera la confirmacion
         */
        xQueueSend(I2C_write_queue ,&masterXfer_prueba,portMAX_DELAY);
        xQueueReceive(I2C_read_queue,&masterXfer_prueba,portMAX_DELAY);
        /*
         * Para enviar el dato por medio de la uart se modifica toSend_UART
         */

        toSend_UART->data = masterXfer_prueba->data;
#else
        toSend_UART->data = masterXfer_prueba->data;
#endif
        toSend_UART->dataSize = 1;

        /*
         * Se envia el dato por medio de la UART
         */
        message->string_to_be_printed[0] = toSend_UART->data[0];
        xQueueSend(SPI_queue,&message,portMAX_DELAY);
        xQueueSend(UART_send_Queue,&toSend_UART,portMAX_DELAY);

    }


}
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    inicializacion_spi();
    inicializacion_uart();
    inicializacion_I2C();
    xTaskCreate(print_task, "PRINT TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    vTaskStartScheduler();
    while(1) {

    }
    return 0 ;
}
