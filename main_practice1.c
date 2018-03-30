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


extern QueueHandle_t I2C_write_queue;
extern QueueHandle_t I2C_read_queue;
extern QueueHandle_t SPI_queue;
extern SemaphoreHandle_t I2C_done;

static QueueHandle_t UART0_send_Queue;
static QueueHandle_t UART0_receive_Queue;
static uart_handle_t g_uart0Handle;
static EventGroupHandle_t g_UART0_Events;


static QueueHandle_t UART1_send_Queue;
static QueueHandle_t UART1_receive_Queue;
static uart_handle_t g_uart1Handle;
static EventGroupHandle_t g_UART1_Events;


static uart_struct UART_0_struct = {
        UART0,
        &g_uart0Handle,
        &g_UART0_Events,
        &UART0_receive_Queue,
        &UART0_send_Queue,
        UART_0
};


static uart_struct UART_1_struct = {
        UART1,
        &g_uart1Handle,
        &g_UART1_Events,
        &UART1_receive_Queue,
        &UART1_send_Queue,
        UART_1
};

#if 1
void print_eco_task(void* args){
    SPI_msg_t *message;
    uart_struct* UART_struct = (uart_struct*) args;


    i2c_master_transfer_t *masterXfer_prueba;
    masterXfer_prueba = pvPortMalloc(sizeof(i2c_master_transfer_t*));
  volatile  uint8_t buffer_prueba[1] = {0};

    uart_transfer_t* received_UART;
    uart_transfer_t* toSend_UART;
    uint8_t address_index = 0;
    xSemaphoreGive(I2C_done);
    while(1){

        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        message = pvPortMalloc(sizeof(SPI_msg_t*)); /**memory is reserved for the message to be sent*/

        /*
         * Se recibe un dato por medio de la UART
         */
        xQueueReceive(*UART_struct->UART_receive_Queue,&received_UART,portMAX_DELAY);
        /*
         * Configuracion del masterXfer para guardar en I2C
         */
        masterXfer_prueba->data = received_UART->data;
        masterXfer_prueba->dataSize =1;
        masterXfer_prueba->slaveAddress =0x51;
        masterXfer_prueba->subaddress = address_index;
        masterXfer_prueba->subaddressSize =2;
        masterXfer_prueba->flags = kI2C_TransferDefaultFlag;
        masterXfer_prueba->direction = kI2C_Write;
        /*
         * Se envía a la memoria por medio de I2C y se espera confirmacion de escritura
         */
        xQueueSend(I2C_write_queue ,&masterXfer_prueba,portMAX_DELAY);
        taskYIELD();
        xSemaphoreTake(I2C_done,portMAX_DELAY);
        xSemaphoreGive(I2C_done);

        /*
         * Se modifica el masterXfer para realizar una lectura en otra ubicacion para comprobar la correcta escritura/lectura
         * del dispositivo
         */
        vTaskDelay(pdMS_TO_TICKS(20));
        masterXfer_prueba->data = buffer_prueba;
        masterXfer_prueba->direction = kI2C_Read;
        /*
         * Se lee el dato y se espera la confirmacion
         */
        xQueueSend(I2C_write_queue ,&masterXfer_prueba,portMAX_DELAY);
        xSemaphoreTake(I2C_done,portMAX_DELAY);
             xSemaphoreGive(I2C_done);
        /*
         * Para enviar el dato por medio de la uart se modifica toSend_UART
         */

        toSend_UART->data = masterXfer_prueba->data;
        toSend_UART->dataSize = 1;

        /*
         * Se envia el dato por medio de la UART
         */
        message->string = toSend_UART->data;
        message->LCD_to_be_clear = pdFALSE;
        message->string[1] = '\0';
        xQueueSend(SPI_queue,&message,portMAX_DELAY);
        xQueueSend(*UART_struct->UART_send_Queue,&toSend_UART,portMAX_DELAY);
        address_index++;
    }
}
#endif


int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    uart_struct* p_UART_0_struct = &UART_0_struct;
//    uart_struct* p_UART_1_struct = &UART_1_struct;


    xTaskCreate(SystemConfiguration, "CONFIG",configMINIMAL_STACK_SIZE,NULL,5,NULL);

   SYSconfig_UARTConfiguration(p_UART_0_struct);
    UART_tasks((void*) p_UART_0_struct);

//  SYSconfig_UARTConfiguration(p_UART_1_struct);
//  UART_tasks((void*) p_UART_1_struct);


    inicializacion_I2C();
   // xTaskCreate(print_eco_task, "ECO", configMINIMAL_STACK_SIZE, (void*)p_UART_0_struct, 2, NULL);
     xTaskCreate(TerminalMenus_MainMenu, "test menu 0", configMINIMAL_STACK_SIZE, (void*)p_UART_0_struct,4, NULL);
  // xTaskCreate(TerminalMenus_MainMenu, "test menu 1", configMINIMAL_STACK_SIZE, (void*)p_UART_1_struct,4, NULL);

    vTaskStartScheduler();
    while(1) {

    }
    return 0 ;
}
