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
 * @file    I2C_no_bloqueante.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "fsl_i2c.h"
#include "semphr.h"
#include "fsl_port.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "FreeRTOSConfig.h"
#include "I2C_no_bloqueante.h"

static i2c_master_handle_t g_m_handle; //I2C_0 master handler declared

QueueHandle_t I2C_write_queue;
QueueHandle_t I2C_read_queue;
EventGroupHandle_t I2C_events;

SemaphoreHandle_t I2C_done;


#define I2C_free (1<< 0)
#define I2C_data_ready (1 << 1)

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                                status_t status, void * userData) {
    BaseType_t pxHigherPriorityTaskWoken;
    pxHigherPriorityTaskWoken = pdFALSE;
    if (status == kStatus_Success)
    {

            xEventGroupSetBitsFromISR(I2C_events, I2C_free,
                                      &pxHigherPriorityTaskWoken);

    }

    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void I2CInit() {
    //clock enabling
    CLOCK_EnableClock(kCLOCK_PortC); //I2C_1 pins port clock enabling
    CLOCK_EnableClock(kCLOCK_I2c1); //I2C_1 clock enabling

    //I2C_0 pins configuration
    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTC, 10, &config_i2c);  //I2C_0 SCL pin configuration
    PORT_SetPinConfig(PORTC, 11, &config_i2c);  //I2C_0 SDA pin configuration

    //I2C_0 master configuration
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig); //I2C_0 master default config. obtanined
    masterConfig.baudRate_Bps = 100000;
    I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

    //I2C_0 master handler creation
    I2C_MasterTransferCreateHandle(I2C1, &g_m_handle, i2c_master_callback,
    NULL);
    NVIC_EnableIRQ(I2C1_IRQn);
    NVIC_SetPriority(I2C1_IRQn, 6);
    xEventGroupSetBits(I2C_events, I2C_free);
    vTaskDelete(NULL);

}

void I2C_transfer() {

    i2c_master_transfer_t *masterXfer;
    uint8_t valor_a_enviar[1];

//I2C_0 data block transmission
    while (1)
    {

        xQueueReceive(I2C_write_queue, &masterXfer, portMAX_DELAY);

        xEventGroupWaitBits(I2C_events, I2C_free, pdTRUE, pdTRUE,
        portMAX_DELAY);
        I2C_MasterTransferNonBlocking(I2C1, &g_m_handle, masterXfer);
        xEventGroupWaitBits(I2C_events, I2C_free, pdFALSE, pdTRUE,
               portMAX_DELAY);
        xQueueSend(I2C_read_queue,&masterXfer,portMAX_DELAY);

    }

}

void I2C_prueba() {
    i2c_master_transfer_t *masterXfer_write_read;
    i2c_master_transfer_t *masterXfer_recibido;
    uint8_t buffer_escritura[1] = {4};
    masterXfer_write_read = pvPortMalloc(sizeof(i2c_master_transfer_t*));


    //I2C_0 data block definition
    masterXfer_write_read->slaveAddress = 0x50;
    masterXfer_write_read->subaddress = 0x01;
    masterXfer_write_read->subaddressSize = 2;
    masterXfer_write_read->dataSize = 1;
    masterXfer_write_read->flags = kI2C_TransferDefaultFlag;
    masterXfer_write_read->direction = kI2C_Write;
    masterXfer_write_read->data = buffer_escritura;

    while (1)
    {

        xQueueSend(I2C_write_queue, &masterXfer_write_read, portMAX_DELAY);
        xQueueReceive(I2C_read_queue, &masterXfer_write_read, portMAX_DELAY);


        PRINTF("\r %i \n", masterXfer_write_read->data[0]);


    }

}

void inicializacion_I2C(void) {

    I2C_events = xEventGroupCreate();
    I2C_done = xSemaphoreCreateBinary();
    I2C_write_queue = xQueueCreate(1, sizeof(i2c_master_transfer_t*));
    I2C_read_queue = xQueueCreate(1, sizeof(i2c_master_transfer_t*));

    xTaskCreate(I2CInit, "Init I2C", configMINIMAL_STACK_SIZE , NULL, 4,
    NULL);
    xTaskCreate(I2C_transfer, "transfer", configMINIMAL_STACK_SIZE, NULL,
                        1, NULL);

}
