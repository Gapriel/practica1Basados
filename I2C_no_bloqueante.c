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

/*
 *      Author: Avelar Díaz José Francisco      ie704217@iteso.mx
 *      Author: Santamaría García Gabriel       ie699356@iteso.mx
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

////////////////////////////////** For the Events of the I2C  **/////////////////////////////
#define I2C_free (1<< 0)                                                                   //
#define I2C_data_ready (1 << 1)                                                            //
/////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_WAITING_TIME (pdMS_TO_TICKS(500))  /* Maximum Wwaiting time for the I2C device */

//////////////**OS I2C resources declarations*///////////////////////////////////////////////

i2c_master_handle_t g_m_handle; /* I2C_0 master handler declared */

QueueHandle_t I2C_write_queue; /* Queue used for the transfer of the I2C */

EventGroupHandle_t I2C_events; /* To check if the I2C is free */

SemaphoreHandle_t I2C_done; /* Points that the I2C have finished the transfer */

TaskHandle_t I2C_Transfer; /*Handler of the transfer task of the I2C */

TimerHandle_t I2C_Timer_Handler; /*Handler for the I2C timer to check if the device is found */

/////////////////////**module callbacks*//////////////////////////////////////////////////////
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                                status_t status, void * userData) {

    /* If the transfer is succesfull the I2C Events is Set to FreeI2C  */
    BaseType_t pxHigherPriorityTaskWoken;
    pxHigherPriorityTaskWoken = pdFALSE;
    if (status == kStatus_Success)
    {
        xEventGroupSetBitsFromISR(I2C_events, I2C_free,
                                  &pxHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}




/* Returns the address of the Event Group Handler of the I2C*/
EventGroupHandle_t* pGetI2CEvents() {
    return &I2C_events;
}

/* Returns the addres of the Queue Handler of the I2C Transfer */
QueueHandle_t* pGetI2CHandler() {
    return &I2C_write_queue;
}

/* Returns the Handler of the Mutex of the I2C */
SemaphoreHandle_t* pGetI2Mutex() {
    return &I2C_done;
}

void I2CInit() {
    /* Initialize the Events, Mutex and Timers of the I2C */
    I2C_events = xEventGroupCreate();
    I2C_done = xSemaphoreCreateMutex();
    I2C_write_queue = xQueueCreate(1, sizeof(i2c_master_transfer_t*));
    I2C_Timer_Handler = xTimerCreate("I2C Timer",
                                     pdMS_TO_TICKS(MAX_WAITING_TIME), pdFALSE,
                                     NULL,
                                     I2C_restart);

    /*I2C master handler creation*/
    I2C_MasterTransferCreateHandle(I2C1, &g_m_handle, i2c_master_callback,
    NULL);
    /*  Indicates that the I2C it's free to transfer */
    xEventGroupSetBits(I2C_events, I2C_free);
    xSemaphoreGive(I2C_done);

    /* Initialize the Transfer task to be ready to transfer */
    xTaskCreate(I2C_transfer, "I2C Transfer", configMINIMAL_STACK_SIZE, NULL, 5,
                I2C_Transfer);
    /* Not needed anymore because it's just for configuration.*/
    vTaskDelete(NULL);

}

void I2C_transfer() {

    i2c_master_transfer_t *masterXfer;

    /* I2C_0 data block transmission */
    while (1)
    {
        /* Waits for a pointer of the i2c_master_t with the desired for the transfer */
        xQueueReceive(I2C_write_queue, &masterXfer, portMAX_DELAY);

        /* Mutex to indicate that the I2C it's taken */
        xSemaphoreTake(I2C_done, portMAX_DELAY);

        /* Event Group to indicate that the I2C it's bussy */
        xEventGroupWaitBits(I2C_events, I2C_free, pdTRUE, pdTRUE,
        portMAX_DELAY);

        /* Starts the timer of the I2C to prevent a block if the device is not found*/
        xTimerStart(I2C_Timer_Handler, 0);
        I2C_MasterTransferNonBlocking(I2C1, &g_m_handle, masterXfer);

        /* Waits for the I2C to finishes the transfer*/
        xEventGroupWaitBits(I2C_events, I2C_free, pdFALSE, pdTRUE,
        portMAX_DELAY);

        /* waits for the I2C to be free */
        xSemaphoreGive(I2C_done);

        /* Stops the timer in case the transfer is completed */
        xTimerStop(I2C_Timer_Handler, 0);

        /* Releases the heap memory assigned for the queue transfer */
        vPortFree(masterXfer);
    }

}
