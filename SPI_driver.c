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
 * SPI_driver.c
 *
 *  Created on: Mar 10, 2018
 *      Author: Avelar Díaz José Francisco      ie704217@iteso.mx
 *      Author: Santamaría García Gabriel       ie699356@iteso.mx
 */

#include "SPI_driver.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "NVIC.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "LCDNokia5110.h"
#include "queue.h"

#define DEBUG_SPI_CONFIGURE_AS_IN_DSPS 0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define SPI_TRANSFER_IN_PROGRESS 1<<0

//////////////**OS SPI resources declarations*//////////////////////////
dspi_master_handle_t g_m_handle_I2C;
dspi_transfer_t masterXfer_SPI;
SemaphoreHandle_t g_SPI_mutex;
EventGroupHandle_t g_SPI_events;

QueueHandle_t SPI_queue;

//////////////////**user types definitions*////////////////////////////
typedef enum {
    PTD0_RST, PTD1_CLK, PTD2_DIN, PTD3_DC
} PORTD_SPI_pins_t;

typedef enum {
    PTD1_SCK = 1, PTD2_SOUT
} PORTD_SPI0_pins_t;

/////////////////////**module callbacks*///////////////////////////////
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle,
                             status_t status, void *userData) {
    if (status == kStatus_Success)
    {
        __NOP();
    }
    BaseType_t xHigherPriorityTaskWoken;
    xEventGroupSetBitsFromISR(g_SPI_events, SPI_TRANSFER_IN_PROGRESS,
                              &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//////////////////**mechanisms definitions*/////////////////////////
void SPI_configuration() {
    g_SPI_mutex = xSemaphoreCreateMutex(); /* SPI0 MUTEX creation */
    g_SPI_events = xEventGroupCreate(); /* SPI0 event group bits creation */

    /* Created with the size of a void* because it's received and casted as
     *  a dpsi_master_config_t pointer */
    SPI_queue = xQueueCreate(1, sizeof(void*));

    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_Spi0);

    port_pin_config_t config_spi = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTD, PTD1_SCK, &config_spi);
    PORT_SetPinConfig(PORTD, PTD2_SOUT, &config_spi);

    dspi_master_config_t masterConfig;
    DSPI_MasterGetDefaultConfig(&masterConfig);
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetBusClkFreq());
    NVIC_enableInterruptAndPriotity(SPI0_IRQn, 7);

    /*Initialize the print tasks of the DSPI     */
    xTaskCreate(task_SPI_print, "spi printing task", configMINIMAL_STACK_SIZE,
                (void*) NULL, 5, NULL);
    DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle_I2C,
                                    DSPI_MasterUserCallback, NULL);
}


void SPI_sendOneByte(uint8_t byte) {
    /* Assigns the received value to the tx buffer of the masterXfer structure*/
    masterXfer_SPI.txData = &byte;
    masterXfer_SPI.rxData = NULL;
    masterXfer_SPI.dataSize = sizeof(uint8_t);
    masterXfer_SPI.configFlags = kDSPI_MasterCtar0
            | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

    /* takes the DSPI mutex to indicate it's bussy*/
    xSemaphoreTake(g_SPI_mutex, portMAX_DELAY); /**MUTEX take*/

    /* Starts the DSPI trasnfer and send the information*/
    DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle_I2C, &masterXfer_SPI);
    xSemaphoreGive(g_SPI_mutex); /**MUTEX release in callback */

    /*Waits for finalization of the transfer of the DSPI */
    xEventGroupWaitBits(g_SPI_events, SPI_TRANSFER_IN_PROGRESS, pdTRUE, pdTRUE, portMAX_DELAY);

    /*The DSPI transfer is stopped when finished*/
    DSPI_StopTransfer(SPI0);
}

void task_SPI_print(void*args) {
    vTaskDelay(pdMS_TO_TICKS(500));
    static SPI_msg_t *message;
    for (;;)
    {/* Waits for a message to send by the spi*/
        xQueueReceive(SPI_queue, &message, portMAX_DELAY);
        /* If clear the display it's needed*/
        if (pdTRUE == message->LCD_to_be_clear)
        {
            LCDNokia_clear();
        } else
        {
            /*Else, calls the send string function of the LCD nokia*/
            LCDNokia_sendString(message->string);
        }
        /*Free reserved memory for the message queue*/
        vPortFree(message);
    }

}

/* Returns the address of the SPI Transfer Handler */
QueueHandle_t* pGetSPIHandler() {
    return &SPI_queue;
}
