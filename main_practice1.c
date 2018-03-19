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
 * @file    practice1.c
 * @brief   Application entry point.
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "FreeRTOS.h"
#include "task.h"
#include "SysConfiguration.h"
#include "fsl_debug_console.h"

#include "SPI_driver.h"
#include "I2C_driver.h"
#include "LCDNokia5110.h"

#define STACK_SIZE 150

void dummy_task1(void* args)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(2000);
    uint8_t counter = 0;
    for (;;)
    {
        PRINTF("IN TASK 1: %i +++++++++++++++\r\n", counter);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(2000));
        // vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void dummy_task2(void* args)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);
    uint8_t counter = 0;
    for (;;)
    {
        PRINTF("IN TASK 2: %i ***************\r\n", counter);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
        //    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void dummy_task3(void* args)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(4000);
    uint8_t counter = 0;
    for (;;)
    {
        PRINTF("IN TASK 3: %i ---------------\r\n", counter);
        counter++;
        vTaskDelay(pdMS_TO_TICKS(4000));
        //vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

int main(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    xTaskCreate(SystemConfiguration, "System initial configuration", STACK_SIZE,
                (void*) NULL, 4, NULL); /**System configuration task creation*/
   // xTaskCreate(dummy_task1, "tarea 1", STACK_SIZE, (void*) NULL, 1, NULL);
   // xTaskCreate(dummy_task2, "tarea 2", STACK_SIZE, (void*) NULL, 3, NULL);
   // xTaskCreate(dummy_task3, "tarea 3", STACK_SIZE, (void*) NULL, 1, NULL);
    //xTaskCreate(probandoSPI, "prueba", STACK_SIZE, (void*) NULL, 2, NULL);
    xTaskCreate(writes, "prueba 2", STACK_SIZE, (void*) NULL, 1, NULL);
    xTaskCreate(readd, "prueba 2", STACK_SIZE, (void*) NULL, 1, NULL);
    vTaskStartScheduler(); /**FREERTOS scheduler control taking*/
    for (;;)
        ; /**practice superloop; execution doesn't reach this point*/
    return 0;
}
