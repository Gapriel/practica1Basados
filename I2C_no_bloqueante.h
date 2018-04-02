/*
 * I2C_no_bloqueante.h
 *
 *  Created on: Mar 24, 2018
 *      Author: Avelar Díaz José Francisco      ie704217@iteso.mx
 *      Author: Santamaría García Gabriel       ie699356@iteso.mx
 */

#ifndef I2C_NO_BLOQUEANTE_H_
#define I2C_NO_BLOQUEANTE_H_
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
#include "timers.h"




/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Release the Transfer of the I2C if the dive is not found.
     \param[in] TimerHandle_t
    \return void
 */
void I2C_restart(TimerHandle_t handler);




/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Function used to get the address of the EventGroupHandle of the I2C
    \return EventGroupHandle_t*  The address of the handler of the events.
 */
EventGroupHandle_t* pGetI2CEvents();



/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Function to get the SemaphoreHandler of the Mutex of the I2C
    \return SemaphoreHandle_t* Address of the I2C Mutex Handler
 */
SemaphoreHandle_t* pGetI2Mutex();


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Task used to configure handlers and tasks of the I2C
    \return void
 */
void I2CInit();


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Function to get the Queue Handler of the transfer function of the I2C
    \return QueueHandle_t* Address of the I2C queue handler
 */
QueueHandle_t* pGetI2CHandler();


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Task that uses the I2C_MasterTransferNonBlocking function protected by Events and Mutex
    \brief  it makes the transfer of the received i2c_transfer_t* by a xQueueReceived of the I2C Transfer Handler
    \return void
 */
void I2C_transfer() ;

#endif /* I2C_NO_BLOQUEANTE_H_ */
