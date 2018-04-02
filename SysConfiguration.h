/*
 * SysConfiguration.h
 *
 *  Created on: Mar 5, 2018
 *      Author: gabrielpc
 */

#ifndef SYSCONFIGURATION_H_
#define SYSCONFIGURATION_H_
#include "FreeRTOS.h"

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  Task to configure I2C, DSPI and GPIO buttons
    \return void
 */
void SystemConfiguration(void* args);

#endif /* SYSCONFIGURATION_H_ */
