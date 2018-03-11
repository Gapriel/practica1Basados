/*
 * SPI_driver.h
 *
 *  Created on: Mar 10, 2018
 *      Author: gabrielpc
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include "DataTypeDefinitions.h"
#include "stdint.h"

void SPI_configuration();
void SPI_sendOneByte(uint8_t byte);

#endif /* SPI_DRIVER_H_ */
