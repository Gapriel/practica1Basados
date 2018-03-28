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


#define STRING_MAX_LENGTH 30

typedef struct {
    //uint8_t string_to_be_printed[STRING_MAX_LENGTH];
    uint8_t *string;

    uint8_t LCD_to_be_clear;
} SPI_msg_t;

void SPI_configuration();
void SPI_sendOneByte(uint8_t byte);
void probandoSPI(void* args);

#endif /* SPI_DRIVER_H_ */
