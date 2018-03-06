/*
 * I2C.h
 *
 *  Created on: Mar 5, 2018
 *      Author: gabrielpc
 */

#ifndef I2C_H_
#define I2C_H_

#include "fsl_i2c.h"

void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData);

#endif /* I2C_H_ */
