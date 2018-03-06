/*
 * I2C.c
 *
 *  Created on: Mar 5, 2018
 *      Author:
 */

#include "I2C.h"

//////////////////**callback definitions*///////////////////////////
volatile bool g_MasterCompletionFlag = false;

void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData)
{

    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}
