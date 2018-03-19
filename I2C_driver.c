/*
 * I2C_driver.c
 *
 *  Created on: Mar 17, 2018
 *      Author: gabrielpc
 */

#include "I2C_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_i2c.h"
#include "semphr.h"
#include "event_groups.h"
#include "NVIC.h"

#define I2C_TRANSFER_IN_PROGRESS 1<<0

typedef enum {
    PTE24_SCL = 24, PTE25_SDA
} PORTE_I2C0_pins_t;

typedef struct {
    uint8_t slave_address;
    i2c_direction_t direction;
    uint32_t sub_address;
    uint8_t sub_address_size;
    uint8_t data_buffer;
    uint8_t data_size;
    i2c_master_transfer_t masterXfer_i2c;
} i2c_msg_t;

void i2c_ReleaseBus();
void task_I2C_write();
void writes();
void readd();

//i2c_master_transfer_t masterXfer_i2c;
i2c_master_handle_t g_m_handle_i2c;
QueueHandle_t I2C_queue;
SemaphoreHandle_t g_I2C_mutex;
EventGroupHandle_t g_I2C_events;

void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                         status_t status, void * userData)
{
    BaseType_t xHigherPriorityTaskWoken;
    if (status == kStatus_Success)
    {

    }
    xSemaphoreGiveFromISR(g_I2C_mutex,&xHigherPriorityTaskWoken);
    //xEventGroupSetBitsFromISR(g_I2C_events, I2C_TRANSFER_IN_PROGRESS,
      //                        &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void I2C_configuration()
{
   // g_I2C_mutex = xSemaphoreCreateMutex();      //SPI0 MUTEX creation
    g_I2C_mutex = xSemaphoreCreateBinary();      //SPI0 MUTEX creation
    g_I2C_events = xEventGroupCreate();        //SPI0 event group bits creation
    I2C_queue = xQueueCreate(3, sizeof(void*)); /**IPC queue created with the size of a void pointer*/
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_I2c1);
    /**I2C module configuration*/
    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister };
    PORT_SetPinConfig(PORTC, 10, &config_i2c); /**I2C_0 SCL configured*/
    PORT_SetPinConfig(PORTC, 11, &config_i2c); /**I2C_0 SDA configured*/
    /**I2C_0 master configuration and initialization*/
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    I2C_MasterInit(I2C1, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
    /**I2C_0 master handle creation*/
    I2C_MasterTransferCreateHandle(I2C1, &g_m_handle_i2c, i2c_master_callback,
                                   NULL);
    NVIC_enableInterruptAndPriotity(I2C1_IRQn, 7);
    xTaskCreate(task_I2C_write, "i2c write", 120, (void*)NULL, 2, NULL);
//
//    i2c_ReleaseBus(); /**bug fixing function provided by NXP*/
//    /**I2C_0 and PORTE clock enabling, as those are the modules required for the I2C operation*/
//    CLOCK_EnableClock(kCLOCK_I2c0);
//    CLOCK_EnableClock(kCLOCK_PortE);
//    /**I2C module configuration*/
//    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
//        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
//        kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister };
//    PORT_SetPinConfig(PORTE, PTE24_SCL, &config_i2c); /**I2C_0 SCL configured*/
//    PORT_SetPinConfig(PORTE, PTE25_SDA, &config_i2c); /**I2C_0 SDA configured*/
//    /**I2C_0 master configuration and initialization*/
//    i2c_master_config_t masterConfig;
//    I2C_MasterGetDefaultConfig(&masterConfig);
//    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
//    /**I2C_0 master handle creation*/
//    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle_i2c, i2c_master_callback,
//                                   NULL);
//    NVIC_enableInterruptAndPriotity(I2C0_IRQn, 7);
//    xTaskCreate(task_I2C_write, "I2C write task", 150, (void*) NULL, 2, NULL);
}

void task_I2C_write(void* args)
{
    //xEventGroupSetBits(g_I2C_events, I2C_TRANSFER_IN_PROGRESS);
    i2c_msg_t * message;
    xSemaphoreGive(g_I2C_mutex);
    i2c_master_transfer_t masterXfer_i2c;
    for (;;)
    {
        xQueueReceive(I2C_queue, &message, portMAX_DELAY);
        //message->masterXfer_i2c.data = &message->data_buffer;
        masterXfer_i2c.slaveAddress = message->slave_address;
        masterXfer_i2c.direction = message->direction;
        masterXfer_i2c.subaddress = message->sub_address;
        masterXfer_i2c.subaddressSize = message->sub_address_size;
        masterXfer_i2c.data = &message->data_buffer;
        masterXfer_i2c.dataSize = message->data_size;
        masterXfer_i2c.flags = kI2C_TransferDefaultFlag;

        //xEventGroupWaitBits(g_I2C_events, I2C_TRANSFER_IN_PROGRESS, pdTRUE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(g_I2C_mutex, portMAX_DELAY); /**MUTEX take*/
        I2C_MasterTransferNonBlocking(I2C1, &g_m_handle_i2c, &message->masterXfer_i2c);
        vPortFree(message);
       // xSemaphoreGive(g_I2C_mutex); /**MUTEX release in callback */


    }
}
//
//void task_I2C_write()
//{
//    static i2c_msg_t *message;
//    for (;;)
//    {
//        xQueueReceive(I2C_queue, &message, portMAX_DELAY);
//
//        masterXfer_i2c.slaveAddress = message->slave_address;
//        masterXfer_i2c.direction = message->direction;
//        masterXfer_i2c.subaddress = message->sub_address;
//        masterXfer_i2c.subaddressSize = message->sub_address_size;
//        masterXfer_i2c.data = &message->data_buffer;
//        masterXfer_i2c.dataSize = message->data_size;
//        masterXfer_i2c.flags = kI2C_TransferDefaultFlag;
//
//        xSemaphoreTake(g_I2C_mutex, portMAX_DELAY); /**MUTEX take*/
//        I2C_MasterTransferNonBlocking(I2C0, &g_m_handle_i2c, &masterXfer_i2c);
//        xSemaphoreGive(g_I2C_mutex); /**MUTEX release in callback */
//
//        vPortFree(message); //liberates the previously reserved memory for the received message
//        xEventGroupWaitBits(g_I2C_events, I2C_TRANSFER_IN_PROGRESS, pdTRUE,
//        pdTRUE,portMAX_DELAY);
//    }
//}

void readd(void*args)
{
    static i2c_msg_t* message;
    uint8_t rd = 3;
   // message->data_buffer = 4;
    message = pvPortMalloc(sizeof(void*));
//    message->masterXfer_i2c.slaveAddress = 0x50;
//        message->masterXfer_i2c.subaddress = 0x04;
//        message->masterXfer_i2c.subaddressSize = 1;
//        message->masterXfer_i2c.data = &rd;
//        message->masterXfer_i2c.dataSize = 1;
//        message->masterXfer_i2c.direction = kI2C_Read;
//        message->masterXfer_i2c.flags = kI2C_TransferDefaultFlag;

    message->slave_address = 0x50;  //RTC slave address
    message->sub_address_size = 1;
    message->data_size = 1;
    message->sub_address = 0x04;
    message->data_buffer = 0;
    message->direction = kI2C_Read;
    xQueueSend(I2C_queue, &message, portMAX_DELAY);
    vTaskSuspend(NULL);
}

void writes(void*args)
{
    static i2c_msg_t* message;
    uint8_t wd = 7;
    message = pvPortMalloc(sizeof(void*));
//    message->masterXfer_i2c.slaveAddress = 0x50;
//    message->masterXfer_i2c.subaddress = 0x04;
//    message->masterXfer_i2c.subaddressSize = 1;
//    message->masterXfer_i2c.data = &wd;
//    message->masterXfer_i2c.dataSize = 1;
//    message->masterXfer_i2c.direction = kI2C_Write;
//    message->masterXfer_i2c.flags = kI2C_TransferDefaultFlag;
    message->slave_address = 0x50;  //RTC slave address
    message->sub_address_size = 1;
    message->data_size = 1;
    message->sub_address = 0x04;
    message->data_buffer = 8;
    message->direction = kI2C_Write;
    xQueueSend(I2C_queue, &message, portMAX_DELAY);
    vTaskSuspend(NULL);
}
