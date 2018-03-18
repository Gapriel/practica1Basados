/*
 * SPI_driver.c
 *
 *  Created on: Mar 10, 2018
 *      Author:
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
#define STRING_MAX_LENGTH 72

//////////////**OS SPI resources declarations*//////////////////////////
dspi_master_handle_t g_m_handle;
dspi_transfer_t masterXfer;
SemaphoreHandle_t g_SPI_mutex;
EventGroupHandle_t g_SPI_events;
static volatile QueueHandle_t SPI_queue;

//////////////////**user types definitions*////////////////////////////
typedef enum {
    PTD0_RST, PTD1_CLK, PTD2_DIN, PTD3_DC
} PORTD_SPI_pins_t;

typedef enum {
    PTD1_SCK = 1, PTD2_SOUT
} PORTD_SPI0_pins_t;

typedef struct {
    uint8_t string_to_be_printed[STRING_MAX_LENGTH];
    uint8_t LCD_to_be_clear;
} SPI_msg_t;


void task_SPI_print();
/////////////////////**module callbacks*///////////////////////////////
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle,
                             status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        __NOP();
    }
    BaseType_t xHigherPriorityTaskWoken;
    xEventGroupSetBitsFromISR(g_SPI_events, SPI_TRANSFER_IN_PROGRESS, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


//////////////////**mechanisms definitions*/////////////////////////
void SPI_configuration()
{
    g_SPI_mutex = xSemaphoreCreateMutex();      //SPI0 MUTEX creation
    g_SPI_events = xEventGroupCreate();        //SPI0 event group bits creation
    CLOCK_EnableClock(kCLOCK_PortD);
    CLOCK_EnableClock(kCLOCK_Spi0);
    port_pin_config_t config_spi = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTD, PTD1_SCK, &config_spi);
    PORT_SetPinConfig(PORTD, PTD2_SOUT, &config_spi);
    SPI_queue = xQueueCreate(3, sizeof(void*)); /**IPC queue created with the size of a void pointer*/
    dspi_master_config_t masterConfig;
    DSPI_MasterGetDefaultConfig(&masterConfig);
#if DEBUG_SPI_CONFIGURE_AS_IN_DSPS
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveLow;
    masterConfig.ctarConfig.baudRate = 9600;
#endif
    DSPI_MasterInit(SPI0, &masterConfig, CLOCK_GetBusClkFreq());
    NVIC_enableInterruptAndPriotity(SPI0_IRQn, 7);
    DSPI_MasterTransferCreateHandle(SPI0, &g_m_handle, DSPI_MasterUserCallback,
                                    NULL);
    NVIC_enableInterruptAndPriotity(SPI0_IRQn,5);
    xTaskCreate(task_SPI_print, "spi printing task", 150, (void*)NULL, 1, NULL);
#if DEBUG_SPI_CONFIGURE_AS_IN_DSPS
    DSPI_SetFifoEnable(SPI0, false, false);
    DSPI_Enable(SPI0, true);
    DSPI_EnableInterrupts(SPI0, kDSPI_AllInterruptEnable);
#endif
}

void SPI_sendOneByte(uint8_t byte)
{
    uint8_t masterTxData[1];
    masterTxData[0] = byte;
    masterXfer.txData = masterTxData;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = sizeof(masterTxData);
    masterXfer.configFlags = kDSPI_MasterCtar0
            | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;

    xSemaphoreTake(g_SPI_mutex,portMAX_DELAY);     /**MUTEX take*/
    DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle, &masterXfer);
    xSemaphoreGive(g_SPI_mutex);                    /**MUTEX release in callback */

    xEventGroupWaitBits(g_SPI_events, SPI_TRANSFER_IN_PROGRESS, pdTRUE, pdTRUE, portMAX_DELAY);
    DSPI_StopTransfer(SPI0);
}


void task_SPI_print(){
    static SPI_msg_t *message;
    for(;;){
        xQueueReceive(SPI_queue, &message, portMAX_DELAY);
        if(pdTRUE == message->LCD_to_be_clear){
           LCDNokia_clear();
        }else{
           //uint8_t text = message->string_to_be_printed;
           LCDNokia_sendString(message->string_to_be_printed);
        }
        vPortFree(message);
    }

}


void probandoSPI(void* args)
{
    static uint8_t count = 0;

    SPI_msg_t * message;
    static uint8_t mensaje[] = "hola";
    static uint8_t vacio[] = "";
    for (;;)
    {
        message = pvPortMalloc(sizeof(SPI_msg_t)); /**memory is reserved for the message to be sent*/
        if(count >= 18){
            count = 0;
            message->LCD_to_be_clear = 1;
            message->string_to_be_printed[0] = "";
        }else{
            message->string_to_be_printed[0] = 'h';
            message->string_to_be_printed[1] = 'o';
            message->string_to_be_printed[2] = 'l';
            message->string_to_be_printed[3] = 'a';
            message->LCD_to_be_clear = 0;
            count++;
        }
        xQueueSend(SPI_queue,&message,portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(6000));
    }
}
