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

#define DEBUG_SPI_CONFIGURE_AS_IN_DSPS 0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define SPI_TRANSFER_IN_PROGRESS 1<<0

                    //volatile bool isTransferCompleted = false;
//////////////**OS SPI resources declarations*//////////////////////////
dspi_master_handle_t g_m_handle;
dspi_transfer_t masterXfer;
SemaphoreHandle_t g_SPI_mutex;
EventGroupHandle_t g_SPI_events;


//////////////////**user types definitions*////////////////////////////
typedef enum {
    PTD0_RST, PTD1_CLK, PTD2_DIN, PTD3_DC
} PORTD_SPI_pins_t;

typedef enum {
    PTD1_SCK = 1, PTD2_SOUT
} PORTD_SPI0_pins_t;


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
   // isTransferCompleted = true;
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
#if DEBUG_SPI_CONFIGURE_AS_IN_DSPS
    DSPI_SetFifoEnable(SPI0, false, false);
    DSPI_Enable(SPI0, true);
    DSPI_EnableInterrupts(SPI0, kDSPI_AllInterruptEnable);
#endif
}

void SPI_sendOneByte(uint8_t byte)
{
   // isTransferCompleted = false;
    uint8_t masterTxData[1];
    masterTxData[0] = byte;
    masterXfer.txData = masterTxData;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = sizeof(masterTxData);
    masterXfer.configFlags = kDSPI_MasterCtar0
            | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
   // xEventGroupSetBits(g_SPI_events, SPI_TRANSFER_IN_PROGRESS);

    xSemaphoreTake(g_SPI_mutex,portMAX_DELAY);     /**MUTEX take*/
    DSPI_MasterTransferNonBlocking(SPI0, &g_m_handle, &masterXfer);
    xSemaphoreGive(g_SPI_mutex);                    /**MUTEX release in callback */

    xEventGroupWaitBits(g_SPI_events, SPI_TRANSFER_IN_PROGRESS, pdTRUE, pdTRUE, portMAX_DELAY);
    /*
    while (!isTransferCompleted)            ///cambiar por un event group?
    {
    }*/

}
