/*
 * terminal_menus.c
 *
 *  Created on: Mar 24, 2018
 *      Author: Paco
 */

#include "terminal_menus.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_uart.h"
#include "fsl_debug_console.h"
#include "UART_personal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FIFO.h"
#include "fsl_dspi.h"
#include "SPI_driver.h"
#include "UART_personal.h"
#include "semphr.h"
#include "event_groups.h"
#include "I2C_no_bloqueante.h"

#define PTA1 (1<< 1)
#define PTA2 (1<< 2)
#define PTB23 (1 << 23)
#define PTB9 (1 << 9)
#define PC_UART 0
#define BLUE_UART 1
#define RETURN 'z'
#define ENTER 'x'
#define ASCII_NUMBER_OFFSET 0x30
#define INPUT_SPACES 5
#define MENUS_QUANTITY 11
#define TensMask 0x30
#define UnitsMask 0x0F
#define YearsMask 0xC0
#define TimeFormatMaskNegate 0x7F
#define TimeFormatMaskSet 0x80
#define AMFMBitMask 0x40
#define MINUTESSECONDSLOW 0x0F
#define MINUTESSECONDSHIGH 0xF0
#define FREE_MEM_EVENT (1<<0)
#define FREE_RTC_EVENT (1<< 1)

#define I2C_free (1<< 0)
#define I2C_data_ready (1 << 1)

EventGroupHandle_t UpdateValueByButtons_Events;
#define DAY_UP (1 << 0)
#define DAY_DOWN (1 << 1)
#define MONTH_UP (1 << 2)
#define MONTH_DOWN (1 << 3)
#define MINUTES_UP (1 << 0)
#define MINUTES_DOWN (1 << 1)
#define HOURS_UP (1 << 2)
#define HOURS_DOWN (1 << 3)

#define I2C_free (1<< 0)
#define I2C_data_ready (1 << 1)

#define STRING_SIZE 77

typedef enum {
    MEMORY_TASK, RTC_TASK, SPI_TASK
} task_type;

QueueHandle_t* pSPI_queue;

SemaphoreHandle_t Interface_mutex;

QueueHandle_t* pI2C_write_queue;
SemaphoreHandle_t* pI2C_done;
EventGroupHandle_t* pI2C_events;
EventGroupHandle_t SubTasks_Events;
BooleanType DateFlag;

typedef struct {
    struct {
        uint8_t terminal_position[9];
    } Positions[INPUT_SPACES];
} Menu_terminal_positions_t;

typedef struct {
    uint8_t menuPositionLineIndex;
    uint8_t linePositionIndex;
    uint8_t MAXwritableLines;
    uint8_t initialPositioning;
    uint8_t lINECharsExtrictNumber[INPUT_SPACES];
    Menu_terminal_positions_t menuUartPositions;
    uint16_t charactersPerInputSpace[INPUT_SPACES];
    uint8_t validCharacter;
} LineMovementPack_t;

void I2C_restart(TimerHandle_t handler)
{
    pI2C_events = pGetI2CEvents();
    xEventGroupSetBits(*pI2C_events, I2C_free);
    xSemaphoreGive(*pI2C_done);

}

EventGroupHandle_t* pGetSubTasksEvents()
{
    return &SubTasks_Events;
}

SemaphoreHandle_t* pGetInterfaceMutex()
{
    return &Interface_mutex;
}

void PORTA_IRQHandler() {
    uint32_t InterruptFlags;

    BaseType_t pxHigherPriorityTaskWoken;
    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOA);
    if (PTA1 == (InterruptFlags & PTA1))
    {
        xEventGroupSetBitsFromISR(UpdateValueByButtons_Events, MONTH_DOWN, pxHigherPriorityTaskWoken);
        GPIO_ClearPinsInterruptFlags(GPIOA, PTA1);
    } else if (PTA2 == (InterruptFlags & PTA2))
    {
        xEventGroupSetBitsFromISR(UpdateValueByButtons_Events, DAY_DOWN, pxHigherPriorityTaskWoken);
        GPIO_ClearPinsInterruptFlags(GPIOA, PTA2);
    }
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void PORTB_IRQHandler() {
    uint32_t InterruptFlags;
    BaseType_t pxHigherPriorityTaskWoken;
    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOB);
    if (PTB9 == (InterruptFlags & PTB9))
    {
        xEventGroupSetBitsFromISR(UpdateValueByButtons_Events, MONTH_UP, pxHigherPriorityTaskWoken);
        GPIO_ClearPinsInterruptFlags(GPIOB, PTB9);

        PRINTF("del B9");
    } else if (PTB23 == (InterruptFlags & PTB23))
    {
        xEventGroupSetBitsFromISR(UpdateValueByButtons_Events, DAY_UP, pxHigherPriorityTaskWoken);

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB23);
    }

    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

//quoting: Gonzalez, E., Santamaria, G. ; DSP's 2017
typedef enum { /**enum used to seamlessly call the different terminal menus */
    MainMenu,
    ReadMemoryI2C,
    WriteMemoryI2C,
    StablishHour,
    StablishDate,
    TimeFormat,
    ReadHour,
    ReadDate,
    Terminal2Communication,
    LCDEcho,
    ErrorScreen
} MenusType;

typedef struct { /**struct used to contain the various menus string to be printed*/
    uint8_t Strings_quantity; /**stores how many strings will be printed*/
    struct {
        uint8_t String[STRING_SIZE]; /**stored string*/
        uint8_t positionXYCommand[10]; /**stores the printing positions*/
    } Strings[50]; /**stores the strings that correspond with a menu*/
} TerminalMenuType_t;
static TerminalMenuType_t Menus[MENUS_QUANTITY] = { //menus strings to be displayed initialization
        /**0*/{ 9, { { "1) Leer Memoria I2C", "\033[3;10H" }, {
            "2) Escribir memoria I2C", "\033[4;10H" }, { "3) Establecer Hora",
            "\033[5;10H" }, { "4) Establecer Fecha", "\033[6;10H" }, {
            "5) Formato de hora", "\033[7;10H" },
            { "6) Leer hora", "\033[8;10H" }, { "7) Leer fecha", "\033[9;10H" },
            { "8) Comunicacion con terminal 2", "\033[10;10H" }, {
                "9) Eco en LCD", "\033[11;10H" } } },
        /**1*/{ 7, { { "1) Leer Memoria I2C", "\033[3;10H" }, {
            "Direccion de lectura: ", "\033[5;10H" }, { "0x"/*"0x0000"*/,
            "\033[5;32H" }, { "Longitud en bytes: ", "\033[6;10H" }, {
            "0x"/*"50"*/, "\033[6;29H" }, { "Contenido: ", "\033[7;10H" }, {
            ""/*"Micros y DSP ITESO 2017"*/, "\033[8;10H" } } },
        /**2*/{ 5, { { "2) Escribir memoria I2C", "\033[4;10H" }, {
            "Direccion de escritura: ", "\033[6;10H" }, { "0x"/*"0x0050"*/,
            "\033[6;34H" }, { "Texto a guardar: ", "\033[7;10H" }, {
            ""/*"Esta es una prueba de escritura"*/, "\033[8;10H" } } },
        /**3*/{ 3, { { "3) Establecer Hora", "\033[5;10H" }, {
            "Escribir hora en hh:mm:ss: ", "\033[7;10H" }, { ""/*"7:30:55"*/,
            "\033[7;37H" } } },
        /**4*/{ 3, { { "4) Establecer Fecha", "\033[6;10H" }, {
            "Escribir fecha en dd/mm/aa: ", "\033[8;10H" }, {
            ""/*"24:12:2017"*/, "\033[8;38H" } } },
        /**5*/{ 5, { { "5) Formato de hora", "\033[7;10H" }, {
            "El formato actual es ", "\033[9;10H" },
            { ""/*"12h"*/, "\033[9;31H" }, {
                "Escoger formato 12 o 24 (1 / 0): ", "\033[10;10H" }, {
                ""/*"s"*/, "\033[10;43H" } } },
        /**6*/{ 3, { { "6) Leer hora", "\033[8;10H" }, { "La hora actual es:",
            "\033[10;10H" }, { ""/*"5:40:54 am"*/, "\033[11;10H" } } },
        /**7*/{ 3, { { "7) Leer fecha", "\033[9;10H" }, { "La fecha actual es:",
            "\033[11;10H" }, { ""/*"20/10/2017"*/, "\033[12;10H" } } },
        /**8*/{ 5/*variable*/, { { "8) Comunicacion con terminal 2",
            "\033[10;10H" }, { ""/*Terminal 1:*/, "\033[12;10H" }, {
            ""/*"Como esta?"*/, "\033[13;10H" }, { ""/*Terminal 2*/,
            "\033[15;10H" }, { ""/*Bien, aqui sufriendo con la practica y tu?*/,
            "\033[16;10H" } } },
        /**9*/{ 3, { { "9) Eco en LCD", "\033[11;10H" }, { "Escribir texto: ",
            "\033[13;10H" }, { ""/*"texto"*/, "\033[14;10H" } } },
        /**10*/{ 6, { { "ERROR", "\033[3;28H" }, {
            "Ocasionado por uno de los siguientes motivos: ", "\033[5;08H" }, {
            "I) Componente faltante", "\033[7;07H" }, {
            "II) Menu de componente ocupado por otra terminal", "\033[8;07H" },
            { "III) Error en la interaccion con un menu", "\033[9;07H" }, {
                "Presione 2 veces = para continuar...", "\033[12;08H" } } } };

/////////////////////////////////////////MECHANISMS DEFINITIONS/////////////////////////////////////////////
void MenuPrinter(uart_struct* uart_struct, uint8_t menuToBePrinted)
{

    uart_transfer_t* toSend_UART;
    uint8_t printedStringIndex = 0;
    uint8_t screen_erease[] = { "\033[2J" }; /**VT100 terminal clear screen command*/
    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
    toSend_UART->data = screen_erease;
    toSend_UART->dataSize = 1;

    xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/

    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
    while (Menus[menuToBePrinted].Strings_quantity > printedStringIndex)
    {
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                Menus[menuToBePrinted].Strings[printedStringIndex].positionXYCommand;
        toSend_UART->dataSize = 1;

        xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/

        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                Menus[menuToBePrinted].Strings[printedStringIndex].String;
        toSend_UART->dataSize = 1;

        xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/

        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        printedStringIndex++;
    }
}

void CharacterValidator(LineMovementPack_t *Pack, uint8_t menuToBePrinted,
                        uint8_t receivedChar)
{

    switch (menuToBePrinted)
    {
        case ReadMemoryI2C:
            if ((('0' <= receivedChar) && ('9' >= receivedChar))
                    || (('A' <= receivedChar) && ('F' >= receivedChar))
                    || (('a' <= receivedChar) && ('f' >= receivedChar)))
            {
                Pack->validCharacter = pdTRUE;
            } else
            {
                Pack->validCharacter = pdFALSE;
            }
        break;
        case WriteMemoryI2C:
            Pack->validCharacter = pdTRUE;
        break;
        case StablishHour:
        case StablishDate:
            if (('0' <= receivedChar) && (receivedChar <= '9'))
            {
                Pack->validCharacter = pdTRUE;
            } else
            {
                Pack->validCharacter = pdFALSE;
            }
        break;
        case TimeFormat:
            if (('0' <= receivedChar) && (receivedChar <= '1'))
            {
                Pack->validCharacter = pdTRUE;
            } else
            {
                Pack->validCharacter = pdFALSE;
            }
        break;
        case ReadHour:
        case ReadDate:
            Pack->validCharacter = pdFALSE;
        break;
        case Terminal2Communication:
        case LCDEcho:
            Pack->validCharacter = pdTRUE;
        break;

    }

}

void CreateMenuTask(uart_struct* uart_struct, uint8_t menuToBeCreated)
{
    switch (menuToBeCreated)
    {
        case ReadMemoryI2C:
            xTaskCreate(TerminalMenus_ReadMemory, "Read Memory Menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);

        break;
        case WriteMemoryI2C:
            xTaskCreate(TerminalMenus_WriteMemory, "Write memory menu",
            configMINIMAL_STACK_SIZE + 20,
                        (void*) uart_struct, 3, NULL);

        break;
        case StablishHour:
            xTaskCreate(TerminalMenus_EstablishRTCHour, "Establish hour menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);

        break;
        case StablishDate:
            xTaskCreate(TerminalMenus_EstablishRTCDate, "Establish date menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);

        break;
        case TimeFormat:
            xTaskCreate(TerminalMenus_EstablishRTCHourFormat,
                        "Establish time format menu", configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);
        break;
        case ReadHour:
            xTaskCreate(TerminalMenus_ReadRTCHour, "Read hour menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);
        break;
        case ReadDate:
            xTaskCreate(TerminalMenus_ReadRTCDate, "read date menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);
        break;
        case Terminal2Communication:
            xTaskCreate(TerminalMenus_TerminalsCommunication,
                        "terminals communication menu",
                        configMINIMAL_STACK_SIZE + 50,
                        (void*) uart_struct, 3, NULL);
        break;
        case LCDEcho:
            xTaskCreate(TerminalMenus_LCDEcho, "LCD echo menu",
            configMINIMAL_STACK_SIZE,
                        (void*) uart_struct, 3, NULL);
        break;
        default:
        break;
    }
}

void CheckIfReturnToMenu(uart_struct* uart_struct, uint8_t receivedChar,
                         task_type type)
{
    if (RETURN == receivedChar)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        xTaskCreate(TerminalMenus_MainMenu, "menuu task",
        configMINIMAL_STACK_SIZE,
                    (void*) uart_struct, 3, NULL); //create menu task
        if (MEMORY_TASK == type)
        {
            xEventGroupSetBits(SubTasks_Events, FREE_MEM_EVENT);
        } else if (RTC_TASK == type)
        {

            xEventGroupSetBits(SubTasks_Events, FREE_RTC_EVENT);
        }
        vTaskDelete(NULL);  //deletes current task
    }
}

void CheckIfLineMovementAndUartEcho(uart_struct* UART_struct,
                                    uint8_t receivedChar,
                                    LineMovementPack_t * lineMoverPack)
{
    uart_transfer_t* toSend_UART;
    if (((lineMoverPack->charactersPerInputSpace[lineMoverPack->menuPositionLineIndex]
            <= lineMoverPack->linePositionIndex) && (ENTER == receivedChar))
            || (pdFALSE == lineMoverPack->initialPositioning))
    {
        if (pdTRUE == lineMoverPack->initialPositioning)
        {
            lineMoverPack->menuPositionLineIndex++; /**increases the MenuUartPositions index for the next field*/
        }
        lineMoverPack->initialPositioning = pdTRUE;
        lineMoverPack->linePositionIndex = 0;
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->dataSize = 1;
        toSend_UART->data =
                &lineMoverPack->menuUartPositions.Positions[lineMoverPack->menuPositionLineIndex];
        xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
    }
    if ((pdTRUE == lineMoverPack->validCharacter)
            && (lineMoverPack->MAXwritableLines
                    > lineMoverPack->menuPositionLineIndex))
    {
        if (lineMoverPack->charactersPerInputSpace[lineMoverPack->menuPositionLineIndex]
                <= lineMoverPack->linePositionIndex)
        {
            if (pdFALSE
                    == lineMoverPack->lINECharsExtrictNumber[lineMoverPack->menuPositionLineIndex])
            {
                lineMoverPack->linePositionIndex++;

                FIFO_push(lineMoverPack->menuPositionLineIndex, receivedChar);
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
                toSend_UART->dataSize = 1;
                uint8_t buffer[2] = { 0, '\0' };
                buffer[0] = receivedChar;
                toSend_UART->data = buffer;
                xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                           portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
            }
        } else
        {
            lineMoverPack->linePositionIndex++;
            FIFO_push(lineMoverPack->menuPositionLineIndex, receivedChar);
            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->dataSize = 1;
            uint8_t buffer[2] = { 0, '\0' };
            buffer[0] = receivedChar;
            toSend_UART->data = buffer;
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        }
    }
}

uint16_t ASCII_TO_uint8_t(uint8_t fifo_number, uint8_t fifo_pops)
{
    uint8_t ascii_address_index;
    uint16_t address = 0;
    uint8_t fifo_value;
    for (ascii_address_index = 0; ascii_address_index < fifo_pops;
            ascii_address_index++)
    {
        fifo_value = FIFO_pop(fifo_number);
        if (('0' <= fifo_value) && (fifo_value <= '9'))
        {
            fifo_value -= '0';
        } else if (('A' <= fifo_value) && (fifo_value <= 'F'))
        {
            fifo_value -= ('A' - 10);
        }
        address += (fifo_value << (4 * (fifo_pops - ascii_address_index - 1)));
    }
    FIFO_pop(fifo_number);
    return address;
}

//////////////////////////////////////////MENUS FUNCTIONS BODIES////////////////////////////////////////////
void TerminalMenus_MainMenu(void* args)
{
    pI2C_write_queue = pGetI2CHandler();
    pSPI_queue = pGetSPIHandler();
    pI2C_done = pGetI2Mutex();

    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    vTaskDelay(pdMS_TO_TICKS(500));
    for (;;)
    {
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, MainMenu); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;

        CreateMenuTask(UART_struct, charReceived - 0x30);
        vPortFree(received_UART);
        vTaskDelete(NULL);
    }
}

void TerminalMenus_ReadMemory(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_MEM_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    FIFO_clearFIFOs();
    LineMovementPack_t LineMover = { 0, 0, 2,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, { { { "\033[5;34H" }, { "\033[6;31H" },
        { "\033[8;10H" } } }, //struct with constant position reference used in memory read menu
        { 4, 2, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, ReadMemoryI2C); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART);

        CharacterValidator(&LineMover, ReadMemoryI2C, charReceived);
        CheckIfReturnToMenu(UART_struct, charReceived, MEMORY_TASK);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);
        if (2 <= LineMover.menuPositionLineIndex)
        {
            i2c_master_transfer_t *masterXfer_I2C_read; // = &dummy_i2c;
            masterXfer_I2C_read = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            uint8_t BytesToRead = (uint8_t) ASCII_TO_uint8_t(1, 2);
            uint16_t subaddress = ASCII_TO_uint8_t(0, 4);
            uint8_t read_buffer[50] = { 0 };
            masterXfer_I2C_read->data = read_buffer;
            masterXfer_I2C_read->dataSize = BytesToRead;
            masterXfer_I2C_read->direction = kI2C_Read;
            masterXfer_I2C_read->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_read->slaveAddress = 0x51;
            masterXfer_I2C_read->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_read->subaddressSize = 2;
            read_buffer[BytesToRead] = '\0';

            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_read, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(100));
            uart_transfer_t* toSend_UART;
            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->data = read_buffer;
            toSend_UART->dataSize = 1;
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
void TerminalMenus_WriteMemory(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_MEM_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    FIFO_clearFIFOs();
    LineMovementPack_t LineMover = { 0, 0, 2,
    pdFALSE, { pdTRUE, pdFALSE, pdTRUE }, {
        { { "\033[6;36H" }, { "\033[8;10H" } } }, //struct with constant position reference used in memory write menu
        { 4, 1, 0 } };
    for (;;)
    {
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, WriteMemoryI2C); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART);

        CharacterValidator(&LineMover, WriteMemoryI2C, charReceived);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

        CheckIfReturnToMenu(UART_struct, charReceived, MEMORY_TASK);

        if (2 <= LineMover.menuPositionLineIndex)
        {
            i2c_master_transfer_t *masterXfer_I2C_write;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            uint16_t subaddress = ASCII_TO_uint8_t(0, 4);
            FIFO_stacks_t* local_fifo = FIFO_stacks_address(1);
            masterXfer_I2C_write->data = local_fifo->FIFO_array;
            masterXfer_I2C_write->dataSize = local_fifo->FIFO_index;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x51;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 2;
            xSemaphoreGive(*pI2C_done);
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(50));

        }

    }
}

void TerminalMenus_EstablishRTCHour(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_RTC_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);

    uint8_t hours = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;

    uint8_t time_buffer[6] = { 0, 0, 0, 0, 0, 0 };
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, {
        { { "\033[7;37H" }, { "\033[7;37H" } } }, //struct with constant position reference used in memory read menu
        { 6, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, StablishHour); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART);
        CharacterValidator(&LineMover, StablishHour, charReceived);
        CheckIfReturnToMenu(UART_struct, charReceived, RTC_TASK);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

        if (1 <= LineMover.menuPositionLineIndex)
        {
            uint8_t index;
            for (index = 0; index < 6; index++)
            {
                time_buffer[index] = FIFO_pop(0) - 0x30;
            }
            hours = (time_buffer[0] << 4) + time_buffer[1];
            minutes = (time_buffer[2] << 4) + time_buffer[3];
            seconds = (time_buffer[4] << 4) + time_buffer[5];

            i2c_master_transfer_t *masterXfer_I2C_write;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            uint16_t subaddress = 0x02;/* segundos;*/
            masterXfer_I2C_write->data = &seconds;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;

            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);

            vTaskDelay(pdMS_TO_TICKS(50));
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            subaddress = 0x03;
            masterXfer_I2C_write->data = &minutes;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;

            vTaskDelay(pdMS_TO_TICKS(50));
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);

            vTaskDelay(pdMS_TO_TICKS(50));
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            subaddress = 0x04;

            uint8_t readBuffer = 0;
            uint8_t Hours = 0;
            masterXfer_I2C_write->data = &readBuffer;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Read;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);

            readBuffer = readBuffer & 0xC0;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            Hours = (readBuffer) | (time_buffer[0] << 4);
            Hours += time_buffer[1];
            masterXfer_I2C_write->data = &Hours;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;

            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(50));

        }
    }
}
void TerminalMenus_EstablishRTCDate(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_MEM_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    uint8_t date_buffer[6] = { 0, 0, 0, 0, 0, 0 };
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, {
        { { "\033[8;38H" }, { "\033[8;38H" } } }, //struct with constant position reference used in memory read menu
        { 6, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, StablishDate); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART);

        CharacterValidator(&LineMover, StablishDate, charReceived);
        CheckIfReturnToMenu(UART_struct, charReceived, RTC_TASK);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

        if (1 == LineMover.menuPositionLineIndex)
        {
            uint8_t days;
            uint8_t months;
            uint8_t years;
            uint8_t index;
            for (index = 0; index < 6; index++)
            {
                date_buffer[index] = FIFO_pop(0) - 0x30;
            }
            days = (date_buffer[0] << 4) + (date_buffer[1]);
            months = (date_buffer[2] << 4) + (date_buffer[3]);
            years = days + (date_buffer[5] << 6);

            i2c_master_transfer_t *masterXfer_I2C_write;
            uint8_t subaddress = 0x05;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &years;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(50));

            subaddress = 0x06;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &months;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);

            vTaskDelay(pdMS_TO_TICKS(100));

        }

    }
}

void TerminalMenus_EstablishRTCHourFormat(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_RTC_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);

    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    volatile uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, { { { "\033[10;43H" },
        { "\033[10;43H" } } }, //struct with constant position reference used in memory read menu
        { 1, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, TimeFormat); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;

        CharacterValidator(&LineMover, TimeFormat, charReceived);
        CheckIfReturnToMenu(UART_struct, charReceived, RTC_TASK);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

        vPortFree(received_UART);
        if (1 == LineMover.menuPositionLineIndex)
        {
            uint8_t timeFormat = 0;
            uint8_t readBuffer = 0;
            i2c_master_transfer_t *masterXfer_I2C_write;
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            uint16_t subaddress = 0x04;/* segundos;*/
            masterXfer_I2C_write->data = &readBuffer;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Read;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);

            vTaskDelay(pdMS_TO_TICKS(20));
            uint8_t hours_provitional;
            uint8_t character = FIFO_pop(0);
            if ('1' == character)
            {
                timeFormat = readBuffer | TimeFormatMaskSet;

            } else
            {
                // 0 es para formato de 24 horas
                if (pdFALSE == (readBuffer >> 7))
                { //Si ya estaba en formato de 24 horas
                    timeFormat = readBuffer; //| TimeFormatMaskSet;

                } else
                {
                    hours_provitional = ((readBuffer & 0x3F) >> 4) * 10
                            + ((readBuffer & 0xF));
                    if (hours_provitional == 12)
                    {
                        readBuffer = readBuffer;
                    } else
                    {
                        hours_provitional = hours_provitional
                                + 12 * (((readBuffer >> 6)) & 0x1);
                        uint8_t dec_hours = hours_provitional / 10;
                        uint8_t u_hours = hours_provitional - dec_hours * 10;

                        readBuffer = ((dec_hours << 4) + u_hours);
                    }
                    timeFormat = readBuffer & TimeFormatMaskNegate;
                }

            }

            vTaskDelay(pdMS_TO_TICKS(100));
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &timeFormat;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;

            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(100));

        }

    }
}

void TerminalMenus_ReadRTCHour(void* args) {
    xEventGroupWaitBits(SubTasks_Events, FREE_RTC_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uint8_t charReceived = 0;
    uint8_t time_buffer_variable[3];
    uint8_t printing_time_chars[9] = {"  ERROR "};
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, { { { "\033[11;10H" } } }, //struct with constant position reference used in memory read menu
        { 0, 0 } };
    volatile uart_transfer_t* received_UART = NULL;

    for (;;)
    {

        xLastWakeTime = xTaskGetTickCount();
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, ReadHour); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }

        if (pdPASS
                == xQueueReceive(*UART_struct->UART_receive_Queue,
                                 &received_UART, 0))
        {

            charReceived = *received_UART->data;
            //  vPortFree( &received_UART);
            CharacterValidator(&LineMover, ReadHour, charReceived);
            CheckIfReturnToMenu(UART_struct, charReceived, RTC_TASK);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);

        } else
        {

            i2c_master_transfer_t *masterXfer_I2C_write;
            uint8_t index;
            uint16_t subaddress = 0x02;
            for (index = 0; index < 3; index++)
            {
                masterXfer_I2C_write = pvPortMalloc(
                        sizeof(i2c_master_transfer_t*));
                masterXfer_I2C_write->data = &time_buffer_variable[index];
                masterXfer_I2C_write->dataSize = 1;
                masterXfer_I2C_write->direction = kI2C_Read;
                masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
                masterXfer_I2C_write->slaveAddress = 0x50;
                masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
                masterXfer_I2C_write->subaddressSize = 1;
                xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(*pI2C_done, portMAX_DELAY);
                xSemaphoreGive(*pI2C_done);
                vTaskDelay(pdMS_TO_TICKS(100));
                subaddress++;
            }
            uint8_t EventValues =xEventGroupGetBits( UpdateValueByButtons_Events);
            switch (EventValues){
                case 0:
                    break;
                case HOURS_UP:
                    time_buffer_variable[2] +=1;
                    break;
                case HOURS_DOWN:
                    time_buffer_variable[2] -=1;
                    break;
                case MINUTES_UP:
                    time_buffer_variable[1] +=1;
                    break;
                case MINUTES_DOWN:
                    time_buffer_variable[1] -=1;
                    break;
            }

            xEventGroupClearBits(UpdateValueByButtons_Events, HOURS_UP|HOURS_DOWN|MINUTES_UP|MINUTES_DOWN);

            printing_time_chars[0] = ((time_buffer_variable[2] & TensMask) >> 4)
                    + '0'; /**hours tens*/
            printing_time_chars[1] = (time_buffer_variable[2] & UnitsMask)
                    + '0';
            printing_time_chars[2] = ':';
            printing_time_chars[3] = ((time_buffer_variable[1]
                    & MINUTESSECONDSHIGH) >> 4) + '0';
            printing_time_chars[4] = (time_buffer_variable[1]
                    & MINUTESSECONDSLOW) + '0';
            printing_time_chars[5] = ':';
            printing_time_chars[6] = ((time_buffer_variable[0]
                    & MINUTESSECONDSHIGH) >> 4) + '0';
            printing_time_chars[7] = (time_buffer_variable[0]
                    & MINUTESSECONDSLOW) + '0';
            printing_time_chars[8] = '\0';

            uint8_t testHours = (((printing_time_chars[0] - '0') * 10))
                    + (printing_time_chars[1] - '0');
            if (12 < testHours
                    && (pdTRUE
                            == (time_buffer_variable[2] & TimeFormatMaskSet)
                                    >> 7))
            {
                testHours -= 12;
                uint8_t temporalBuffer = time_buffer_variable[2] & 0xC0;
                temporalBuffer |= AMFMBitMask;
                uint8_t temporalHoursTens = (testHours / 10);
                uint8_t temporalHoursUnits = (testHours
                        - (temporalHoursTens * 10));
                time_buffer_variable[2] = (temporalBuffer
                        | (temporalHoursTens << 4)) + temporalHoursUnits;
                masterXfer_I2C_write = pvPortMalloc(
                        sizeof(i2c_master_transfer_t*));
                masterXfer_I2C_write->data = &time_buffer_variable[2];
                masterXfer_I2C_write->dataSize = 1;
                masterXfer_I2C_write->direction = kI2C_Write;
                masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
                masterXfer_I2C_write->slaveAddress = 0x50;
                masterXfer_I2C_write->subaddress = (uint32_t) 0x04;
                masterXfer_I2C_write->subaddressSize = 1;
                xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(*pI2C_done, portMAX_DELAY);
                xSemaphoreGive(*pI2C_done);
                vTaskDelay(pdMS_TO_TICKS(100));
                printing_time_chars[0] = ((time_buffer_variable[2] & TensMask)
                        >> 4) + '0'; /**hours tens*/
                printing_time_chars[1] = (time_buffer_variable[2] & UnitsMask)
                        + '0';

            }

            uart_transfer_t* toSend_UART;
            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->data = printing_time_chars;
            toSend_UART->dataSize = 1;
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));

            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));

            uint8_t AM[] = { "  AM" };
            uint8_t PM[] = { "  PM" };
            uint8_t HRS[] = { "  HRS" };
            if (pdTRUE == (time_buffer_variable[2] >> 7))
            {
                if (pdTRUE == (((time_buffer_variable[2] >> 6) & 0x1)))
                {
                    toSend_UART->data = PM;
                } else
                {
                    toSend_UART->data = AM;
                }
            } else
            {
                toSend_UART->data = HRS;
            }

            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));

            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->data = &LineMover.menuUartPositions.Positions[0];
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));

            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
}
void TerminalMenus_ReadRTCDate(void* args)
{
    xEventGroupWaitBits(SubTasks_Events, FREE_RTC_EVENT, pdTRUE, pdTRUE,
    portMAX_DELAY);

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t dummy_uart = { 0, 1 };
    uint8_t charReceived = 0;
    uint8_t time_buffer_variable[3];
    uint8_t printing_time_chars[9];
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdTRUE, pdTRUE, pdTRUE }, { { { "\033[12;10H" } } }, //struct with constant position reference used in memory read menu
        { 0, 0 } };
    for (;;)
    {

        uart_transfer_t* received_UART = &dummy_uart;
        xLastWakeTime = xTaskGetTickCount();
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, ReadDate); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART, 0);

        charReceived = *received_UART->data;
        if (0 != charReceived)
        {
            // vPortFree(received_UART);
            CharacterValidator(&LineMover, ReadDate, charReceived);
            CheckIfReturnToMenu(UART_struct, charReceived, RTC_TASK);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);

        } else
        {

            i2c_master_transfer_t *masterXfer_I2C_write;
            uint8_t index;
            uint16_t subaddress = 0x05;
            for (index = 0; index < 2; index++)
            {
                masterXfer_I2C_write = pvPortMalloc(
                        sizeof(i2c_master_transfer_t*));
                masterXfer_I2C_write->data = &time_buffer_variable[index];
                masterXfer_I2C_write->dataSize = 1;
                masterXfer_I2C_write->direction = kI2C_Read;
                masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
                masterXfer_I2C_write->slaveAddress = 0x50;
                masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
                masterXfer_I2C_write->subaddressSize = 1;
                xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(*pI2C_done, portMAX_DELAY);
                xSemaphoreGive(*pI2C_done);
                vTaskDelay(pdMS_TO_TICKS(50));
                subaddress++;
            }
            printing_time_chars[0] = ((time_buffer_variable[0] & TensMask) >> 4)
                    + '0';
            printing_time_chars[1] = (time_buffer_variable[0] & UnitsMask)
                    + '0';
            printing_time_chars[2] = '/';
            printing_time_chars[3] = ((time_buffer_variable[1] & TensMask) >> 4)
                    + '0';
            printing_time_chars[4] = (time_buffer_variable[1] & UnitsMask)
                    + '0';
            printing_time_chars[5] = '/';
            printing_time_chars[6] = '1';
            printing_time_chars[7] = (time_buffer_variable[0] & YearsMask >> 6)
                    + '0';
            printing_time_chars[8] = '\0';

            uart_transfer_t* toSend_UART;
            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->data = printing_time_chars;
            toSend_UART->dataSize = 1;
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));

            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->data = &LineMover.menuUartPositions.Positions[0];
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(50));

            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
}

/**this is the terminals communication menu*/
void TerminalMenus_TerminalsCommunication(void* args)
{
    FIFO_clearFIFOs();  /**clears all previous information in the FIFOs*/
    uart_struct* UART_struct = (uart_struct*) args; /**the tasks package containing important parameters is copied locally*/
    uint8_t firstEntryToMenu = pdFALSE; /**this is used to know if the task has entered for the first time to the execution*/
    uart_transfer_t* received_UART; /**a UART transfer pointer is created for the content to be received through the UART Queue*/
    uint8_t connection_array[22] = { "Terminal x se conecto" }; /**the entry point messaage for the other terminal is declared here*/
    //uint8_t EMPTY[25] = { "                         " };    /***/
    uint8_t charReceived[2] = { 0, '\0' };  /**in this array, index 0, the received char is stored*/
    uart_transfer_t* toSend_UART; /**a UART transfer pointer is created for the content to be sent through the UART Queue*/
    uint8_t localPositionUnits = 0; /**variable to pull the current terminal position units*/
    uint8_t localPositionTens = 0; /**variable to pull the current terminal position tens*/
    uint8_t realLocalPosition = 0; /**variable to concatenate the current terminal position*/
    uint8_t sharedPositionUnits = 0; /**variable to pull the shared terminal position units*/
    uint8_t sharedPositionTens = 0; /**variable to pull the shared terminal position tens*/
    uint8_t realsharedPosition = 0; /**variable to concatenate the shared terminal position */
    uint8_t charToBePrinted[] = { 0, '\0' }; /**in this array, the char to be printed is stored in the index 0*/
    uint8_t receivedFromUart = pdFALSE; /**this variable is used to know if something was received from a queue*/
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdFALSE, pdTRUE, pdTRUE }, { { { "\033[12;10H" } } }, //struct with constant position reference used in memory read menu
        { 10000, 0 } }; /**this structure variable is used  for the checkiflinemovement function, and the charvalidation one */
    for (;;)    /**task superloop*/
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY); /**takes a semaphore for the menu printed in order to protect this execution*/
            firstEntryToMenu = pdTRUE;  /**sets the flag as the task now has entered to the menu*/
            UART_struct->ChatStates->terminalChatStates[UART_struct->uart_number] =
            pdTRUE; /**sets the current terminal state as TRUE as the current terminal is inside the chat now*/

            MenuPrinter(UART_struct, Terminal2Communication); /**prints in the current terminal the chat menu interface*/

            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
            toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
            xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                       portMAX_DELAY);  /**sends content through the UART queue to be printed in the terminal*/
            vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

            connection_array[9] =
                    (UART_0 == UART_struct->uart_number) ? '1' : '2';   /**depending on which terminal is inside the task, the identifier message changes its number*/
            xSemaphoreGive(Interface_mutex); /**gives the previously taken semaphore, as menu printing protection is no longer needed*/
        }

        if (pdFALSE
                == UART_struct->ChatStates->FirstEntry[UART_struct->uart_number])
        {   /**if it's not the first time this particular terminal is inside this task then,*/
            UART_struct->ChatStates->FirstEntry[UART_struct->uart_number] =
                    pdTRUE; /**the terminal entry point flag is set*/
            if ((UART_0 == UART_struct->uart_number)
                    && pdTRUE
                            == (UART_struct->ChatStates->terminalChatStates[UART_1]))
            { /**if this is the terminal 0, and the other terminal is also inside the chat menu then,*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = connection_array; /**the data to be sent will be the terminal identification string*/
                xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**data is sent to the uart 1*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

            } else if ((UART_1 == UART_struct->uart_number)
                    && pdTRUE
                            == (UART_struct->ChatStates->terminalChatStates[UART_0]))
            { /**if this is the terminal 1, and the other terminal is also inside the chat menu then,*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = connection_array; /**the data to be sent will be the terminal identification string*/
                xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**data is sent to the uart 0*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

            }
            if (pdTRUE == (UART_struct->ChatStates->terminalChatStates[UART_0])
                    && pdTRUE
                            == (UART_struct->ChatStates->terminalChatStates[UART_1]))
            { /**if both terminals are in the chat menu*/

                uint8_t messageZonePosition[] = { "\033[04;10H\0" }; /**VT100 command to establish the message indication zone string*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = messageZonePosition; /**the data to be sent will be the position VT100 command for the message printing in the terminal*/
                xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                           &toSend_UART, portMAX_DELAY);    /**sends info to the terminal 1*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = messageZonePosition; /**the data to be sent will be the position VT100 command for the message printing in the terminal*/
                xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**sends info to the terminal 2*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                uint8_t messageZone[] = { "Mensaje a enviar:" };
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = messageZone; /**the data to be sent will be a string*/
                xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**sends info to the terminal 1*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = messageZone; /**the data to be sent will be a string*/
                xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**sends info to the terminal 2*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                UART_struct->ChatStates->chatPosition[3] = '5';
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**sends info to the terminal 1*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                           &toSend_UART, portMAX_DELAY); /**sends info to the terminal 2*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); /**stalls the procedure for a short time to ensure proper flags reading*/
        receivedFromUart = xQueueReceive(*UART_struct->UART_receive_Queue,
                                         &received_UART, pdMS_TO_TICKS(100)); /**receives from the terminal a char*/

        if (pdTRUE == (UART_struct->ChatStates->terminalChatStates[UART_0])
                && pdTRUE
                        == (UART_struct->ChatStates->terminalChatStates[UART_1]))
        { /**if both terminals are in the chat menu*/

            if (pdTRUE == receivedFromUart)
            { /**if something was received in the UART queue*/
                charReceived[0] = *received_UART->data; /**the received char is stored*/
                CharacterValidator(&LineMover, Terminal2Communication,
                                   charReceived[0]); /**the received char is validated to see if its a valid menu char*/

                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                toSend_UART->data =
                        UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint; /**the data to be sent will be the local terminal position*/
                xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                           portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                localPositionUnits =
                        UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6]
                                - 0x30; /**the local position units are converted from ascii to decimal*/
                localPositionTens =
                        UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5]
                                - 0x30; /**the local position tens are converted from ascii to decimal*/
                realLocalPosition = (localPositionTens * 10)
                        + localPositionUnits; /**the previous variables are joined to form the true local horizontal position*/
                realLocalPosition++; /**the local terminal position is increased in 1 space horizontally*/
                localPositionTens = realLocalPosition / 10; /**then the previously modified position is separed in tens*/
                localPositionUnits = realLocalPosition
                        - (localPositionTens * 10); /**then the previously modified position is separed in units*/
                UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5] =
                        (localPositionTens) + '0'; /**the new local position tens are converted back to ascii and stored in the local position string*/
                UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6] =
                        (localPositionUnits) + '0'; /**the new local position units are converted back to ascii and stored in the local position string*/

                if ((RETURN != charReceived[0]) && (ENTER != charReceived[0]))
                { /**if the received char is neither enter nor return*/
                    charReceived[1] = '\0'; /**the received char array is asured to end with an end of char char*/
                    FIFO_push(UART_struct->uart_number, charReceived[0]); /**pushes the local terminal received char in the local terminal fifo*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data = charReceived; /**the data to be sent to the uart will be the received char*/
                    xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                               portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated
                }

                if (RETURN == charReceived[0])
                { /**if the received char is a return to main menu,*/

                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                    xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data will be sent to the terminal 1*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
                    toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                    xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data will be sent to the terminal 2*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    if ((UART_1 == UART_struct->uart_number)
                            && pdTRUE
                                    == (UART_struct->ChatStates->terminalChatStates[UART_0]))
                    { /**if the current terminal to exit is uart 1 and the other terminal is inside the chat menu,*/
                        uint8_t byebye[] = { "Terminal 2 se desconecto" }; /**exiting message to be sent to the other uart*/
                        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                        toSend_UART->data = byebye; /**the data to be sent will be the exiting message*/
                        xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                                   &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 1*/
                        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    } else if ((UART_0 == UART_struct->uart_number)
                            && pdTRUE
                                    == (UART_struct->ChatStates->terminalChatStates[UART_1]))
                    { /**if the current terminal to exit is uart 1 and the other terminal is inside the chat menu,*/
                        uint8_t byebye[] = { "Terminal 1 se desconecto" }; /**exiting message to be sent to the other uart*/
                        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                        toSend_UART->data = byebye; /**the data to be sent will be the exiting message*/
                        xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                                   &toSend_UART, portMAX_DELAY);  /**data is sent to the terminal 2*/
                        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    }

                    /**current terminal chat structure reset*/
                    UART_struct->ChatStates->FirstEntry[UART_struct->uart_number] =
                    pdFALSE;
                    UART_struct->ChatStates->chatPosition[0] = '\033';
                    UART_struct->ChatStates->chatPosition[1] = '[';
                    UART_struct->ChatStates->chatPosition[2] = '1';
                    UART_struct->ChatStates->chatPosition[3] = '2';
                    UART_struct->ChatStates->chatPosition[4] = ';';
                    UART_struct->ChatStates->chatPosition[5] = '1';
                    UART_struct->ChatStates->chatPosition[6] = '0';
                    UART_struct->ChatStates->chatPosition[7] = 'H';
                    UART_struct->ChatStates->chatPosition[8] = '\0';
                    UART_struct->ChatStates->terminalChatStates[UART_struct->uart_number] =
                    pdFALSE;
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[0] =
                            '\033';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[1] =
                            '[';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[2] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[3] =
                            '5';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[4] =
                            ';';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5] =
                            '1';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[7] =
                            'H';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[8] =
                            '\0';

                    vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated
                    CheckIfReturnToMenu(UART_struct, charReceived[0], 2); /**calls the change menu function*/
                }

                if (ENTER == charReceived[0])
                { /**if the received char was an enter (i.e. sent to the chat),*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                    xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 1*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data = UART_struct->ChatStates->chatPosition; /**the data to be sent through the UART is the shared terminal position*/
                    xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 2*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                    uint8_t terminalSpeaker[] = { "Terminal x: " }; /**message that will say which terminal has sent the message*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    terminalSpeaker[9] =
                            (UART_0 == UART_struct->uart_number) ? '1' : '2'; /**according to which terminal is sending
                                                                              the message, the terminal speaker message
                                                                              changes its number at the end of the string*/
                    toSend_UART->data = terminalSpeaker; /**the data to be sent will be the terminal speaker message*/
                    xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 1*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data = terminalSpeaker; /**the data to be sent will be the terminal speaker message*/
                    xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                               &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 2*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                    while (pdTRUE != FIFO_checkIfEmpty(UART_struct->uart_number))
                    { /**while there is something to take out from the fifo,*/
                        charToBePrinted[0] = FIFO_pop(UART_struct->uart_number); /**takes the char to be printed from the fifo*/
                        charToBePrinted[1] = '\0'; /**the char to be printed is assured to end with an end of char char*/
                        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                        toSend_UART->data = charToBePrinted; /**the data to be printed will be the popped char*/
                        xQueueSend(*UART_struct->ChatStates->UART0_send_Queue,
                                   &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 1*/
                        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                        toSend_UART->data = charToBePrinted; /**the data to be printed will be the popped char*/
                        xQueueSend(*UART_struct->ChatStates->UART1_send_Queue,
                                   &toSend_UART, portMAX_DELAY); /**data is sent to the terminal 2*/
                        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    }

                    /**the vertical shared position is increased*/
                    sharedPositionUnits =
                            UART_struct->ChatStates->chatPosition[3] - 0x30;
                    sharedPositionTens =
                            UART_struct->ChatStates->chatPosition[2] - 0x30;
                    realsharedPosition = (sharedPositionTens * 10)
                            + sharedPositionUnits;
                    realsharedPosition += 2;
                    sharedPositionTens = realsharedPosition / 10;
                    sharedPositionUnits = realsharedPosition
                            - (sharedPositionTens * 10);
                    UART_struct->ChatStates->chatPosition[2] =
                            (sharedPositionTens) + '0';
                    UART_struct->ChatStates->chatPosition[3] =
                            (sharedPositionUnits) + '0';

                    /**the local terminal position is set to the top margin*/
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[0] =
                            '\033';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[1] =
                            '[';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[2] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[3] =
                            '5';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[4] =
                            ';';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5] =
                            '1';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[7] =
                            'H';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[8] =
                            '\0';

                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data =
                            UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint; /**the data to be printed will be the local position*/
                    xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                               portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                    uint8_t index; /**index variable for the for loop*/
                    uint8_t writeZoneCleaner[] = { " \0" }; /**char used for cleaning the top space*/
                    for (index = 0; index < 51; index++)
                    { /**loop that clears space by space the upmost space*/
                        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                        toSend_UART->data = writeZoneCleaner; /**the data to be sent will be the cleaning char*/
                        xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                                   portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
                        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
                    }

                    /**the local terminal position is set to the top margin*/
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[0] =
                            '\033';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[1] =
                            '[';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[2] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[3] =
                            '5';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[4] =
                            ';';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5] =
                            '1';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[7] =
                            'H';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[8] =
                            '\0';

                    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*)); /**memory is reserved for a UART transference*/
                    toSend_UART->data =
                            UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint; /**the data to be sent will be the local position*/
                    xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART,
                               portMAX_DELAY); /**sends content through the UART queue to be printed in the terminal*/
                    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/

                    charReceived[0] = '\0'; /**the received char variable is cleaned for future receptions*/
                }
            }
        } else if (pdTRUE
                == UART_struct->ChatStates->terminalChatStates[UART_struct->uart_number])
        { /**if only one terminal is inside the terminal chat*/
            if (pdTRUE == receivedFromUart)
            { /**if something was actually received from the UART Queue*/
                charReceived[0] = *received_UART->data; /**the char received is stored*/
                if (RETURN == charReceived[0]) /**the char received is asured to end with an end of char char*/
                {
                    /**current terminal chat structure reset*/
                    UART_struct->ChatStates->FirstEntry[UART_struct->uart_number] =
                            pdFALSE;
                    UART_struct->ChatStates->chatPosition[0] = '\033';
                    UART_struct->ChatStates->chatPosition[1] = '[';
                    UART_struct->ChatStates->chatPosition[2] = '1';
                    UART_struct->ChatStates->chatPosition[3] = '2';
                    UART_struct->ChatStates->chatPosition[4] = ';';
                    UART_struct->ChatStates->chatPosition[5] = '1';
                    UART_struct->ChatStates->chatPosition[6] = '0';
                    UART_struct->ChatStates->chatPosition[7] = 'H';
                    UART_struct->ChatStates->chatPosition[8] = '\0';
                    UART_struct->ChatStates->terminalChatStates[UART_struct->uart_number] =
                            pdFALSE;
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[0] =
                            '\033';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[1] =
                            '[';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[2] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[3] =
                            '5';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[4] =
                            ';';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[5] =
                            '1';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[6] =
                            '0';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[7] =
                            'H';
                    UART_struct->ChatStates->localPosition[UART_struct->uart_number].localPrint[8] =
                            '\0';

                    CheckIfReturnToMenu(UART_struct, charReceived[0], 2); /**calls the change menu function*/
                }
                vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated
            }
        }

    }
}

void TerminalMenus_LCDEcho(void* args)
{

    uart_struct* UART_struct = (uart_struct*) args;
    SPI_msg_t *charToBeMirrored;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdFALSE, pdTRUE, pdTRUE }, { { { "\033[14;10H" } } }, //struct with constant position reference used in memory read menu
        { 10000, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, LCDEcho); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART);

        if (RETURN == charReceived)
        {
            charToBeMirrored = pvPortMalloc(sizeof(SPI_msg_t*));
            charToBeMirrored->LCD_to_be_clear = pdTRUE;
            xQueueSend(*pSPI_queue, &charToBeMirrored, portMAX_DELAY);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                                       &LineMover);
            CheckIfReturnToMenu(UART_struct, charReceived, SPI_TASK);
        } else if (ENTER != charReceived)
        {
            CharacterValidator(&LineMover, LCDEcho, charReceived);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);
            charToBeMirrored = pvPortMalloc(sizeof(SPI_msg_t*));
            charToBeMirrored->LCD_to_be_clear = pdFALSE;
            uint8_t stringBuffer[2];
            stringBuffer[0] = charReceived;
            stringBuffer[1] = '\0';
            charToBeMirrored->string = stringBuffer;
            xQueueSend(*pSPI_queue, &charToBeMirrored, portMAX_DELAY);
        }
    }
}

void SPIReadHour(void * args)
{
    TickType_t xLastWakeTime; /**variable used to know when was the last wake time*/
    const TickType_t xPeriod = pdMS_TO_TICKS(1000); /**the task period will be of 1s*/
    volatile int8_t time_buffer_variable[3]; /**rtc time reading buffer*/
    uint8_t printing_time_chars[9] = { '0' }; /**array that will hould the hour/date info to be printed on the spi screen*/
    SPI_msg_t *timeString; /**where the printing time chars will be concatenated for posterior sending*/
    i2c_master_transfer_t *masterXfer_I2C_write; /**master transfer variable used for I2C transfers*/
    uint8_t index; /**index variable used for the for loop*/
    vTaskDelay(pdMS_TO_TICKS(1000)); /**1s the task is sent to sleep in order to assure previous correct modules initialization*/
    for (;;)
    { /**task superloop*/
        xLastWakeTime = xTaskGetTickCount(); /**gets the passed ticks*/

        uint16_t subaddress = 0x02; /**the starting subaddress is set to 0x02 as it is the seconds register*/
        for (index = 0; index < 3; index++)
        {
            /**master transfer configuration, memory reservation, mastertransfer is sent to the I2C with
                                                        * its queue, and the subaddress is increased by one*/
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &time_buffer_variable[index];
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Read;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(100));
            subaddress++;
        }

        /**after reading the registers, the hour is properly interpreted and stored, and also converted to ascii*/
        printing_time_chars[0] = ((time_buffer_variable[2] & TensMask) >> 4)
                + '0'; /**hours tens*/
        printing_time_chars[1] = (time_buffer_variable[2] & UnitsMask) + '0';
        printing_time_chars[2] = ':';
        printing_time_chars[3] = ((time_buffer_variable[1] & MINUTESSECONDSHIGH)
                >> 4) + '0';
        printing_time_chars[4] = (time_buffer_variable[1] & MINUTESSECONDSLOW)
                + '0';
        printing_time_chars[5] = ':';
        printing_time_chars[6] = ((time_buffer_variable[0] & MINUTESSECONDSHIGH)
                >> 4) + '0';
        printing_time_chars[7] = (time_buffer_variable[0] & MINUTESSECONDSLOW)
                + '0';
        printing_time_chars[8] = '\0';

        /**hour message zone indicating message is sent to the SPI screen*/
        uint8_t HourString[12] = { "Hour:       " };
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = HourString;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        /**hour string is sent to the SPI screen*/
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = printing_time_chars;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        /**spacing string is sent to the SPI screen*/
        uint8_t SpaceString[12] = { "            " };
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = SpaceString;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        subaddress = 0x05; /**the starting subaddress is set to 0x02 as it is the days register*/
        for (index = 0; index < 3; index++)
        {
            /**master transfer configuration, memory reservation, mastertransfer is sent to the I2C with
                                                                    * its queue, and the subaddress is increased by one*/
            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &time_buffer_variable[index];
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Read;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;
            xQueueSend(*pI2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(*pI2C_done, portMAX_DELAY);
            xSemaphoreGive(*pI2C_done);
            vTaskDelay(pdMS_TO_TICKS(100));
            subaddress++;
        }

        /**after reading the registers, the date is properly interpreted and stored, and also converted to ascii*/
        printing_time_chars[0] = ((time_buffer_variable[0] & TensMask) >> 4)
                + '0';
        printing_time_chars[1] = (time_buffer_variable[0] & UnitsMask) + '0';
        printing_time_chars[2] = '/';
        printing_time_chars[3] = ((time_buffer_variable[1] & TensMask) >> 4)
                + '0';
        printing_time_chars[4] = (time_buffer_variable[1] & UnitsMask) + '0';
        printing_time_chars[5] = '/';
        printing_time_chars[6] = '1';
        printing_time_chars[7] = (time_buffer_variable[0] & YearsMask >> 6)
                + '0';
        printing_time_chars[8] = '\0';

        /**date message zone indicating message is sent to the SPI screen*/
        uint8_t DateString[12] = { "Date:       " };
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = DateString;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        /**date string is sent to the SPI screen*/
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = printing_time_chars;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        /**spacing string is sent to the SPI screen*/
        timeString = pvPortMalloc(sizeof(SPI_msg_t*));
        timeString->LCD_to_be_clear = pdFALSE;
        timeString->string = SpaceString;
        xQueueSend(*pSPI_queue, &timeString, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));

        vTaskDelayUntil(&xLastWakeTime, xPeriod); /**the task is sent to sleep until the 1s period has transcurred*/
    }
}
