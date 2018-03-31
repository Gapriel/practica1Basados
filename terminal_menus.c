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

extern QueueHandle_t SPI_queue;
extern QueueHandle_t I2C_read_queue;
extern QueueHandle_t I2C_write_queue;
extern SemaphoreHandle_t I2C_done;
extern SemaphoreHandle_t Interface_mutex;

#define STRING_SIZE 77
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

void PORTA_IRQHandler()
{
    uint32_t InterruptFlags;

    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOA);
    if (PTA1 == (InterruptFlags & PTA1))
    {

        GPIO_ClearPinsInterruptFlags(GPIOA, PTA1);
        /*
         * prueba
         */
        PRINTF("del A1");
    } else if (PTA2 == (InterruptFlags & PTA2))
    {
        PRINTF("MAS del A2");

        GPIO_ClearPinsInterruptFlags(GPIOA, PTA2);
    }

}

void PORTB_IRQHandler()
{
    uint32_t InterruptFlags;

    InterruptFlags = GPIO_GetPinsInterruptFlags(GPIOB);
    if (PTB9 == (InterruptFlags & PTB9))
    {

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB9);
        /*
         * prueba
         */
        PRINTF("del B9");
    } else if (PTB23 == (InterruptFlags & PTB23))
    {
        PRINTF("MAS del B23");

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB23);
    }

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

    xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
    while (Menus[menuToBePrinted].Strings_quantity > printedStringIndex)
    {
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                Menus[menuToBePrinted].Strings[printedStringIndex].positionXYCommand;
        toSend_UART->dataSize = 1;

        xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                Menus[menuToBePrinted].Strings[printedStringIndex].String;
        toSend_UART->dataSize = 1;

        xQueueSend(*uart_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY);

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
            configMINIMAL_STACK_SIZE,
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
                        configMINIMAL_STACK_SIZE,
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
void CheckIfReturnToMenu(uart_struct* uart_struct, uint8_t receivedChar)
{
    if (RETURN == receivedChar)
    {
        xTaskCreate(TerminalMenus_MainMenu, "menu task",
        configMINIMAL_STACK_SIZE,
                    (void*) uart_struct, 3, NULL); //create menu task
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
        xQueueSend(*UART_struct->UART_send_Queue, &toSend_UART, portMAX_DELAY);
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

//////////////////////////////////////////MENUS FUNCTIONS BODIES////////////////////////////////////////////
void TerminalMenus_MainMenu(void* args)
{
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
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

void TerminalMenus_ReadMemory(void* args)
{
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
        CheckIfReturnToMenu(UART_struct, charReceived);
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

            xQueueSend(I2C_write_queue, &masterXfer_I2C_read, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);
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
        CheckIfReturnToMenu(UART_struct, charReceived);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

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
            xSemaphoreGive(I2C_done);
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);
            vTaskDelay(pdMS_TO_TICKS(50));

        }

    }
}

void TerminalMenus_EstablishRTCHour(void* args)
{

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
        CheckIfReturnToMenu(UART_struct, charReceived);
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

            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);

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
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);

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
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);

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

            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);
            vTaskDelay(pdMS_TO_TICKS(50));

        }
    }
}
void TerminalMenus_EstablishRTCDate(void* args)
{

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
        CheckIfReturnToMenu(UART_struct, charReceived);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

        if (1 == LineMover.menuPositionLineIndex)
        {
<<<<<<< HEAD
            uint8_t days;
            uint8_t months;
            uint8_t years;
            uint8_t index;
=======
            uint8_t months;
            uint8_t days;
            uint8_t years;
            uint8_t index;
            uint8_t date_buffer[6];
>>>>>>> 13c092a4d93ad7b89ed032a1e0b21711cdce9602
            for (index = 0; index < 6; index++)
            {
                date_buffer[index] = FIFO_pop(0) - 0x30;
            }
<<<<<<< HEAD
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
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);
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
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);

            vTaskDelay(pdMS_TO_TICKS(100));
=======

            years = (date_buffer[0] << 7)|(date_buffer[1] << 6);
>>>>>>> 13c092a4d93ad7b89ed032a1e0b21711cdce9602

        }

    }
}

void TerminalMenus_EstablishRTCHourFormat(void* args)
{

    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
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
        vPortFree(received_UART);

        CharacterValidator(&LineMover, TimeFormat, charReceived);
        CheckIfReturnToMenu(UART_struct, charReceived);
        CheckIfLineMovementAndUartEcho(UART_struct, charReceived, &LineMover);

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
            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);

            vTaskDelay(pdMS_TO_TICKS(20));

            if ('1' == (FIFO_pop(0)))
            {
                timeFormat = readBuffer | TimeFormatMaskSet;
            } else
            {
                timeFormat = readBuffer & TimeFormatMaskNegate;
            }

            masterXfer_I2C_write = pvPortMalloc(sizeof(i2c_master_transfer_t*));
            masterXfer_I2C_write->data = &timeFormat;
            masterXfer_I2C_write->dataSize = 1;
            masterXfer_I2C_write->direction = kI2C_Write;
            masterXfer_I2C_write->flags = kI2C_TransferDefaultFlag;
            masterXfer_I2C_write->slaveAddress = 0x50;
            masterXfer_I2C_write->subaddress = (uint32_t) subaddress;
            masterXfer_I2C_write->subaddressSize = 1;

            xQueueSend(I2C_write_queue, &masterXfer_I2C_write, portMAX_DELAY);
            xSemaphoreTake(I2C_done, portMAX_DELAY);
            xSemaphoreGive(I2C_done);
            vTaskDelay(pdMS_TO_TICKS(100));

        }

    }
}

void TerminalMenus_ReadRTCHour(void* args)
{

    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1000);
    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t dummy_uart = { 0, 1 };
    uint8_t charReceived = 0;
    uint8_t time_buffer_variable[3];
    uint8_t printing_time_chars[9];
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
            CheckIfReturnToMenu(UART_struct, charReceived);
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
                xQueueSend(I2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(I2C_done, portMAX_DELAY);
                xSemaphoreGive(I2C_done);
                vTaskDelay(pdMS_TO_TICKS(100));
                subaddress++;
            }
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
                xQueueSend(I2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(I2C_done, portMAX_DELAY);
                xSemaphoreGive(I2C_done);
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
            CheckIfReturnToMenu(UART_struct, charReceived);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);

        } else
        {

            i2c_master_transfer_t *masterXfer_I2C_write;
            uint8_t index;
            uint16_t subaddress = 0x05;
            for (index; index < 2; index++)
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
                xQueueSend(I2C_write_queue, &masterXfer_I2C_write,
                           portMAX_DELAY);
                xSemaphoreTake(I2C_done, portMAX_DELAY);
                xSemaphoreGive(I2C_done);
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

void TerminalMenus_TerminalsCommunication(void* args)
{

    uart_struct* UART_struct = (uart_struct*) args;
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    LineMovementPack_t LineMover = { 0, 0, 1,
    pdFALSE, { pdFALSE, pdTRUE, pdTRUE }, { { { "\033[12;10H" } } }, //struct with constant position reference used in memory read menu
        { 10000, 0 } };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            xSemaphoreTake(Interface_mutex, portMAX_DELAY);
            firstEntryToMenu = pdTRUE;
            MenuPrinter(UART_struct, Terminal2Communication); //TODO: identify which terminal is inside the function
            xSemaphoreGive(Interface_mutex);
        }
        xQueueReceive(*UART_struct->UART_receive_Queue, &received_UART,
                      portMAX_DELAY);
        charReceived = *received_UART->data;
        vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated

        if (ENTER != charReceived)
        {

            CharacterValidator(&LineMover, Terminal2Communication,
                               charReceived);
            CheckIfReturnToMenu(UART_struct, charReceived);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);
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
            xQueueSend(SPI_queue, &charToBeMirrored, portMAX_DELAY);

            CheckIfReturnToMenu(UART_struct, charReceived);
        } else if (ENTER != charReceived)
        {
            //CheckIfReturnToMenu(charReceived);

            CharacterValidator(&LineMover, LCDEcho, charReceived);
            CheckIfLineMovementAndUartEcho(UART_struct, charReceived,
                                           &LineMover);
            charToBeMirrored = pvPortMalloc(sizeof(SPI_msg_t*));
            charToBeMirrored->LCD_to_be_clear = pdFALSE;
            uint8_t stringBuffer[2];
            stringBuffer[0] = charReceived;
            stringBuffer[1] = '\0';
            charToBeMirrored->string = stringBuffer;
            xQueueSend(SPI_queue, &charToBeMirrored, portMAX_DELAY);
        }

    }
}

