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

#define PTA1 (1<< 1)
#define PTA2 (1<< 2)
#define PTB23 (1 << 23)
#define PTB9 (1 << 9)
#define PC_UART 0
#define BLUE_UART 1
#define RETURN '='
#define ENTER '/'
#define ASCII_NUMBER_OFFSET 0x30
#define INPUT_SPACES 5
#define MENUS_QUANTITY 11

extern QueueHandle_t UART_send_Queue;
extern QueueHandle_t UART_receive_Queue;

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

#define STRING_SIZE 77
typedef struct {
    struct {
        uint8_t terminal_position[9];
    } Positions[INPUT_SPACES];
} Menu_terminal_positions_t;
typedef struct { /**struct used to contain the various menus string to be printed*/
    uint8_t Strings_quantity; /**stores how many strings will be printed*/
    struct {
        uint8_t String[STRING_SIZE]; /**stored string*/
        uint8_t positionXYCommand[10]; /**stores the printing positions*/
    } Strings[50]; /**stores the strings that correspond with a menu*/
} TerminalMenuType_t;
static TerminalMenuType_t Menus[MENUS_QUANTITY] = { //menus strings to be displayed initialization
/**0*/ {10,{{"1) Leer Memoria I2C",     "\033[3;10H"},{"2) Escribir memoria I2C",       "\033[4;10H"}, {"3) Establecer Hora",   "\033[5;10H" },{"4) Establecer Fecha","\033[6;10H"},{"5) Formato de hora","\033[7;10H"},{"6) Leer hora","\033[8;10H"},{"7) Leer fecha","\033[9;10H"},{"8) Comunicacion con terminal 2","\033[10;10H"},{"9) Eco en LCD","\033[11;10H"},{"0) Configurar contrasena","\033[12;10H"}}},
/**1*/ {7, {{"1) Leer Memoria I2C",     "\033[3;10H"},{"Direccion de lectura: ",        "\033[5;10H"}, {"0x"/*"0x0000"*/,       "\033[5;32H" },{"Longitud en bytes: ","\033[6;10H"},{"0x"/*"50"*/,        "\033[6;29H"},{"Contenido: ","\033[7;10H"},{""/*"Micros y DSP ITESO 2017"*/,"\033[8;10H"}} },
/**2*/ {5, {{"2) Escribir memoria I2C", "\033[4;10H"},{"Direccion de escritura: ",      "\033[6;10H"}, {"0x"/*"0x0050"*/,       "\033[6;34H" },{"Texto a guardar: "  ,"\033[7;10H"},{""/*"Esta es una prueba de escritura"*/,"\033[8;10H"}} },
/**3*/ {3, {{"3) Establecer Hora",      "\033[5;10H"},{"Escribir hora en hh:mm:ss: ",   "\033[7;10H"}, {""/*"7:30:55"*/,        "\033[7;37H" }} },
/**4*/ {3, {{"4) Establecer Fecha",     "\033[6;10H"},{"Escribir fecha en dd/mm/aa: ",  "\033[8;10H"}, {""/*"24:12:2017"*/,     "\033[8;38H" }} },
/**5*/ {5, {{"5) Formato de hora",      "\033[7;10H"},{"El formato actual es ",         "\033[9;10H"}, {""/*"12h"*/,            "\033[9;31H" },{"Escoger formato 12 o 24 (0 / 1): ","\033[10;10H"},{""/*"s"*/,"\033[10;43H"}} },
/**6*/ {3, {{"6) Leer hora",                "\033[8;10H"},{"La hora actual es:",            "\033[10;10H"},{""/*"5:40:54 am"*/,     "\033[11;10H"}} },
/**7*/ {3, {{"7) Leer fecha","\033[9;10H"},{"La fecha actual es:","\033[11;10H"},{""/*"20/10/2017"*/,"\033[12;10H"}} },
/**8*/ {5/*variable*/, {{"8) Comunicacion con terminal 2","\033[10;10H"},{""/*Terminal 1:*/,"\033[12;10H"},{""/*"Como esta?"*/,"\033[13;10H"},{""/*Terminal 2*/,"\033[15;10H"},{""/*Bien, aqui sufriendo con la practica y tu?*/,"\033[16;10H"}} },
/**9*/ {3, {{"9) Eco en LCD","\033[11;10H"},{"Escribir texto: ","\033[13;10H"},{""/*"texto"*/,"\033[14;10H"}} },
/**10*/{6,{{"ERROR","\033[3;28H"},{"Ocasionado por uno de los siguientes motivos: ","\033[5;08H"},{"I) Componente faltante","\033[7;07H"},{"II) Menu de componente ocupado por otra terminal","\033[8;07H"},{"III) Error en la interaccion con un menu","\033[9;07H"},{"Presione 2 veces = para continuar...","\033[12;08H"}} }
};

/////////////////////////////////////////MECHANISMS DEFINITIONS/////////////////////////////////////////////
void MenuPrinter(uint8_t terminalID, uint8_t menuToBePrinted)
{
    uart_transfer_t* toSend_UART;
    uint8_t printedStringIndex = 0;
    uint8_t screen_erease[] = { "\033[2J" }; /**VT100 terminal clear screen command*/
    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
    toSend_UART->data = &screen_erease;
    toSend_UART->dataSize = 1;
    if (PC_UART == terminalID)
    {
        xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
    } else
    {
        //TODO: here would be the queue send for the other UART
    }
    vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
    while (Menus[menuToBePrinted].Strings_quantity > printedStringIndex)
    {
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                &Menus[menuToBePrinted].Strings[printedStringIndex].positionXYCommand;
        toSend_UART->dataSize = 1;
        if (PC_UART == terminalID)
        {
            xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
        } else
        {
            //TODO: here would be the queue send for the other UART
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
        toSend_UART->data =
                &Menus[menuToBePrinted].Strings[printedStringIndex].String;
        toSend_UART->dataSize = 1;
        if (PC_UART == terminalID)
        {
            xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
        } else
        {
            //TODO: here would be the queue send for the other UART
        }
        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        printedStringIndex++;
    }
}

void CreateMenuTask(uint8_t menuToBeCreated)
{
    switch (menuToBeCreated)
    {
        case ReadMemoryI2C:
            xTaskCreate(TerminalMenus_ReadMemory, "Read memory menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case WriteMemoryI2C:
            xTaskCreate(TerminalMenus_WriteMemory, "Write memory menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case StablishHour:
            xTaskCreate(TerminalMenus_EstablishRTCHour, "Establish hour menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case StablishDate:
            xTaskCreate(TerminalMenus_EstablishRTCDate, "Establish date menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case TimeFormat:
            xTaskCreate(TerminalMenus_EstablishRTCHourFormat,
                        "Establish time format menu", configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case ReadHour:
            xTaskCreate(TerminalMenus_ReadRTCHour, "Read hour menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case ReadDate:
            xTaskCreate(TerminalMenus_ReadRTCDate, "read date menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case Terminal2Communication:
            xTaskCreate(TerminalMenus_TerminalsCommunication,
                        "terminals communication menu",
                        configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        case LCDEcho:
            xTaskCreate(TerminalMenus_LCDEcho, "LCD echo menu",
            configMINIMAL_STACK_SIZE,
                        (void*) NULL, 3, NULL);
            vTaskDelete(NULL);
        break;
        default:
        break;
    }
}
void CheckIfReturnToMenu(uint8_t receivedChar)
{
    if (RETURN == receivedChar)
    {
        xTaskCreate(TerminalMenus_MainMenu, "menu task",
        configMINIMAL_STACK_SIZE,
                    (void*) NULL, 1, NULL); //create menu task
        vTaskDelete(NULL);  //deletes current task
    }
}

typedef struct {
    uint8_t menuPositionLineIndex;
    uint8_t linePositionIndex;
    uint8_t MAXwritableLines;
    uint8_t initialPositioning;
    uint8_t lINECharsExtrictNumber[INPUT_SPACES];
    Menu_terminal_positions_t menuUartPositions;
    uint8_t charactersPerInputSpace[INPUT_SPACES];
} LineMovementPack_t;
void CheckIfLineMovementAndUartEcho(uint8_t receivedChar,
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
        xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
    }
    if (((' '==receivedChar)||(('0' <= receivedChar) && ('9' >= receivedChar))
            || (('A' <= receivedChar) && ('Z' >= receivedChar))
            || (('a' <= receivedChar) && ('z' >= receivedChar)))
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
                toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
                toSend_UART->dataSize = 1;
                uint8_t buffer[2] = { 0, '\0' };
                buffer[0] = receivedChar;
                toSend_UART->data = &buffer;
                xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
                vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
            }
        } else
        {
            lineMoverPack->linePositionIndex++;
            toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));
            toSend_UART->dataSize = 1;
            uint8_t buffer[2] = { 0, '\0' };
            buffer[0] = receivedChar;
            toSend_UART->data = &buffer;
            xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
            vTaskDelay(pdMS_TO_TICKS(20)); /**gives time for the UART to properly print the string sent*/
        }
    }
}

//////////////////////////////////////////MENUS FUNCTIONS BODIES////////////////////////////////////////////
void TerminalMenus_MainMenu(void* args)
{
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    for (;;)
    {
        if (pdFALSE == firstEntryToMenu)
        {
            firstEntryToMenu = pdTRUE;
            MenuPrinter(PC_UART, MainMenu); //TODO: identify which terminal is inside the function
        }
        xQueueReceive(UART_receive_Queue, &received_UART, portMAX_DELAY);
        charReceived = *received_UART->data;
        CreateMenuTask(charReceived - 0x30);
        vPortFree(received_UART);
    }
}

void TerminalMenus_ReadMemory(void* args)
{
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    LineMovementPack_t LineMover = {
        0,
        0,
        2,
        pdFALSE,
        { pdTRUE, pdTRUE, pdTRUE }, { { { "\033[5;34H" }, { "\033[6;31H" },
        { "\033[8;10H" } } }, //struct with constant position reference used in memory read menu
        { 4, 2, 0 }
    };
    for (;;)
    {
        /**The task checks if it's the first time in the task, so that the menu is printed*/
        if (pdFALSE == firstEntryToMenu)
        {
            firstEntryToMenu = pdTRUE;
            MenuPrinter(PC_UART, ReadMemoryI2C); //TODO: identify which terminal is inside the function
        }
        xQueueReceive(UART_receive_Queue, &received_UART, portMAX_DELAY);
        charReceived = *received_UART->data;
        CheckIfReturnToMenu(charReceived);
        CheckIfLineMovementAndUartEcho(charReceived, &LineMover);

        vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated
    }
}

void TerminalMenus_WriteMemory(void* args)
{
    uint8_t firstEntryToMenu = pdFALSE;
    uart_transfer_t* received_UART;
    uint8_t charReceived = 0;
    LineMovementPack_t LineMover = {
        0,
        0,
        2,
        pdFALSE,
        { pdTRUE, pdFALSE, pdTRUE },
        { { { "\033[6;36H" },{ "\033[8;10H" } } }, //struct with constant position reference used in memory write menu
        { 4, 1,0 }
    };
   for(;;){
       if (pdFALSE == firstEntryToMenu)
               {
                   firstEntryToMenu = pdTRUE;
                   MenuPrinter(PC_UART, WriteMemoryI2C); //TODO: identify which terminal is inside the function
               }
               xQueueReceive(UART_receive_Queue, &received_UART, portMAX_DELAY);
               charReceived = *received_UART->data;
               CheckIfReturnToMenu(charReceived);
               CheckIfLineMovementAndUartEcho(charReceived, &LineMover);

               vPortFree(received_UART); //the previously reserved memory used for the UART capture is liberated
   }
}

void TerminalMenus_EstablishRTCHour(void* args)
{

}

void TerminalMenus_EstablishRTCDate(void* args)
{

}

void TerminalMenus_EstablishRTCHourFormat(void* args)
{

}

void TerminalMenus_ReadRTCHour(void* args)
{

}

void TerminalMenus_ReadRTCDate(void* args)
{

}

void TerminalMenus_TerminalsCommunication(void* args)
{

}

void TerminalMenus_LCDEcho(void* args)
{

}