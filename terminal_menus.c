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

#define PTA1 (1<< 1)
#define PTA2 (1<< 2)
#define PTB23 (1 << 23)
#define PTB9 (1 << 9)

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
        PRINTF("PUTO del A1");
    } else if (PTA2 == (InterruptFlags & PTA2))
    {
        PRINTF("MAS PUTO del A2");

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
        PRINTF("PUTO del B9");
    } else if (PTB23 == (InterruptFlags & PTB23))
    {
        PRINTF("MAS PUTO del B23");

        GPIO_ClearPinsInterruptFlags(GPIOB, PTB23);
    }

}

//quoting: Gonzalez, E., Santamaria, G. ; DSP's 2017
#define STRING_SIZE 77
typedef struct { /**struct used to contain the various menus string to be printed*/
    uint8_t Strings_quantity; /**stores how many strings will be printed*/
    struct {
        uint8_t String[STRING_SIZE]; /**stored string*/
        uint8_t positionXYCommand[10]; /**stores the printing positions*/
    } Strings[50]; /**stores the strings that correspond with a menu*/
} TerminalMenuType_t;
static TerminalMenuType_t Menus[12] = { //menus strings to be displayed initialization
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
/**10*/{3,{{"SISTEMA BLOQUEADO","\033[3;22H"},{"Contrasena: ","\033[5;25H"},{""/* **** */,"\033[6;28H"}} },
/**11*/{6,{{"ERROR","\033[3;28H"},{"Ocasionado por uno de los siguientes motivos: ","\033[5;08H"},{"I) Componente faltante","\033[7;07H"},{"II) Menu de componente ocupado por otra terminal","\033[8;07H"},{"III) Error en la interaccion con un menu","\033[9;07H"},{"Presione 2 veces = para continuar...","\033[12;08H"}} }
};
//end of quoting

void TerminalMenus_ReadMemory(void* args)
{
    uart_transfer_t* toSend_UART;

    toSend_UART = pvPortMalloc(sizeof(uart_transfer_t*));

    toSend_UART->dataSize = 1;
    void MenuPrinting()
    {
        uint8_t hola[] = {"\033[2J"};
        toSend_UART->data = &hola;
        xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20));

        uint8_t adios[] = {"ADIOS23"};
                toSend_UART->data = &adios;
                xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(20));
//        uint8_t stringIndex = 0; //local variable used to control which string is to be printed
//        uint8_t Stringaaa[] = { " \033[2J" };
//        for (; stringIndex < 5; stringIndex++)
//        {
//            uint8_t temporalBuffer = Stringaaa[stringIndex];
//            toSend_UART->data = &temporalBuffer;
//            xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
//        }
//
//        stringIndex = 0; //local variable used to control which string is to be printed
//        uint8_t Stringbbb[] = { " HOLA"};
//        for (; stringIndex < 5; stringIndex++)
//        {
//            uint8_t temporalBuffer = Stringbbb[stringIndex];
//            toSend_UART->data = &temporalBuffer;
//            xQueueSend(UART_send_Queue, &toSend_UART, portMAX_DELAY);
//        }
    }
    MenuPrinting();

//    volatile  uint8_t buffer_prueba[1] = {'X'};
//    toSend_UART->data = buffer_prueba;
//    toSend_UART->dataSize = 1;
//    xQueueSend(UART_send_Queue,&toSend_UART,portMAX_DELAY);
    vTaskSuspend(NULL);
}

void TerminalMenus_WriteMemory(void* args)
{

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
