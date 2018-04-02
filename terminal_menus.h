/*
 * terminal_menus.h
 *
 *  Created on: Mar 24, 2018
 *      Author: Paco
 */

#ifndef TERMINAL_MENUS_H_
#define TERMINAL_MENUS_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
void TerminalMenus_MainMenu(void* args);
void TerminalMenus_ReadMemory(void* args);
void TerminalMenus_WriteMemory(void* args);
void TerminalMenus_EstablishRTCHour(void* args);
void TerminalMenus_EstablishRTCDate(void* args);
void TerminalMenus_EstablishRTCHourFormat(void* args);
void TerminalMenus_ReadRTCHour(void* args);
void TerminalMenus_ReadRTCDate(void* args);
void TerminalMenus_TerminalsCommunication(void* args);
void TerminalMenus_LCDEcho(void* args);
void SPIReadHour(void * args);

SemaphoreHandle_t* pGetInterfaceMutex();

EventGroupHandle_t* pGetSubTasksEvents();
#endif /* TERMINAL_MENUS_H_ */
