/**
	\file
	\brief
		This is the source file for the FIFO
		This file contains the control for all the FIFOS needed.
	\author Esteban Gonzalez Moreno ie565892@iteso.mx, Gabriel Santamaria Garcia ie699356@iteso.mx
	\date	7/09/2014
 */

#include "FIFO.h"

FIFO_stacks_t FIFOs[5] = {
{{" "},FALSE,FALSE,0},
{{" "},FALSE,FALSE,0},
{{" "},FALSE,FALSE,0},
{{" "},FALSE,FALSE,0},
{{" "},FALSE,FALSE,0}
};//general purpose FIFO stacks

uint8_t PopFlag = FALSE;	//flag used to know if the pop function has been called the first time
uint8_t FIFO_forwardIndex = 0;	//index variable used to move the stack forward while doing pops, and achieve a FIFO behavior

uint8_t FIFO_push(uint8_t fifo_number, uint8 data){
	if(FIFO_SIZE < FIFOs[fifo_number].FIFO_index){	//if the fifo index exceeds the fifo size,
		FIFOs[fifo_number].Full_flag = TRUE;		//the fifo is full
	}
	if(FALSE == FIFOs[fifo_number].Full_flag){		//if the fifo isn't full,
		FIFOs[fifo_number].FIFO_array[FIFOs[fifo_number].FIFO_index] = data;	//stores the received data in the selected fifo current index
		FIFOs[fifo_number].Empty_flag = FALSE;									//the fifo is not empty
		FIFOs[fifo_number].FIFO_index++;						//increases the selected fifo index after inserting a value
	}
	return FIFOs[fifo_number].Full_flag;	//returns current FIFO full flag status
}

uint8_t FIFO_pop(uint8_t fifo_number){
	if(FALSE == PopFlag){	//if it's the first time pop has been called,
		FIFO_forwardIndex = 0;
		PopFlag = TRUE;		//the pop function has already been called
	}
	uint8_t data = 0;
	if(0 >= FIFOs[fifo_number].FIFO_index){
		FIFOs[fifo_number].Empty_flag = TRUE;		//the fifo is empty
		PopFlag = FALSE;	//the pop function is over, so it can be reseted
	}
	if(FALSE == FIFOs[fifo_number].Empty_flag){		//if there is still something to take out,
		FIFOs[fifo_number].FIFO_index--;						//decreases the selected fifo index after withdrawal
		data = FIFOs[fifo_number].FIFO_array[FIFO_forwardIndex];	//takes out the current position data
		FIFOs[fifo_number].FIFO_array[FIFO_forwardIndex] = 0;	//clears the current position
		FIFOs[fifo_number].Full_flag = FALSE;									//the fifo is not full
		FIFO_forwardIndex++;									//the counter is increased
	}
	return data;	//returns the withdrawn data
}

uint8_t FIFO_checkIfEmpty(uint8_t fifo_number){
	return FIFOs[fifo_number].Empty_flag;	//returns current FIFO empty flag status
}

void FIFO_clearFIFOs(){
    uint8_t index = 0;
	while(FIFO_SIZE>index){
		FIFOs[0].FIFO_array[index] = '0';
		FIFOs[1].FIFO_array[index] = '0';
		FIFOs[2].FIFO_array[index] = '0';
		FIFOs[3].FIFO_array[index] = '0';
		FIFOs[4].FIFO_array[index] = '0';
		index++;
	}
	FIFOs[0].Empty_flag = FALSE;
	FIFOs[0].Full_flag = FALSE;
	FIFOs[1].Empty_flag = FALSE;
	FIFOs[1].Full_flag = FALSE;
	FIFOs[2].Empty_flag = FALSE;
	FIFOs[2].Full_flag = FALSE;
	FIFOs[3].Empty_flag = FALSE;
	FIFOs[3].Full_flag = FALSE;
	FIFOs[4].Empty_flag = FALSE;
	FIFOs[4].Full_flag = FALSE;
	FIFOs[0].FIFO_index = 0;
	FIFOs[1].FIFO_index = 0;
	FIFOs[2].FIFO_index = 0;
	FIFOs[3].FIFO_index = 0;
	FIFOs[4].FIFO_index = 0;
}
