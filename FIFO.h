/**
	\file
	\brief
		This is the header file for the FIFO
		This file contains the control for all the FIFOS needed.
	\author Esteban Gonzalez Moreno ie565892@iteso.mx, Gabriel Santamaria Garcia ie699356@iteso.mx
	\date	7/09/2014
 */
#ifndef FIFO_H_
#define FIFO_H_

#include "DataTypeDefinitions.h"
#include "stdint.h"
#define FIFO_SIZE 70

typedef enum{
	FIFO_0,
	FIFO_1,
	FIFO_2,
	FIFO_3,
	FIFO_4
}FIFO_spaces_t;


/**
 * \brief user created struct containing a string for the LCD and his position in x y
 */
typedef struct{
    uint8_t FIFO_array[FIFO_SIZE];	/**static memory array used to represent the FIFO stack (basically because malloc is not allowed)*/
    uint8_t Empty_flag;				/**variable used to know if current FIFO is empty*/
    uint8_t Full_flag;				/**variable used to know if current FIFO is full*/
    uint8_t FIFO_index;				/**variable used to store the current fifo index*/
}FIFO_stacks_t;


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function used to push somethin on the FIFO
	 \param[in] FIFO to push the data
	 \param[in] Data to push
 	\return Flag if fifo is full
 */
uint8_t FIFO_push(uint8_t fifo_number, uint8_t data);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function used to get a value from the FIFO
	 \param[in] FIFO number to get the data
 	\return Data from the FIFO
 */
uint8_t FIFO_pop(uint8_t fifo_number);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function used to check if The selected FIFO is empty
	 \param[in] FIFO number to check if it's empty
 	\return TRUE or FALSE value
 */
uint8_t FIFO_checkIfEmpty(uint8_t fifo_number);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief	function used to clear all the FIFOs
 	\return nothing
 */
void FIFO_clearFIFOs();

#endif
