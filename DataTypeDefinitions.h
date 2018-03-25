/*
 * DataTypeDefinitions.h
 *
 *  Created on: 11/08/2017
 *      Author: jlpe
 */

#ifndef DATATYPEDEFINITIONS_H_
#define DATATYPEDEFINITIONS_H_


/*! This definition is as a general definitions to bits in regiter or pins in the microcontroller.*/
typedef enum {BIT0,  /*!< Bit 0 */
			  BIT1,  /*!< Bit 1 */
			  BIT2,  /*!< Bit 2 */
			  BIT3,  /*!< Bit 3 */
			  BIT4,  /*!< Bit 4 */
			  BIT5,  /*!< Bit 5 */
			  BIT6,  /*!< Bit 6 */
			  BIT7,  /*!< Bit 7 */
	          BIT8,  /*!< Bit 8 */
	          BIT9,  /*!< Bit 9 */
	          BIT10, /*!< Bit 10 */
	          BIT11, /*!< Bit 11 */
	          BIT12, /*!< Bit 12 */
	          BIT13, /*!< Bit 13 */
	          BIT14, /*!< Bit 14 */
	          BIT15, /*!< Bit 15 */
	          BIT16, /*!< Bit 16 */
	          BIT17, /*!< Bit 17 */
	          BIT18, /*!< Bit 18 */
	          BIT19, /*!< Bit 19 */
	          BIT20, /*!< Bit 20 */
	          BIT21, /*!< Bit 21 */
	          BIT22, /*!< Bit 22 */
	          BIT23,/*!< Bit 23 */
	          BIT24, /*!< Bit 24 */
	          BIT25, /*!< Bit 25 */
	          BIT26, /*!< Bit 26 */
	          BIT27, /*!< Bit 27 */
	          BIT28, /*!< Bit 28 */
	          BIT29, /*!< Bit 29 */
	          BIT30, /*!< Bit 30 */
	          BIT31  /*!< Bit 31 */
			} BitsType;


typedef enum{FALSE, TRUE} BooleanType;
/*! This definition is as a general definitions to bits turn-on or turn-off any bit*/
typedef enum {BIT_OFF, BIT_ON} BIT_ON_OFF_Type;
/*! This data type is unsigned integer of 8 bits*/
typedef unsigned char uint8;
/*! This data type is signed integer of 8 bits*/
typedef char sint8;
/*! This data type is 16-bit unsigned integer*/
typedef unsigned short int uint16;
/*! This data type is 16-bit signed integer*/
typedef short int sint16;
/*! This data type is 32-bit unsigned integer*/
typedef unsigned long int uint32;
/*! This data type is 16-bit signed integer*/
typedef long int sint32;


#endif /* DATATYPEDEFINITIONS_H_ */
