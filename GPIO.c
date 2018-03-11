/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	7/09/2014
	\todo
	    Interrupts are not implemented in this API implementation.
 */
#include "MK64F12.h"
#include "GPIO.h"

GPIO_interruptFlags_t GPIO_intrStatusFlag;	/** GPIO_interruptFlags_t struct type variable is declared */

uint32 Position(uint8 pin){	/**Function that returns the desired bit position equivalent in binary*/
	uint32 base = 0x00000001;	/**This is the base which will be shifted *pin* times to the left*/
	return base<<pin;			/**returns the actual shifting of the base, pin-times, to the left"*/
}

uint8 GPIO_getIRQStatus(GPIO_portNameType gpio)
{
	switch (gpio) {		/**Selecting the GPIO port for getting the interrupt status flag current status*/
		case GPIO_A:	/**GPIO A is selected*/
			return(GPIO_intrStatusFlag.flagPortA);	/** Returns the status of the interrupt flag of GPIO A */
			break;
		case GPIO_B:	/**GPIO B is selected*/
			return(GPIO_intrStatusFlag.flagPortB);	/** Returns the status of the interrupt flag of GPIO B */
			break;
		case GPIO_C:	/**GPIO C is selected*/
			return(GPIO_intrStatusFlag.flagPortC);	/** Returns the status of the interrupt flag of GPIO C */
			break;
		case GPIO_D:	/**GPIO D is selected*/
			return(GPIO_intrStatusFlag.flagPortD);	/** Returns the status of the interrupt flag of GPIO D */
			break;
		case GPIO_E:	/**GPIO E is selected*/
			return(GPIO_intrStatusFlag.flagPortE);	/** Returns the status of the interrupt flag of GPIO E */
			break;
		default:
			return(ERROR);	/** If an invalid port is selected, it will return ERROR (0x02) */
			break;
	}	//end switch

}

uint8 GPIO_clearIRQStatus(GPIO_portNameType gpio)
{
	switch (gpio) {
		case GPIO_A:	/**GPIO A is selected*/
			GPIO_intrStatusFlag.flagPortA = FALSE;	/** sets GPIO A interrupt flag to 0 */
			break;
		case GPIO_B:	/**GPIO B is selected*/
			GPIO_intrStatusFlag.flagPortB = FALSE;	/** sets GPIO B interrupt flag to 0 */
			break;
		case GPIO_C:	/**GPIO C is selected*/
			GPIO_intrStatusFlag.flagPortC = FALSE;	/** sets GPIO C interrupt flag to 0 */
			break;
		case GPIO_D:	/**GPIO D is selected*/
			GPIO_intrStatusFlag.flagPortD = FALSE;	/** sets GPIO D interrupt flag to 0 */
			break;
		case GPIO_E:	/**GPIO E is selected*/
			GPIO_intrStatusFlag.flagPortE = FALSE;	/** sets GPIO E interrupt flag to 0 */
			break;
		default:
			return(ERROR);	/** If an invalid port is selected, it will return ERROR (0x02) */
			break;
	}	//end switch

	return(TRUE);

}

void GPIO_clearInterrupt(GPIO_portNameType portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
	{
		case GPIO_A: /** GPIO A is selected*/
			PORTA->ISFR=0xFFFFFFFF;
			break;
		case GPIO_B: /** GPIO B is selected*/
			PORTB->ISFR=0xFFFFFFFF;
			break;
		case GPIO_C: /** GPIO C is selected*/
			PORTC->ISFR = 0xFFFFFFFF;
			break;
		case GPIO_D: /** GPIO D is selected*/
			PORTD->ISFR=0xFFFFFFFF;
			break;
		default: /** GPIO E is selected*/
			PORTE->ISFR=0xFFFFFFFF;
			break;

	}// end switch
}
uint8 GPIO_clockGating(GPIO_portNameType portName)
{
	switch(portName)/** Selecting the GPIO for clock enabling*/
			{
				case GPIO_A: /** GPIO A is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
					break;
				case GPIO_B: /** GPIO B is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
					break;
				case GPIO_C: /** GPIO C is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
					break;
				case GPIO_D: /** GPIO D is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
					break;
				case GPIO_E: /** GPIO E is selected*/
					SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
					break;
				default: /**If doesn't exist the option*/
					return(FALSE);
			}// end switch
	/**Successful configuration*/
	return(TRUE);
}// end function

uint8 GPIO_pinControlRegister(GPIO_portNameType portName,uint8 pin,const GPIO_pinControlRegisterType*  pinControlRegister)
{

	switch(portName)
		{
		case GPIO_A:/** GPIO A is selected*/
			PORTA->PCR[pin] = *pinControlRegister;	/** Sets the configuration of the desired pin as established by the input parameter */
			break;
		case GPIO_B:/** GPIO B is selected*/
			PORTB->PCR[pin] = *pinControlRegister;	/** Sets the configuration of the desired pin as established by the input parameter */
			break;
		case GPIO_C:/** GPIO C is selected*/
			PORTC->PCR[pin] = *pinControlRegister;	/** Sets the configuration of the desired pin as established by the input parameter */
			break;
		case GPIO_D:/** GPIO D is selected*/
			PORTD->PCR[pin] = *pinControlRegister;	/** Sets the configuration of the desired pin as established by the input parameter */
			break;
		case GPIO_E: /** GPIO E is selected*/
			PORTE->PCR[pin]= *pinControlRegister;	/** Sets the configuration of the desired pin as established by the input parameter */
		default:/**If doesn't exist the option*/
			return(FALSE);
		break;
		}
	/**Successful configuration*/
	return(TRUE);
}

void GPIO_writePORT(GPIO_portNameType portName, uint32 Data ){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		GPIOA->PDOR = Data;	/** Sets the value of parameter Data to the PDOR register of GPIO A */
		break;
	case GPIO_B:	/** GPIO B is selected*/
		GPIOB->PDOR = Data;	/** Sets the value of parameter Data to the PDOR register of GPIO B */
		break;
	case GPIO_C:	/** GPIO C is selected*/
		GPIOC->PDOR = Data;	/** Sets the value of parameter Data to the PDOR register of GPIO C */
		break;
	case GPIO_D:	/** GPIO D is selected*/
		GPIOD->PDOR = Data;	/** Sets the value of parameter Data to the PDOR register of GPIO D */
		break;
	case GPIO_E:	/** GPIO E is selected*/
		GPIOE->PDOR = Data;	/** Sets the value of parameter Data to the PDOR register of GPIO E */
		break;
	default:
		break;
	}
}

uint32 GPIO_readPORT(GPIO_portNameType portName){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		return GPIOA->PDIR;	/** Returns the value in the PDIR register of the GPIO A */
		break;
	case GPIO_B:	/** GPIO B is selected*/
		return GPIOB->PDIR;	/** Returns the value in the PDIR register of the GPIO B */
		break;
	case GPIO_C:	/** GPIO C is selected*/
		return GPIOC->PDIR;	/** Returns the value in the PDIR register of the GPIO C */
		break;
	case GPIO_D:	/** GPIO D is selected*/
		return GPIOD->PDIR;	/** Returns the value in the PDIR register of the GPIO D */
		break;
	case GPIO_E:	/** GPIO E is selected*/
		return GPIOE->PDIR;	/** Returns the value in the PDIR register of the GPIO E */
		break;
	default:
		return 0;	/** If an invalid port is selected, a 0 will be returned */
		break;
	}
}

uint8 GPIO_readPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		return GPIOA->PDIR & Position(pin);	/** Returns the value in the PDIR register of the GPIO A in the specified pin */
		break;
	case GPIO_B:	/** GPIO B is selected*/
		return GPIOB->PDIR & Position(pin);	/** Returns the value in the PDIR register of the GPIO B in the specified pin */
		break;
	case GPIO_C:	/** GPIO C is selected*/
		return GPIOC->PDIR & Position(pin);	/** Returns the value in the PDIR register of the GPIO C in the specified pin */
		break;
	case GPIO_D:	/** GPIO D is selected*/
		return GPIOD->PDIR & Position(pin);	/** Returns the value in the PDIR register of the GPIO D in the specified pin */
		break;
	case GPIO_E:	/** GPIO E is selected*/
		return GPIOE->PDIR & Position(pin);	/** Returns the value in the PDIR register of the GPIO E in the specified pin */
		break;
	default:
		return 0;	/** If an invalid port is selected, a 0 will be returned */
		break;
	}
}

void GPIO_setPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		GPIOA->PSOR = (Position(pin));	/** Sets the specified pin to 1 in the GPIO A */
		break;
	case GPIO_B:	/** GPIO B is selected*/
		GPIOB->PSOR = (Position(pin));	/** Sets the specified pin to 1 in the GPIO B */
		break;
	case GPIO_C:	/** GPIO C is selected*/
		GPIOC->PSOR = (Position(pin));	/** Sets the specified pin to 1 in the GPIO C */
		break;
	case GPIO_D:	/** GPIO D is selected*/
		GPIOD->PSOR = (Position(pin));	/** Sets the specified pin to 1 in the GPIO D */
		break;
	case GPIO_E:	/** GPIO E is selected*/
		GPIOE->PSOR = (Position(pin));	/** Sets the specified pin to 1 in the GPIO E */
		break;
	default:
		break;
	}
}

void GPIO_clearPIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
		case GPIO_A:	/** GPIO A is selected*/
			GPIOA->PCOR = (Position(pin));	/** Sets the specified pin to 0 in the GPIO A */
			break;
		case GPIO_B:	/** GPIO B is selected*/
			GPIOB->PCOR = (Position(pin));	/** Sets the specified pin to 0 in the GPIO B */
			break;
		case GPIO_C:	/** GPIO C is selected*/
			GPIOC->PCOR = (Position(pin));	/** Sets the specified pin to 0 in the GPIO C */
			break;
		case GPIO_D:	/** GPIO D is selected*/
			GPIOD->PCOR = (Position(pin));	/** Sets the specified pin to 0 in the GPIO D */
			break;
		case GPIO_E:	/** GPIO E is selected*/
			GPIOE->PCOR = (Position(pin));	/** Sets the specified pin to 0 in the GPIO E */
			break;
		default:
			break;
		}
}

void GPIO_tooglePIN(GPIO_portNameType portName, uint8 pin){
	switch(portName){
		case GPIO_A:	/** GPIO A is selected*/
			GPIOA->PTOR = (Position(pin));	/** Toggles the specified pin from 0 to 1, or 1 to 0, in the GPIO A */
			break;
		case GPIO_B:	/** GPIO B is selected*/
			GPIOB->PTOR = (Position(pin));	/** Toggles the specified pin from 0 to 1, or 1 to 0, in the GPIO B */
			break;
		case GPIO_C:	/** GPIO C is selected*/
			GPIOC->PTOR = (Position(pin));	/** Toggles the specified pin from 0 to 1, or 1 to 0, in the GPIO C */
			break;
		case GPIO_D:	/** GPIO D is selected*/
			GPIOD->PTOR = (Position(pin));	/** Toggles the specified pin from 0 to 1, or 1 to 0, in the GPIO D */
			break;
		case GPIO_E:	/** GPIO E is selected*/
			GPIOE->PTOR = (Position(pin));	/** Toggles the specified pin from 0 to 1, or 1 to 0, in the GPIO E */
			break;
		default:
			break;
		}
}

void GPIO_dataDirectionPORT(GPIO_portNameType portName ,uint32 direction){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		GPIOA->PDDR = direction;	/** Sets the entire GPIO A as input/output depending of the direction input parameter */
		break;
	case GPIO_B:	/** GPIO B is selected*/
		GPIOB->PDDR = direction;	/** Sets the entire GPIO B as input/output depending of the direction input parameter */
		break;
	case GPIO_C:	/** GPIO C is selected*/
		GPIOC->PDDR = direction;	/** Sets the entire GPIO C as input/output depending of the direction input parameter */
		break;
	case GPIO_D:	/** GPIO D is selected*/
		GPIOD->PDDR = direction;	/** Sets the entire GPIO D as input/output depending of the direction input parameter */
		break;
	case GPIO_E:	/** GPIO E is selected*/
		GPIOE->PDDR = direction;	/** Sets the entire GPIO E as input/output depending of the direction input parameter */
		break;
	default:
		break;
	}
}

void GPIO_dataDirectionPIN(GPIO_portNameType portName, uint8 State, uint8 pin){
	switch(portName){
	case GPIO_A:	/** GPIO A is selected*/
		if(State==TRUE){	/** If the input parameter State value is TRUE, */
			GPIOA->PDDR |= Position(pin);	/** Sets a specified pin in the GPIO A as input/output depending of the State input parameter, without altering other pins */
		}
		break;
	case GPIO_B:	/** GPIO B is selected*/
		if(State==TRUE){	/** If the input parameter State value is TRUE, */
			GPIOB->PDDR |= Position(pin);	/** Sets a specified pin in the GPIO B as input/output depending of the State input parameter, without altering other pins */
		}
		break;
	case GPIO_C:	/** GPIO C is selected*/
		if(State==TRUE){	/** If the input parameter State value is TRUE, */
			GPIOC->PDDR |= Position(pin);	/** Sets a specified pin in the GPIO C as input/output depending of the State input parameter, without altering other pins */
		}
		break;
	case GPIO_D:	/** GPIO D is selected*/
		if(State==TRUE){	/** If the input parameter State value is TRUE, */
			GPIOD->PDDR |= Position(pin);	/** Sets a specified pin in the GPIO D as input/output depending of the State input parameter, without altering other pins */
		}
		break;
	case GPIO_E:	/** GPIO E is selected*/
		if(State==TRUE){	/** If the input parameter State value is TRUE, */
			GPIOE->PDDR |= Position(pin);	/** Sets a specified pin in the GPIO E as input/output depending of the State input parameter, without altering other pins */
		}
		break;
	default:
		break;
	}
}
