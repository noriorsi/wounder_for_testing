
#include "Error.h"

/*********************************************************************************************************************
-------------------------------------------------- Static Functions -----------------------------------------------
 *********************************************************************************************************************/

static void PrintErrNum(unsigned int errnum){
	char errno[10];
	snprintf(errno, 10, "[%d]", errnum);
	send_string(errno);
}

/***************************************************************
 *  Sends the error number and the appropriate message.
***************************************************************/
void ErrorHandler(unsigned int errnum){
	switch(errnum){

		case SEND_STRING_ENCODING_ERROR_NUMBER:{

			PrintErrNum(SEND_STRING_ENCODING_ERROR_NUMBER);
			send_string("String encoding error.\n");
			break;

		}

		case SEND_STRING_BUFFER_OVERFLOW_ERROR_NUMBER:{

			PrintErrNum(SEND_STRING_BUFFER_OVERFLOW_ERROR_NUMBER);
			send_string("String is too long.\n");
			break;

		}

		case SEND_DOUBLE_BUFFER_OVERFLOW_ERROR_NUMBER:{

			PrintErrNum(SEND_DOUBLE_BUFFER_OVERFLOW_ERROR_NUMBER);
			send_string("The value is too high.\n");
			break;

		}

		case SEND_INT_BUFFER_OVERFLOW_ERROR_NUMBER:{

			PrintErrNum(SEND_INT_BUFFER_OVERFLOW_ERROR_NUMBER);
			send_string("The value is too high.\n");
			break;

		}
	}
}

void PrintAndAbort(unsigned int errnum){
	switch(errnum){
	case GPIO_ERROR_NUMBER:{
		send_string("***\n");
		send_string("This pin is\n"); send_string("already taken\n");
		send_string("***\n");
		break;
	}
	default: send_string("\n"); send_string("***\n");
	}

	EnterEM4();

}

void GPIOError(GPIO_Port_TypeDef port, unsigned int pin){
	send_string("\n");
	send_string("***\n");
	send_string("GPIO error \n");
	send_string("PORT: ");
	switch(port){
		case gpioPortA: send_string("A\n"); break;
		case gpioPortB: send_string("B\n"); break;
		case gpioPortC: send_string("C\n"); break;
		case gpioPortD: send_string("D\n"); break;
		case gpioPortE: send_string("E\n"); break;
		case gpioPortF: send_string("F\n"); break;
	}
	send_string("\nPIN: "); send_int(pin);
	send_string("***\n");

	PrintAndAbort(DEFAULT_ERROR);
}
