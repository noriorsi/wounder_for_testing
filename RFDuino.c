/*
 * RFDuino.c
 *
 *  Created on: 2017. júl. 13.
 *      Author: szmik_000
 */

#include "RFDuino.h"

/*********************************************************************************************************************
-------------------------------------------------- Variables --------------------------------------------------
 *********************************************************************************************************************/

print_buffer_struct buffer;
rx_buffer_struct	rx_buffer;
unsigned cmd_char_counter = 0;
unsigned param_char_counter = 0;
char commandchar = 0;
int commandchar_isfree = true;

/*********************************************************************************************************************
-------------------------------------------------- Static Functions --------------------------------------------------
 *********************************************************************************************************************/

/*************************************************************
 * Contains everything that is needed to initialize the uart.
 *************************************************************/
static void InitUART(){
	/* USART is a HFPERCLK peripheral. Enable HFPERCLK domain and USART1.
	 * We also need to enable the clock for GPIO to configure pins. */
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_USART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);


	/* Initialize with default settings and then update fields according to application requirements. */
	USART_InitAsync_TypeDef initAsync = USART_INITASYNC_DEFAULT;
	initAsync.baudrate = BAUD;

	USART_InitAsync(USART0, &initAsync);



	/* Enable I/O and set location */
	USART0->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | (USER_LOCATION<<8);
	/* Also enable CS and CLK pins if the USART is configured for synchronous mode.
	 * Set GPIO mode. */

	 /* To avoid false start, configure TX pin as initial high */
	 GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_TX_PORT(USER_LOCATION), AF_USART0_TX_PIN(USER_LOCATION), gpioModePushPull, 1);
	 GPIO_PinModeSet((GPIO_Port_TypeDef)AF_USART0_RX_PORT(USER_LOCATION), AF_USART0_RX_PIN(USER_LOCATION), gpioModeInput, 0);
	 /* Don't enable CTS/RTS hardware flow control pins in this example. */

	  USART_IntClear(USART0, USART_IF_RXDATAV);
	  USART_IntEnable(USART0, USART_IF_RXDATAV);

	  NVIC_ClearPendingIRQ(USART0_RX_IRQn);
	  NVIC_EnableIRQ(USART0_RX_IRQn);

	  USART_IntClear(USART0, USART_IF_TXC);
	  USART_IntEnable(USART0, USART_IF_TXC);

	  NVIC_ClearPendingIRQ(USART0_TX_IRQn);
	  NVIC_EnableIRQ(USART0_TX_IRQn);

}


/**************************************************
 *  Gives starting value for the buffer variable.
**************************************************/
static void InitBuffer(){
	for(int i = 0; i<PRINT_BUFFER_SIZE;++i){
		buffer.data[i]=0;
	}
	buffer.head=0;
	buffer.tail=0;
	buffer.now_printing=false;

	for(int i=0; i<RX_BUFFER_SIZE; ++i){
		rx_buffer.data[i] = 0;
	}
	rx_buffer.index = 0;
}





/**************************************************
 *  Prepares the string to be sent
**************************************************/
static void send(char * string){

	#ifndef	LEDS_OFF
	SetGPIO(MCULED3_PORT, MCULED3_PIN, 1);
	#endif

      while (*string != 0)
      {
            if (*string == '\n' || *string == '\0')
            {
                  //USART_Tx(USART1, '\r');
                  buffer.data[buffer.head++] = '\r';
                  buffer.head = fix_overflow(buffer.head);
            }
            //USART_Tx(USART1, *string++);

            buffer.data[buffer.head++] = *string++;
            buffer.head = fix_overflow(buffer.head);
      }

      // We need to kick off the first transfer sometimes to get things going
      CORE_AtomicDisableIrq();
      if (!buffer.now_printing)
      {
            buffer.now_printing = true;
            USART_Tx(USART0, buffer.data[buffer.tail++]);
            buffer.tail = fix_overflow(buffer.tail);
      }
      CORE_AtomicEnableIrq();

      Delay(DELAY_AFTER_SENDING);

      SetGPIO(MCULED3_PORT, MCULED3_PIN, 0);
}


/***************************************************
Sets the command character
***************************************************/
static void SetCommandChar(char ch){

  if(commandchar_isfree){

    switch(ch){
      case COMMAND_CHARACTER:   commandchar = COMMAND_CHARACTER;  commandchar_isfree = false;   break;
      default: break;
    }

  }

}


/***************************************************
Stores the command string
***************************************************/
static void GetCommand(char ch){
  if(ch != 0){
    if(ch == COMMAND_CHARACTER) cmd_char_counter++;

	rx_buffer.data[rx_buffer.index] = ch;
	rx_buffer.index++;
	if(rx_buffer.index >= RX_BUFFER_SIZE-1) rx_buffer.index=0;


    if(cmd_char_counter == 2){
      rx_buffer.data[rx_buffer.index] = '\0';
      int cmd = VerifyCommand(rx_buffer.data);
      ExecuteCommand(cmd);

      rx_buffer.index=0;
      commandchar = 0;
      commandchar_isfree = true;
      cmd_char_counter = 0;
    }

  }

}
/***************************************************
Stores the given parameter
***************************************************/
/*static void GetParam(char ch){
  if(ch != 0){
    if(ch == PARAM_CHAR) param_char_counter++;
    rx_buffer.data[rx_buffer.index] = ch;
    rx_buffer.index++;

    if(rx_buffer.index >= RX_BUFFER_SIZE-1) rx_buffer.index=0;

    if(param_char_counter==2){
    	rx_buffer.data[rx_buffer.index] = 0;
        ResetParams(); //Reset the params before receiving new ones
        SaveParam(rx_buffer.data, rx_buffer.index);

        rx_buffer.index=0;
        commandchar = 0;
        commandchar_isfree = true;
        param_char_counter = 0;
    }
  }


}

*/

/*********************************************************************************************************************
-------------------------------------------------- Functions --------------------------------------------------
 *********************************************************************************************************************/

/*************************************************************
 * Initializes everything.
 *************************************************************/
void InitRFDuino(){
	  InitUART();
	  InitBuffer();

}


/*************************************************************
 * The interrupt handler for received packages.
 *************************************************************/
void USART0_RX_IRQHandler(void){
	char ch = USART0->RXDATA;

	SetCommandChar(ch);

	switch(commandchar){
	  case COMMAND_CHARACTER: GetCommand(ch); break;

	  default: break;
	}

	GPIO_IntClear(RX_PIN_INT_MASK);
	GPIO_IntEnable(RX_PIN_INT_MASK); //enable interrupt again

}


/*************************************************************
 * The interrupt handler for sending packages.
 *************************************************************/
void USART0_TX_IRQHandler(void){
if (USART0->IF & USART_IF_TXC)
      {
            // This flag is not automatically cleared like RXDATAV
            USART_IntClear(USART0, USART_IF_TXC);

            if (buffer.tail != buffer.head)
            {
                  USART_Tx(USART0, buffer.data[buffer.tail++]);
                  buffer.tail = fix_overflow(buffer.tail);
            }
            else
            {
                  buffer.now_printing = false;
            }
      }
}

/**************************************************
 *  Handles overflow
**************************************************/
uint16_t fix_overflow(uint16_t index){
	if (index >= PRINT_BUFFER_SIZE)
	{
		return 0;
	}
	return index;
}



/**************************************************
 *  Prepares the integer to be sent out.
**************************************************/
void send_int(int data){
	char tempc[13];
		if(data>999999999){ //Max 10 decimal digits is allowed
			ErrorHandler(SEND_INT_BUFFER_OVERFLOW_ERROR_NUMBER);
			return;
		}
#ifdef SEND_TYPE_IDENTIFIERS
	snprintf(tempc,13, "d%d\n", data);
#endif

#ifndef SEND_TYPE_IDENTIFIERS
	snprintf(tempc,13, "%d\n", data);
#endif
	send(tempc);
}

/**************************************************
 *  Prepares the double to be sent out.
**************************************************/
void send_double(double data){
	char tempc[23];
	int whole = data;
	if(whole>999999999){//Max 10 decimals
		ErrorHandler(SEND_DOUBLE_BUFFER_OVERFLOW_ERROR_NUMBER);
		return;
	}

	double fraction = (data-whole);

	int array[RESOLUTION];
	for(int i=0; i<RESOLUTION; ++i){
		array[i] = fraction*10;
		fraction = fraction*10 - array[i];
	}
#ifdef SEND_TYPE_IDENTIFIERS
	switch(RESOLUTION){
	case 0: snprintf(tempc,23, "f%d\n",whole); break;
	case 1: snprintf(tempc,23, "f%d.%d\n",whole,array[0]); break;
	case 2: snprintf(tempc,23, "f%d.%d%d\n",whole,array[0],array[1]); break;
	case 3: snprintf(tempc,23, " %d.%d%d%d\n",whole,array[0],array[1],array[2]); break;
	case 4: snprintf(tempc,23, "f%d.%d%d%d%d\n",whole,array[0],array[1],array[2],array[3]); break;
	case 5: snprintf(tempc,23, "f%d.%d%d%d%d%d\n",whole,array[0],array[1],array[2],array[3],array[4]); break;
	case 6: snprintf(tempc,23, "f%d.%d%d%d%d%d%d\n",whole,array[0],array[1],array[2],array[3],array[4],array[5]); break;
	default: snprintf(tempc,23, "f%d.%d%d%d\n",whole,array[0],array[1],array[2]); break;
	}
#endif

#ifndef SEND_TYPE_IDENTIFIERS
		switch(RESOLUTION){
		case 0: snprintf(tempc,23, "%d\n",whole); break;
		case 1: snprintf(tempc,23, "%d.%d\n",whole,array[0]); break;
		case 2: snprintf(tempc,23, "%d.%d%d\n",whole,array[0],array[1]); break;
		case 3: snprintf(tempc,23, "%d.%d%d%d\n",whole,array[0],array[1],array[2]); break;
		case 4: snprintf(tempc,23, "%d.%d%d%d%d\n",whole,array[0],array[1],array[2],array[3]); break;
		case 5: snprintf(tempc,23, "%d.%d%d%d%d%d\n",whole,array[0],array[1],array[2],array[3],array[4]); break;
		case 6: snprintf(tempc,23, "%d.%d%d%d%d%d%d\n",whole,array[0],array[1],array[2],array[3],array[4],array[5]); break;
		default: snprintf(tempc,23, "%d.%d%d%d\n",whole,array[0],array[1],array[2]); break;
		}
	#endif
	send(tempc);
}


/**************************************************
 *  Prepares the string to be sent out
**************************************************/
void send_string(char* string){
	int n = 100;
		char tempc[n];
		int ret = 0;

		#ifdef SEND_TYPE_IDENTIFIERS
			ret = snprintf(tempc, n, "s%s", string);
		#endif

		#ifndef SEND_TYPE_IDENTIFIERS
			ret = snprintf(tempc, n, "%s", string);
		#endif
		if(ret < 0){//encoding error
			ErrorHandler(SEND_STRING_ENCODING_ERROR_NUMBER);
			return;
		}
		if(ret>n){
			ErrorHandler(SEND_STRING_BUFFER_OVERFLOW_ERROR_NUMBER);
			return;
		}
		send(tempc);
}


/**************************************************
 *  Sends empty strings on usart.
**************************************************/
void SendEmpty(unsigned n){
	for(int i=0; i<n;++i){
		send(" ");
	}
	send("\n");
}

/**************************************************
 *  A function for debug purposes
**************************************************/
void send_debug(char* string){
	send(string);
}

/**************************************************
 *  Sends out a command
**************************************************/
void send_RFDuino_command(char* cmd){
	int n = 100;
		char tempc[n];
		int ret = snprintf(tempc, n, "%s", cmd);
		if(ret>n){
			ErrorHandler(SEND_STRING_BUFFER_OVERFLOW_ERROR_NUMBER);
			return;
		}
		send(tempc);
}


/**************************************************
 *  Gives an interrupt to the RFDuino
**************************************************/
void RFDuino_GiveIT(){
	SetGPIO(RFDuino_IT_PORT, RFDuino_IT_PIN, 1);
	Delay(10);
	SetGPIO(RFDuino_IT_PORT, RFDuino_IT_PIN, 0);
}
/**************************************************
 *  For debug purposes
**************************************************/
void SendRXBuffer(){
	send_string("------------------\n");
	for(int i=0; i<rx_buffer.index; ++i){
		USART_Tx(USART0, rx_buffer.data[i]);
	}
	send_string("------------------\n");
	Delay(1000);
}



/**************************************************
 *  Initializes the UART for the RFduino
**************************************************/
void InitRFduinoUART(){
	InitUART();
}
