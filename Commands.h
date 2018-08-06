

#ifndef SRC_COMMANDS_H_
#define SRC_COMMANDS_H_


/**************************************************
 *  Includes
**************************************************/

#include "StateMachine.h"
#include "RFDuino.h"

/**************************************************
 *  Functions
**************************************************/

void InitCMD();
int VerifyCommand(char* data);
void ExecuteCommand(int cmd);


/**************************************************
 *  Global Defines
**************************************************/

#define COMMAND_CHARACTER   '$'


typedef enum{
	CMD_START,
	CMD_STOP,
	CMD_SLEEP,
	CMD_STARTM2,
	NUMBER_OF_COMMANDS
}command_enum;

extern char* CMD[NUMBER_OF_COMMANDS];

typedef enum{
  PARAM_PERIOD,  	//The period of the sampling of the sensor data [ms]
  PARAM_NUM,  		//The number of sensor data to be sampled
  NUMBER_OF_PARAMS
}param_enum;

extern char PARAM[NUMBER_OF_PARAMS];

#define MAX_PERIOD_DIGITS_MS    6   //Meaning the maximum period time is 999 secs
extern char param_period[MAX_PERIOD_DIGITS_MS+1];
extern int param_period_number;

#define MAX_NUM_DIGITS          3  //The maximum number of sampled data is 999
extern char param_num[MAX_NUM_DIGITS+1];
extern int param_num_number;
#define MAX_NUMBER_OF_SAMPLED_DATA		2000
extern unsigned set_params[NUMBER_OF_PARAMS];
#define PARAM_CHAR          '/'
#endif /* SRC_COMMANDS_H_ */
