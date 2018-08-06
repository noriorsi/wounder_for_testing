


#include "Commands.h"


/*********************************************************************************************************************
-------------------------------------------------- Variables --------------------------------------------------
 *********************************************************************************************************************/

char* CMD[NUMBER_OF_COMMANDS];


char param_period[MAX_PERIOD_DIGITS_MS+1] = {0};
int param_period_number = 1000;
char param_num[MAX_NUM_DIGITS+1] = {0};
int param_num_number = 31;
char PARAM[NUMBER_OF_PARAMS];
unsigned set_params[NUMBER_OF_PARAMS] = {false};


/***************************************************
Initializes the command strings
***************************************************/
void InitCMD(){
   CMD[CMD_START] 		= 	"$START$";
   CMD[CMD_STOP] 		= 	"$STOP$";
   CMD[CMD_SLEEP]	 	=   "$SLEEP$";
   CMD[CMD_STARTM2]		=	"$STARTM2$";
   //InitPARAM();

}


/***************************************************
Verifies the command string
***************************************************/
int VerifyCommand(char* data){
  for(int i=0; i<NUMBER_OF_COMMANDS; ++i){
    if(strcmp(data, CMD[i])==0) return i;
  }
  return -1;
}



/***************************************************
Executes the given command
***************************************************/
void ExecuteCommand(int cmd){

  switch(cmd){


    case CMD_STOP:{
      event = STOP_EVENT;
      break;
    }

    case CMD_STARTM2:{
    	event = STARTM2_EVENT;
    	break;
    }

    default:{
      break;
    }

  }

}


/*

void SaveParam(char* data, int n){
  if(data[0] == PARAM_CHAR){
    int start_i = 0;
    unsigned start_i_found = false;
    int stop_i = 0;

    //check the parameter character stream
    for(int i=0; i<n; ++i){

      //if we havent found the place of start_index && it is a parameters char ---> we found the start_i
      if(!start_i_found && isParam(data[i])){ start_i = i; start_i_found =true;}

      //if we are not standing on the same char as before && we already found the start && it is also a param ---> we found the stop_i
      if(i!=start_i && start_i_found && isParam(data[i])) stop_i = i;

      //If its not the starting character && it is not a param char && it is also not a number ---> it is invalid so we stop here
      if(i!=start_i && !isParam(data[i]) && !isNum(data[i])) stop_i = i;

      //if we are at the end of the stream
      if(i==n-1) stop_i = i;

      if(start_i_found && stop_i > start_i){
         switch(paramIndex(data[start_i])){

           case PARAM_PERIOD:{
            if(ExtractCharactersTo(data, start_i+1, stop_i, param_period, MAX_PERIOD_DIGITS_MS)){
            	set_params[PARAM_PERIOD] = true;
            	CharToInt(param_period, (stop_i-start_i-1), &param_period_number);
            }
            break;
           }

           case PARAM_NUM:{
            if(ExtractCharactersTo(data, start_i+1, stop_i, param_num, MAX_NUM_DIGITS)){
            	set_params[PARAM_NUM] = true;
            	CharToInt(param_num, (stop_i-start_i-1), &param_num_number);
            }
            break;
           }

           default:  break; //Unkown parameter
         }
         start_i_found = true;
         start_i = stop_i; //The next paramter is at the end of the previous one
      }
    }

  }
  else{
    //Not a parameter
  }

}


void ResetParams(){
  for(int i=0; i<MAX_PERIOD_DIGITS_MS+1; ++i){
    param_period[i] = 0;
  }
  for(int i=0; i<MAX_NUM_DIGITS+1; ++i){
   param_num[i] = 0;
  }

  for(int i=0; i<NUMBER_OF_PARAMS; ++i){
    set_params[i] = false;
  }

  param_period_number = 0;
  param_num_number = 0;

}
*/
