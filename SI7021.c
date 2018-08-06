
#include "SI7021.h"
#include "RFDuino.h"
/******************************************************************************************************************
 *  Static variables
******************************************************************************************************************/

//static 	uint32_t 	humData;
//static 	int32_t  	tData;
static I2CSPM_Init_TypeDef i2cInit = I2CSPM_INIT_new;

/*********************************************************************
 * Initializes SI7021 temperature and humidity sensor
*********************************************************************/
void InitSI7021(){
	/* Enable GPIO clock */
	  CMU_ClockEnable(cmuClock_GPIO, true);


	  I2CSPM_Init(&i2cInit);
}



void SI7021_Measure(uint32_t *humData, int32_t *tData){
	Si7013_MeasureRHAndTemp(i2cInit.port, si7021_addr, humData, tData);
}






