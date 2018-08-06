

#ifndef SRC_SI7021_H_
#define SRC_SI7021_H_

/**************************************************
 *  Includes
**************************************************/

#include "i2cspm.h"
//#include <stdio.h>
#include "em_gpio.h"
#include "em_cmu.h"
#include "si7013.h"


/**************************************************
 *  Functions
**************************************************/
#define si7021_addr				0x80
#define I2CSPM_INIT_new                                                   \
  { I2C0,                      /* Use I2C instance 0 */                        \
    I2C_SCL_PORT,                 /* SCL port */                                  \
    I2C_SCL_PIN,                  /* SCL pin */                                   \
    I2C_SDA_PORT,                 /* SDA port */                                  \
    I2C_SDA_PIN,                  /* SDA pin */                                   \
    4,                         /* Location */                                  \
    0,                         /* Use currently configured reference clock */  \
    I2C_FREQ_STANDARD_MAX,     /* Set to standard rate  */                     \
    i2cClockHLRStandard,       /* Set to use 4:4 low/high duty cycle */        \
  }



void InitSI7021();
void SI7021_Measure(uint32_t *rhData, int32_t *tData);


#endif /* SRC_SI7021_H_ */
