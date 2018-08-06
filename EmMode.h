/*
 * EnergyModes.h
 *
 *  Created on: 2017. ápr. 21.
 *      Author: MEMS
 */

#ifndef SRC_ENERGYMODES_H_
#define SRC_ENERGYMODES_H_

/*************************************************
 * Includes
 **************************************************/

#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "RFDuino.h"
#include "Commands.h"

/*************************************************
 * Functions
 **************************************************/

void em_EM0_Hfxo(void);
void em_EM0_Hfrco(CMU_HFRCOBand_TypeDef band);
void em_EM1_Hfxo(void);
void em_EM1_Hfrco(CMU_HFRCOBand_TypeDef band);
void em_EM2_LfrcoRTC(void);
void EnterEM3(void);
void EnterEM4(void);

#endif /* SRC_ENERGYMODES_H_ */
