/*
 * Modes.h
 *
 *  Created on: 27 Apr 2017
 *      Author: Miklós
 */

#ifndef SRC_MODES_H_
#define SRC_MODES_H_

/*************************************************
 * Includes
 **************************************************/

#include "GPIO.h"
#include "SI7021.h"
#include "ADC.h"
#include "RFDuino.h"
#include "EmMode.h"
#include "RTC.h"
#include "EEPROM.h"
#include "Flash.h"
#include <string.h>
#include <stdio.h>
/*************************************************
 * Functions
 **************************************************/

void EnterPowerSaving();
void EnterEM2(void);
void MotionSensing();
void PowerSavingModeNotification(unsigned mode);
void getVDDdatas();
void ContinousMeasurement_for5fsr();
void Temporary_measurements(int n, int period);
void Measure_multipleFSR (int n, int period);
void ResetSystem(void);
//void SendAndSaveDatasPeriod(int n, int period);
void readADConly();
void Measure(int n, int period);
double forceNewton(uint32_t f);
double hgmm (uint32_t f);
double quickMeas(uint32_t f);
double forceing(uint32_t f);
void mesurements_for_testing(int n, int p);
uint32_t AVG(int n ,int f);
uint32_t AVG_f4(int n, int f);

extern double vddVoltage;

/*************************************************
 * Defines
 **************************************************/

#define MODE0		0
#define MODE1		1
#define MODE2		2
#define MODE3		3

#define PARAMETRIC_MEASUREMENT_STORE_SIZE	10

#endif /* SRC_MODES_H_ */
