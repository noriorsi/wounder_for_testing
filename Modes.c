

#include "Modes.h"
#include "RFDuino.h"


unsigned data0 = 0;
unsigned data1 = 0;
int counter = 0;
unsigned inPowerSaving = true;
double vddVoltage = VMCU;
int32_t ff[PARAMETRIC_MEASUREMENT_STORE_SIZE], humData[PARAMETRIC_MEASUREMENT_STORE_SIZE], tData[PARAMETRIC_MEASUREMENT_STORE_SIZE];

double  OFFSET, OFFSET1, OFFSET2, OFFSET4;
void EnterPowerSaving(){
	SetGPIO(MCULED1_PORT, MCULED1_PIN, 0);
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
	SetGPIO(MCULED3_PORT, MCULED3_PIN, 0);
	EnterEM3();;
}

void PowerSavingModeNotification(unsigned mode){
	if(inPowerSaving && (mode != MODE0)){
		SendEmpty(100);
		for(int i=0; i<1; ++i){
			send_string("Leaving EM3 power saving.\n");
		}
		SendEmpty(100);
		inPowerSaving = false;
	}

	if(!inPowerSaving && mode == MODE0){
		SendEmpty(100);
		for(int i=0; i<1; ++i){
			send_string("Entering EM3 power saving.\n");
		}
		SendEmpty(100);
		inPowerSaving = true;
	}
}



void ContinousMeasurement(){
/*
			uint32_t ff = GetADCvalue();
			double Volt = (ADC_to_Voltage(ff));
			double force = (Voltage_to_force(Volt));


			double hgmm = 133.3222365 * force;
			hgmm = hgmm / 38.0;

			InitRFDuino();
			//SendEmpty(n*5);

			SendEmpty(5);
			uint32_t humData;
			int32_t tData;
			SI7021_Measure(&humData, &tData);

			//send_string("Force[N]: \n");
			send_double(force);
			//send_string("F[Hgmm]: \n");
			send_double(hgmm);
			//send_string("T[°C]: \n");
			send_double(tData/1000.0);
			//send_string("H[%]: \n");
			send_double(humData/1000.0);
			//send_string("---------------------\n");
			//send_string("Voltage: \n");
			send_double(Volt);
			//send_string("---------------------\n");

			//send_string("Date\n:");
			SendDate();
			send_string("---------------------\n");

			send_int(ReadFromFlash((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS));
			SendEmpty(5);
*/
}

//CALIBRATION of FSR

double forceNewton(uint32_t f){

	double Volt = (ADC_to_Voltage(f));
	double force = (Voltage_to_force(Volt));//newton
	return force;

}

double forceing(uint32_t f){
	double temp = forceNewton(f);
	temp = temp * 101.972;// change in N and g
	return temp;
}
//finally the FSR measurement in Hgmm
// smaller active area : //double hgmm = temp*91.81188;
// for active area : 10,2 -> 1,02 cm--> Area = 0,817
// for active area 14,68mm -> 1,468 cm
// Area = pi*(d*d/4)
// p = weight(in g ) * 0.7356 / Area
// p = 101.972 * 0.7356 / 1,692552
// pkicsi = 101.972 * 0.7356 / 0.2481
//pkicsi (ha 5.08 mm) = 101.972*0.7356 / 0.20268
// hgmm = p * temp
double hgmm(uint32_t f){
	double temp = forceNewton(f);

	//double hgmm = temp * 44.31805;
	// CALIBRATED bigger fsr
	//double hgmm = temp * 91.8;
	// correct would be
	//double hgmm = temp * 302.34;
	// calibrated
	//double hgmm = temp * 260.5;
	double hgmm = temp * 302.34;
	return hgmm;
}

double hgmm_ebay(uint32_t f){
	double temp = forceNewton(f);

	//double hgmm = temp * 44.31805;
	// CALIBRATED bigger fsr
	//double hgmm = temp * 91.8;
	// correct would be
	//double hgmm = temp * 302.34;
	// calibrated
	double hgmm = temp * 270.9;
	return hgmm;
}


void Measure(int n, int period){

	EraseAllPages();
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);
	uint32_t HumData;
	int32_t TData;
	uint32_t time_ms;

	uint32_t start_time = getTime();

		for(int i=0;i<n;++i){
			ff[i] = GetADCvalue_Force(0);
			SI7021_Measure(&HumData, &TData);
			humData[i]=HumData;
			tData[i]=TData;
			time_ms = getTime()-start_time;
					//BatteryVoltage = BatteryADCMeasurement();
					WriteToFlash((uint32_t)ff[i]);
					//WriteToFlash((uint32_t)ff[i]);
					//WriteToFlash((uint32_t)ff[i]);
					WriteToFlash((uint32_t)humData[i]);
					WriteToFlash((uint32_t)tData[i]);
					//WriteToFlash((uint32_t)time_ms);
					//WriteToFlash((uint32_t) BatteryVoltage);
					//Delay(1);
					//Delay(100);
				}

			UpdateLastDataInFlash();

			RFDuino_GiveIT();
			InitRFduinoUART();
			SendEmpty(5*n);

			//send_string("------\n");

		//	for(uint32_t i=FLASH_START_ADDRESS; i<(FLASH_START_ADDRESS+(4*n*4)); i+=16){
			for(uint32_t i=FLASH_START_ADDRESS; i<(FLASH_START_ADDRESS+(3*n*4)); i+=12){
				uint32_t* address;
				uint32_t readValue;

				//force
			/*	address 	= (uint32_t*)(i);
				readValue 	= ReadFromFlash(address);
				//send_double(quickMeas(readValue));
				send_double (hgmm(readValue));

				//humidity

				address 	= (uint32_t*)(4+i);
				readValue 	= ReadFromFlash(address);
				send_double((double)readValue/1000);
				//send_double (forceing(readValue));

				//temperature

				address 	= (uint32_t*)(8+i);
				readValue 	= ReadFromFlash(address);
				send_double((double)readValue/1000);
				//send_double (hgmm(readValue));

				//time_ms
				/*address 	= (uint32_t*)(12+i);
				readValue 	= ReadFromFlash(address);
				send_int((int16_t)readValue);*/

			}

			/*for(int i=0; i<10; ++i){ //Measure battery after sending the n data
				send_int(BatteryADCMeasurement());
			}*/

			//send_string("Last data address:\n");
			//send_int(ReadFromFlash((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS));
			//SendDate();
			SendEmpty(5*n);
			SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
			EnterPowerSaving();

}




// works as vibration shaker- same function-
// now it works
//sends all the data-but not the right format. it should be changed.
// not used, just a trial
void SendAndSaveDatas(int n,int period){



		EraseAllPages();
		SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);

		uint32_t time_ms, ff, Volt, force, hgmm, humData, tData;

		uint32_t start_time = getTime(); //We started the masurement at this point

		for(int i=0; i<n ; i++){
			SI7021_Measure(&humData, &tData);
			ff = GetADCvalue();
			/*Volt = (ADC_to_Voltage(ff));
			force = (Voltage_to_force(Volt));
			hgmm  = 133.3222365 * force;
			hgmm = hgmm / 38.0;*/

			time_ms = getTime()-start_time;

			WriteToFlash((uint32_t)ff);
			//WriteToFlash((uint32_t)hgmm);
			WriteToFlash((uint32_t)humData);
			WriteToFlash((uint32_t)tData);
			WriteToFlash((uint32_t)time_ms);
			/*if (WriteToFlash((uint32_t)time_ms)==1){
				SetGPIO(MCULED3_PORT, MCULED3_PIN, 1);
			}*/
			//WriteToFlash((uint32_t) BatteryVoltage);
			Delay(2); //this delay is important, see ADXL362 ODR

		}

		UpdateLastDataInFlash();

					RFDuino_GiveIT();
					InitRFduinoUART();
					SendEmpty(5);

					send_string("------\n");


		for(uint32_t i=FLASH_START_ADDRESS; i<(FLASH_START_ADDRESS+(4*n*4)); i+=16){



						uint32_t* address;
						uint32_t readValue;

						address 	= (uint32_t*)(i);
						readValue 	= ReadFromFlash(address);
						send_int((int16_t)readValue);

						address 	= (uint32_t*)(4+i);
						readValue 	= ReadFromFlash(address);
						send_int((int16_t)readValue);

						address 	= (uint32_t*)(8+i);
						readValue 	= ReadFromFlash(address);
						send_int((int16_t)readValue);

						address 	= (uint32_t*)(12+i);
						readValue 	= ReadFromFlash(address);
						send_int((int16_t)readValue);

		}

					send_string("Last data address:\n");
					send_int(ReadFromFlash((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS));
					//SendEmpty(5);
					SendDate();

					SendEmpty(5);

					SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
					EnterPowerSaving();

}
double volt = 0;
int count = 0;
int t;
int sum;

uint32_t AVG(int n ,int f){

		uint32_t sumforavg = 0;
		for(int i=0;i<n;++i){
			ff[i] = GetADCvalue_Force(f);
			sumforavg += ff[i];
		}
		return sumforavg/n;
}

uint32_t AVG_f4(int n){

		uint32_t sumforavg = 0;
		for(int i=0;i<n;++i){
			ff[i] = GetADCvalue();
			sumforavg += ff[i];
		}
		return sumforavg/n;
}

void mesurements_for_testing(int n, int p){

	//ADC_Calibration(ADC0,adcRefVDD);
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);
	int32_t f0, f1, f2, f3, f4;
	int32_t f0_minus_offset, f1_minus_offset, f2_minus_offset, f3_minus_offset, f4_minus_offset  ;
	uint32_t HumData;
	int32_t TData;
	RFDuino_GiveIT();
	//send_string("\n");
	SendEmpty(5);
	//vddVoltage = getVDD(5);
	//send_double(vddVoltage);
	for(int i=0;i<n;++i){
		//SendEmpty(5);
		/*	f0 = GetADCvalue_Force(0);
			f1 = GetADCvalue_Force(1);
			f2 = GetADCvalue_Force(2);
			f3 = GetADCvalue_Force(3);
			f4 = GetADCvalue_Force(4);*/

		f0 = AVG(5 , 0);
		f1 = AVG(5 , 1);
		f2 = AVG(5 , 2);
		f3 = AVG(5 , 3);
		//f4 = AVG(5 , 4);
	//	f4 = GetADCvalue();
		f4 = AVG_f4(5);
		GetADCvalue_Force(5);

								send_string ("F0 = ");
								send_double(hgmm(f0));
									double temp = ADC_to_Voltage(f0);
									send_double(temp);


									send_string ("F1 = ");
									send_double(hgmm(f1));
									// just to get voltage value also:
									send_double(ADC_to_Voltage(f1));


									send_string ("F2 = ");
									send_double(hgmm(f2));
									// just to get voltage value also:
									send_double( ADC_to_Voltage(f2));

									send_string ("F3 = ");
									send_double(hgmm(f3));
									// just to get voltage value also:
									send_double(ADC_to_Voltage(f3));

									send_string ("F4 = ");
									send_double(hgmm(f4));
									// just to get voltage value also:
									send_double(ADC_to_Voltage(f4));


									send_string("\n");
									SI7021_Measure(&HumData, &TData);
									send_string("H1 = ");
									send_double((HumData/1000.0));
									send_string("T1 = ");
									send_double((TData/1000.0));
									send_string("\n");
									//Delay(0.1);
	}
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
}


void getVDDdatas(){
	//send_string("VDD [V]: \0"); send_double(vddVoltage);
		//send_string("Bat [V]: \0"); send_double(batteryVoltage);
}
