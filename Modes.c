

#include "Modes.h"
#include "RFDuino.h"


unsigned data0 = 0;
unsigned data1 = 0;
int counter = 0;
unsigned inPowerSaving = true;
double vddVoltage = VMCU;
double  OFFSET, OFFSET1;

int32_t ff[PARAMETRIC_MEASUREMENT_STORE_SIZE], humData[PARAMETRIC_MEASUREMENT_STORE_SIZE], tData[PARAMETRIC_MEASUREMENT_STORE_SIZE];
//int32_t f0[PARAMETRIC_MEASUREMENT_STORE_SIZE],  f1[PARAMETRIC_MEASUREMENT_STORE_SIZE],  f2[PARAMETRIC_MEASUREMENT_STORE_SIZE],  f3[PARAMETRIC_MEASUREMENT_STORE_SIZE],  f4[PARAMETRIC_MEASUREMENT_STORE_SIZE];
void EnterPowerSaving(){
	SetGPIO(MCULED1_PORT, MCULED1_PIN, 0);
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
	SetGPIO(MCULED3_PORT, MCULED3_PIN, 0);
	EnterEM3();;
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
// hgmm = p * temp
double hgmm(uint32_t f){
	double temp = forceNewton(f);

	//double hgmm = temp * 44.31805;
	// CALIBRATED bigger fsr
	//double hgmm = temp * 91.8;
	// correct would be
	//double hgmm = temp * 302.34;
	// calibrated
	double hgmm = temp * 190.5;
	return hgmm;
}
double volt = 0;
int count = 0;
int t;
int sum;

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
	vddVoltage = getVDD(5);
	send_double(vddVoltage);
	for(int i=0;i<n;++i){

			f0 = GetADCvalue_Force(0);
			f1 = GetADCvalue_Force(1);
			f2 = GetADCvalue_Force(2);
			f3 = GetADCvalue_Force(3);
			f4 = GetADCvalue_Force(4);

/***************  F0 OFFSET  *********************/
			if( f0 < 50.000){
							count++;
							sum += f0;
						}
						else{
							sum += 0;
						}

					OFFSET = sum / count;

					if (f0>OFFSET){
					f0_minus_offset = f0-OFFSET;
					}else{
						f0_minus_offset = 0.000;
					}
/**************  F1 OFFSET  *************************/
					if( f1 < 50.000){
						count++;
						sum += f1;
						}
					else{
						sum += 0;
					}
					OFFSET1 = sum / count;
					if (f1>OFFSET1){
						f1_minus_offset = f1-OFFSET1;
						f2_minus_offset = f2 - OFFSET1;
						f3_minus_offset = f3 - OFFSET1;
						f4_minus_offset = f4 - OFFSET1;
					}else{
						f1_minus_offset = 0.000;
						f2_minus_offset = 0.000;
						f3_minus_offset = 0.000;
						f4_minus_offset = 0.000;
						}

/***********************************************************/
						send_string ("F0 = ");
						send_double(hgmm(f0_minus_offset));
						send_string ("F1 = ");
						send_double(hgmm(f1_minus_offset));
						send_string ("F2 = ");
						send_double(hgmm(f2_minus_offset));
						send_string ("F3 = ");
						send_double(hgmm(f3_minus_offset));
						send_string ("F4 = ");
						send_double(hgmm(f4_minus_offset));

						send_string("\n");
						SI7021_Measure(&HumData, &TData);
						send_string("H1 = ");
						send_double(HumData/1000.0);
						send_string("T1 = ");
						send_double(TData/1000.0);

	}
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
}


void getVDDdatas(){
	send_string("VDD [V]: \0"); send_double(vddVoltage);
		//send_string("Bat [V]: \0"); send_double(batteryVoltage);
}




void Temporary_measurements(int n, int period){
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);
	int32_t f0, f1;
	int32_t force0[n];
	int32_t force1[n];
	int32_t f0_minus_offset, f1_minus_offset  ;
	uint32_t HumData;
	int32_t TData;


	RFDuino_GiveIT();
	send_string("\n");
	send_string ("Collecting datas...\n");

	send_string("\n");

	for(int i=0;i<n;++i){

		f0 = GetADCvalue_Force(0);
		f1 = GetADCvalue_Force(1);


		//force0[i]= f0;
		//force1[i]= f1;

		/****************************************/
		// this is only for offset calculation
			if( f0 < 50.000){
				count++;
				sum += f0;
			}
			else{
				sum += 0;
			}

		OFFSET = sum / count;

		if (f0>OFFSET){
		f0_minus_offset = f0-OFFSET;
		}else{
			f0_minus_offset = 0.000;
		}
		/***************************************/
		if( f1 < 50.000){
				count++;
				sum += f1;
			}
			else{
				sum += 0;
			}
		OFFSET1 = sum / count;
		if (f1>OFFSET1){
			f1_minus_offset = f1-OFFSET1;
			}else{
			f1_minus_offset = 0.000;
			}
		/**********************************/
		//SI7021_Measure(&HumData, &TData);
		force0[i]= f0_minus_offset;
		force1[i]= f1_minus_offset;

	}

		/***************************************/
		t = t+1;

		RFDuino_GiveIT();
		SendEmpty(5);

	for(int i=1;i<n;++i){
		send_string ("\n");
		send_string ("F0 = \0");
		send_double(force0[i]);
		send_string ("F1 = ");
		send_double(force1[i]);

		SI7021_Measure(&HumData, &TData);
		humData[i]=HumData;
		tData[i]=TData;
		send_string ("H = \0");
		send_double((humData[i]/1000.0));
		send_string ("T = ");
		send_double((tData[i]/1000.0));

	}

	send_double (OFFSET);
	send_double (OFFSET1);


		/***************************************/
	//volt = ADC_to_Voltage(f0_minus_offset);
	//send_string ("HGMM: \n");
    //send_double (hgmm(volt));


	SendEmpty(5);
	//send_string ("Avg OFFSET: \n");
	//send_double (OFFSET);
	//send_string ("-----------------------");
	//SendEmpty(5);
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);

}


void Measure_multipleFSR (int n, int period){

	EraseAllPages();
	SetGPIO(MCULED2_PORT, MCULED2_PIN, 1);
	uint32_t HumData;
	int32_t TData;
	uint32_t time_ms;

	int32_t f0;

	uint32_t start_time = getTime();

		for(int i=0;i<n;++i){

			/*uint32_t f0 = GetADCvalue_Force0();
			uint32_t f1 = GetADCvalue_Force1();
			uint32_t f2 = GetADCvalue_Force2();
			uint32_t f3 = GetADCvalue_Force3();
			uint32_t f4 = GetADCvalue_Force4();*/
			/*f0[i] = GetADCvalue_Force0();
			f1[i] = GetADCvalue_Force1();
			f2[i] = GetADCvalue_Force2();
			f3[i] = GetADCvalue_Force3();
			f4[i] = GetADCvalue_Force4();*/

			f0 = GetADCvalue_Force(0);

			/*f1[i] = GetADCvalue_Force(1);
			f2[i] = GetADCvalue_Force(2);
			f3[i] = GetADCvalue_Force(3);
			f4[i] = GetADCvalue_Force(4);*/
			//f0[i] = adcScanDma(0);
			/*f1[i] = adcScanDma(1);
			f2[i] = adcScanDma(2);
			f3[i] = adcScanDma(3);
			f4[i] = adcScanDma(4);*/



			//SI7021_Measure(&HumData, &TData);


			/*uint32_t humData = HumData;
			uint32_t tData=TData;*/
			//humData[i]=HumData;
			//tData[i]=TData;

			time_ms = getTime()-start_time;
					//BatteryVoltage = BatteryADCMeasurement();

					/*WriteToFlash((uint32_t)f0);
					WriteToFlash((uint32_t)f1);
					WriteToFlash((uint32_t)f2);
					WriteToFlash((uint32_t)f3);
					WriteToFlash((uint32_t)f4);
					WriteToFlash((uint32_t)humData);
					WriteToFlash((uint32_t)tData);
					*/
					WriteToFlash((uint32_t)f0);
					/*WriteToFlash((uint32_t)f1[i]);
					WriteToFlash((uint32_t)f2[i]);
					WriteToFlash((uint32_t)f3[i]);
					WriteToFlash((uint32_t)f4[i]);
					WriteToFlash((uint32_t)humData[i]);
					WriteToFlash((uint32_t)tData[i]);*/

					//WriteToFlash((uint32_t)time_ms);
					//WriteToFlash((uint32_t) BatteryVoltage);
					//Delay(2);
					//Delay(100);

				}

		  //  WriteToFlash((uint32_t)time_ms);
			UpdateLastDataInFlash();

			RFDuino_GiveIT();
			//InitRFduinoUART();
			SendEmpty(5);

			//send_string("------\n");


		//	for(uint32_t i=FLASH_START_ADDRESS; i<(FLASH_START_ADDRESS+(7*n*4)); i+=28){

			for(uint32_t i=FLASH_START_ADDRESS; i<(FLASH_START_ADDRESS+(1*n*4)); i+=4){
				uint32_t* address;
				uint32_t readValue;


				//force0
				address 	= (uint32_t*)(i);
				readValue 	= ReadFromFlash(address);
				//send_int(readValue);
				//send_double(quickMeas(readValue));
				send_double (readValue);

				//force1
				/*address 	= (uint32_t*)(4+i);
				readValue 	= ReadFromFlash(address);
				send_double (readValue);

				//force2
				address 	= (uint32_t*)(8+i);
				readValue 	= ReadFromFlash(address);
				send_double (readValue);

				//force3
				address 	= (uint32_t*)(12+i);
				readValue 	= ReadFromFlash(address);
				send_double (readValue);

				//force4
				address 	= (uint32_t*)(16+i);
				readValue 	= ReadFromFlash(address);
				send_double (readValue);

				//humidity

				address 	= (uint32_t*)(20+i);
				readValue 	= ReadFromFlash(address);
				send_double((double)readValue/1000);


				//temperature

				address 	= (uint32_t*)(24+i);
				readValue 	= ReadFromFlash(address);
				send_double((double)readValue/1000);*/

		}

			/*for(int i=0; i<10; ++i){ //Measure battery after sending the n data
				send_int(BatteryADCMeasurement());
			}*/

			//send_string("Last data address:\n");
			//send_int(ReadFromFlash((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS));
			//SendDate();
			//SendEmpty(5*n);
			SetGPIO(MCULED2_PORT, MCULED2_PIN, 0);
			//EnterPowerSaving();


}




