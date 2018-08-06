/*
 * Flash.c
 *
 *  Created on: 2017. okt. 6.
 *      Author: szmik_000
 */

#include "Flash.h"

Flash_Page_Typedef pages[FLASH_MAX_NUMBER_OF_PAGES];
uint32_t lastDataAddress; //This is where we stored the last valid data

/*******************************************************************************
 ******************************   PROTOTYPES   *********************************
 ******************************************************************************/

/**************************************************
 *  Initializes the page variables that holds
 *  information about each page
**************************************************/
void InitFlash(){
	for(int i=0; i<FLASH_NUMBER_OF_PAGES; ++i){
	    pages[i].startAddress 			= (uint32_t *)(FLASH_START_ADDRESS + i * FLASH_PAGE_SIZE);
	    pages[i].endAddress   			= (uint32_t *)(FLASH_START_ADDRESS + i * FLASH_PAGE_SIZE + FLASH_PAGE_SIZE);
	}

	lastDataAddress = *((uint32_t*) ADDRESS_OF_LAST_DATA_ADDRESS); //Read what is stored in the memory

	uint32_t magicNumber = 0xFFFFFFFF; //in order to overwrite what is in the memory only once we need this magic number (this is the current data stored in memory)
	if(lastDataAddress==magicNumber){
		lastDataAddress = FLASH_START_ADDRESS; //We don't have any data yet
		MSC_ErasePage((uint32_t*)FLASH_LASTDATA_PAGE_START_ADDRESS);
		MSC_WriteWord((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS, &lastDataAddress, 4);
	}
}

/****************************************************************
 *  Simply writes the data to the next available place in flash
 *  This function assumes that the place it writes to is erased!
 *  It also assumes that the lastDataAddress variable is up to date.
*****************************************************************/
bool WriteToFlash(uint32_t data){
	if(lastDataAddress<FLASH_START_ADDRESS || lastDataAddress>FLASH_END_ADDRESS) return false;
	if(MSC_WriteWord((uint32_t*)lastDataAddress, &data, 4) != mscReturnOk){
		return false;
	}
	lastDataAddress += 0x4; //jump to the next available space in flash
	return true;
}


/*******************************************************************
 *  Saves the address of the last data to a specific place in flash.
********************************************************************/
void UpdateLastDataInFlash(){
	uint32_t temp = *((uint32_t*) ADDRESS_OF_LAST_DATA_ADDRESS); //Read what is stored in the memory
	if(temp!=ERASED_VALUE) MSC_ErasePage((uint32_t*)FLASH_LASTDATA_PAGE_START_ADDRESS); //erase the last data there (we have to erase the whole page)
	MSC_WriteWord((uint32_t*)ADDRESS_OF_LAST_DATA_ADDRESS, &lastDataAddress, 4); //Write the appropriate value to flash
}


/*******************************************************************
 *  Erases all data from the part of the flash that is
 *  designed for storing data.
********************************************************************/
void EraseAllPages(){
	for(int i=0; i<FLASH_NUMBER_OF_PAGES; ++i){
		 MSC_ErasePage(pages[i].startAddress);
	}
	lastDataAddress = FLASH_START_ADDRESS;
	UpdateLastDataInFlash();
}


/*******************************************************************
 *  Reads from flash and returns the data
********************************************************************/
uint32_t ReadFromFlash(uint32_t *address){
	return *(address);
}


