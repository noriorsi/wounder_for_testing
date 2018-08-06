/*
 * Flash.h
 *
 *  Created on: 2017. okt. 6.
 *      Author: szmik_000
 */

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_


/*******************************************************************************
 ******************************   INCLUDES   ************************************
 ******************************************************************************/

#include "em_device.h"
#include "em_msc.h"


/*******************************************************************************
 ******************************   DEFINES   ************************************
 ******************************************************************************/

//This means that I leave a total space of ~ 45 KB for the code
#define		FLASH_START_ADDRESS		0xB000			//=45056 = 44*1024

//This is where the emulated EEPROM starts, the real flash ends at 0x10000 = 65536; the difference is 2048 = 2 pages
#define		FLASH_END_ADDRESS		0xF800			//=63488

#define		FLASH_TOTAL_SIZE			(FLASH_END_ADDRESS - FLASH_START_ADDRESS)		//=18432
#define		FLASH_MAX_NUMBER_OF_PAGES	(FLASH_TOTAL_SIZE/FLASH_PAGE_SIZE)				//=18

#define 	FLASH_NUMBER_OF_PAGES		18

//This is where to the "last data address" variable. This variable stores the address where the last valid data is in the memory.
#define		ADDRESS_OF_LAST_DATA_ADDRESS	(FLASH_START_ADDRESS-4)
#define		FLASH_LASTDATA_PAGE_START_ADDRESS	(FLASH_START_ADDRESS - FLASH_PAGE_SIZE)

#define		ERASED_VALUE		0xFFFFFFFF


/*******************************************************************************
 ******************************   STRUCTS   ************************************
 ******************************************************************************/

/*
 * The struct for keeping track of pages.
 */
typedef struct{
	  uint32_t *startAddress;
	  uint32_t *endAddress;
}Flash_Page_Typedef;


/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void InitFlash();
bool WriteToFlash(uint32_t data);
void UpdateLastDataInFlash();
void EraseAllPages();
uint32_t ReadFromFlash(uint32_t *address);


#endif /* SRC_FLASH_H_ */
