

#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"


void EPD_W21_Init(void)
{
	EPD_W21_RST_0;		// Module reset
	driver_delay_xms(1000);//At least 10ms delay 
	EPD_W21_RST_1;
	driver_delay_xms(1000);//At least 10ms delay 
	
}

/***********************************************************
						end file
***********************************************************/

