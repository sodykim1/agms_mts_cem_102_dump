/**
 * @file    app_init.c
 * @brief   Source file for application general initialization.
 *
 * @copyright @parblock
 * Copyright (c) 2023 Semiconductor Components Industries, LLC (d/b/a
 * onsemi), All Rights Reserved
 *
 * This code is the property of onsemi and may not be redistributed
 * in any form without prior written permission from onsemi.
 * The terms of use and warranty for this code are covered by contractual
 * agreements between onsemi and the licensee.
 *
 * This is Reusable Code.
 * @endparblock
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */

/* Device and library headers */
#include <hw.h>

/* Application headers */
#include "app.h"

#include "app_init.h"
#include "app_utility.h"

#include "app_globals.h"

/* ----------------------------------------------------------------------------
 * Private Symbolic Constants
 * ------------------------------------------------------------------------- */
#define MAIN_VERSION		3
#define SUB1_VERSION		0
#define SUB2_VERSION		0


/* ----------------------------------------------------------------------------
 * Private Macros
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Private Type Definitions
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Global Variables
 * ------------------------------------------------------------------------- */
uint8_t end_of_day;
uint8_t month_day[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};



/* ----------------------------------------------------------------------------
 * Private Function Prototypes
 * ------------------------------------------------------------------------- */

/**
 * @brief       Initialize the system clocks
 * @details     Configures the system clock to use the XTAL oscillator and sets
 *              the RF clock as the system clock.
 * @param       N/A
 * @return      N/A
 */


/**
 * @brief       Initialize the standby (low-power) clocks
 * @details     Configures the standby clock to the XTAL32 or RC32 oscillators.
 * @param       N/A
 * @return      N/A
 */


/* ----------------------------------------------------------------------------
 * Private Function Definitions
 * ------------------------------------------------------------------------- */
void sys_delay_10ms(uint8_t ms)
{
	uint8_t tmout=0;

	tmout = ms;

	do {
        /* Refresh the watchdog timers */
		SYS_WATCHDOG_REFRESH();
		Sys_Delay((uint32_t)(0.01*SystemCoreClock));					// 10ms * ms =

	}while(tmout-->0);
}



//---------------------------------------------------------------------------------------------
//	COMPILE DATA & TIME
//---------------------------------------------------------------------------------------------
void set_date_time_default_clock(void)
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t month;
	uint16_t year;

	year = 0;
	year = (BUILD_YEAR_CH0-'0')*1000;
	year+= (BUILD_YEAR_CH1-'0')*100;
	year+= (BUILD_YEAR_CH2-'0')*10;
	year+= (BUILD_YEAR_CH3-'0');

	month = 0;
	month = (BUILD_MONTH_CH0-'0')*10;
	month+= (BUILD_MONTH_CH1-'0');

	day = 0;
	day = (BUILD_DAY_CH0-'0')*10;
	day+= (BUILD_DAY_CH1-'0');

	hour = 0;
	hour = (BUILD_HOUR_CH0-'0')*10;
	hour+= (BUILD_HOUR_CH1-'0');

	min = 0;
	min = (BUILD_MIN_CH0-'0')*10;
	min+= (BUILD_MIN_CH1-'0');

	sec = 0;
	sec = (BUILD_SEC_CH0-'0')*10;
	sec+= (BUILD_SEC_CH1-'0');

	set_current_date_time((year-2000), month, day, hour, min, sec);
}


void Product_Information_Service(void)
{
#ifdef SWMTRACE_OUTPUT
	swmLogInfo("*********************************************************************\r\n");
	swmLogInfo("*                                                                   *\r\n");
	swmLogInfo("* AGMS T10 VER %d.%d.%d | %s , %s has started.\r\n", MAIN_VERSION, SUB1_VERSION, SUB2_VERSION, __DATE__, __TIME__);
	swmLogInfo("*                                                                   *\r\n");
	swmLogInfo("*********************************************************************\r\n");
#endif
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	app_utils
//
uint32_t progress_time_condition(uint32_t taget_time, uint32_t check_time)
{
	uint32_t current_time_mm=0;
	uint32_t progress_time=0;

	current_time_mm = Sys_RTC_Value_Seconds();

	if(current_time_mm == taget_time)
		return 0;

	if(current_time_mm > taget_time)
		progress_time = current_time_mm -taget_time;
	else
		progress_time =(0xFFFFFFFF - taget_time) + current_time_mm;

	if(progress_time >= check_time)
	{
		if(current_time_mm == 0)
			current_time_mm++;
		return current_time_mm;
	}
	return 0;
}


uint32_t progress_current_time(void)
{
	uint32_t current_time_mm=0;
	uint32_t progress_time=0;

	current_time_mm = Sys_RTC_Value_Seconds();

	if(current_time_mm == current_date_time->base_sec)
		return 0;

	if(current_time_mm > current_date_time->base_sec)
		progress_time = current_time_mm -current_date_time->base_sec;
	else
		progress_time =(0xFFFFFFFF - current_date_time->base_sec) + current_time_mm;

	current_date_time->base_sec = current_time_mm;

	return progress_time;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	current data & time setting
//
void set_current_date_time(uint8_t year, uint8_t mon, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
	current_date_time->year = year;
	current_date_time->mon = mon ;
	current_date_time->day = day;
	current_date_time->hour = hour;
	current_date_time->min = min;
	current_date_time->sec = sec;

	current_date_time->base_sec = Sys_RTC_Value_Seconds();				// RTC CLOCK
}

void get_current_date_time(void)
{
	uint8_t second=0;
	uint8_t minute=0;
	uint32_t progress_time;

	progress_time = progress_current_time();

	if(progress_time>(CUSTOMSS_NOTIFY_ON_TIME*10)) {										// 10min * 60 = 600...error  -> jump rtc time 10min
		current_date_time->sec +=CUSTOMSS_NOTIFY_ON_TIME;		// default time...10sec
		current_date_time->update = false;							// RTC Time 을 다시 갱신 한다...
	}

	else {
		if(progress_time>60) {
			minute = (uint8_t)((float)progress_time/60);
			second = progress_time%60;

			current_date_time->min +=(uint8_t)minute;
			current_date_time->sec +=(uint8_t)second;
		}
		else {
			current_date_time->sec +=(uint8_t)progress_time;
		}
	}

//---------------------------------------------------------------------------------------------------------------------------------------------
//	CALCULATION RTC TIME
//---------------------------------------------------------------------------------------------------------------------------------------------
	if(current_date_time->sec>=60) {
		current_date_time->sec-=60;
		current_date_time->min++;
	}

	if(current_date_time->min>=60) {
		current_date_time->min-=60;
		current_date_time->hour++;
	}

	if(current_date_time->hour>=24) {
		current_date_time->hour-=24;
		current_date_time->day++;
	}

	if(current_date_time->mon==2) {
		if(((current_date_time->year%4==0) && (current_date_time->year%100 != 0)) || (current_date_time->year%400==0 ))		{
			month_day[1] = 29;
		}
		else {
			month_day[1] = 28;
		}
	}

	end_of_day = month_day[current_date_time->mon-1];
	if(current_date_time->day>end_of_day) {
		current_date_time->day = 1;
		current_date_time->mon++;
	}

	if(current_date_time->mon>12) {
		current_date_time->mon=1;
		current_date_time->year++;
	}

#if 0
	swmLogInfo("get_datetime : %4d/%02d/%02d %02d:%02d:%02d [%4d]\r\n ", current_date_time->year, current_date_time->mon, current_date_time->day,
			current_date_time->hour, current_date_time->min, current_date_time->sec, progress_time);
#endif

}


/***********************************************************************************************************************************
  *		FLASH MEMORY INITIAL & READ & WRITE
  ***********************************************************************************************************************************/
void App_Flash_Config_Init(void)
{
    /* Power up and initialize flash timing registers based on SystemClock */
    Flash_Initialize(0, FLASH_CLOCK_16MHZ);
}




FlashStatus_t App_Write_Flash_Buffer(uint32_t addr, uint32_t *result_data, size_t value_size)
{
    /* Perform a sector erase in the default endurance mode */
    FlashStatus_t result;

    result = Flash_EraseSector(addr, 0);
    if (result != FLASH_ERR_NONE)
    	return result;

    result = Flash_WriteBuffer(addr, value_size, result_data, 0);
    if (result != FLASH_ERR_NONE)
    	return result;

    return FLASH_ERR_NONE;
}


FlashStatus_t App_Read_Flash_Buffer(uint32_t addr, uint32_t *result_data, size_t value_size)
{
    FlashStatus_t result;

    /* Perform a re-verification of written data for illustration purposes */
    result = Flash_ReadBuffer(addr, (uint32_t)result_data, value_size);
    if (result != FLASH_ERR_NONE)
    	return result;

    return FLASH_ERR_NONE;
}


uint32_t Read_Deep_Sleep_Code(void)
{
	uint32_t nvr4_buffer[4];

	App_Read_Flash_Buffer(FLASH0_NVR4_BASE, nvr4_buffer, 4);

	return nvr4_buffer[0];

}


void Write_Deep_Sleep_Code(uint32_t  code_data)
{
	uint32_t nvr4_buffer[2];


	nvr4_buffer[0] = code_data;
	nvr4_buffer[1] = 0x00000000;

	App_Write_Flash_Buffer(FLASH0_NVR4_BASE, nvr4_buffer, 4);
}


