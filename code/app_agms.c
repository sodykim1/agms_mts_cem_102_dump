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
#include "app_agms.h"
#include "app_utility.h"

/* ----------------------------------------------------------------------------
 * Private Symbolic Constants
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Private Macros
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Private Type Definitions
 * ------------------------------------------------------------------------- */


/* ----------------------------------------------------------------------------
 * Global Variables
 * ------------------------------------------------------------------------- */

/* ----------------------------------------------------------------------------
 * Private Function Prototypes
 * ------------------------------------------------------------------------- */

#define MAX_LOOP_COUNT	60

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	agms handler
//
uint16_t lsad_calc_crc16(uint8_t length)
{
    uint8_t i, j;
    unsigned int temp, temp2, flag;

    temp = 0xFFFF;

    for (i = 0; i < length; i++)    {
        temp = temp ^ agms_payload_buff[i];

        for (j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;

            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;
}



void append_data_time_payload(uint16_t idx, uint8_t year, uint8_t mon, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec)
{
	agms_info->date_time[idx].b.year = year;
	agms_info->date_time[idx].b.mon = mon;
	agms_info->date_time[idx].b.day = day;
	agms_info->date_time[idx].b.hour = hour;
	agms_info->date_time[idx].b.min = min;
	agms_info->date_time[idx].b.sec = sec;
}

void load_data_time_payload(uint16_t idx, uint8_t *year, uint8_t *mon, uint8_t *day, uint8_t *hour, uint8_t *min, uint8_t *sec)
{
	*year =agms_info->date_time[idx].b.year;
	*mon = agms_info->date_time[idx].b.mon;
	*day = agms_info->date_time[idx].b.day;
	*hour =agms_info->date_time[idx].b.hour;
	*min = agms_info->date_time[idx].b.min;
	*sec = agms_info->date_time[idx].b.sec;
}


/***********************************************************************************************
  *	CEM102 CALIBRATION ...weo1 & weo2 must 1000&1000...보정 중이라는 표시 중복을 피하기 위해 10000일 경우 10001변경 한다.
  *	calibration 중에는 10000 & 10000을 전송 한 뒤 앱에서 보정 중이라고 표시 하게 한다..기존 프로토콜을 활용...
  ***********************************************************************************************/
void append_lsad_lvl_payload(void)
{
	if (afe_102->measure_update) {
		if(afe_102->weo1_lvl_mV==DAC_TARGET_BASE_MV)
			afe_102->weo1_lvl_mV+=1;

		if(afe_102->weo2_lvl_mV==DAC_TARGET_BASE_MV)
			afe_102->weo2_lvl_mV+=1;
	}
	else {
		afe_102->weo1_lvl_mV = afe_102->weo2_lvl_mV = DAC_TARGET_BASE_MV;					// calibration message
		afe_102->wep1_lvl_mV = afe_102->wep2_lvl_mV = DAC_TARGET_BASE_MV;					// calibration message
	}


	agms_info->weo1_buff[agms_info->idx_head] = afe_102->weo1_lvl_mV;
	agms_info->weo2_buff[agms_info->idx_head] = afe_102->weo2_lvl_mV;

	get_current_date_time();
	append_data_time_payload(agms_info->idx_head, current_date_time->year, current_date_time->mon, current_date_time->day,
			current_date_time->hour, current_date_time->min, current_date_time->sec);

	agms_info->idx_head++;
	agms_info->idx_head%=LSAD_BUFFER_SIZE;																	// max buff size....jump 0 point
	if(agms_info->idx_head==agms_info->idx_tail) {																// if 1 cycle head point, tail point moving
		agms_info->idx_tail++;
		agms_info->idx_tail%=LSAD_BUFFER_SIZE;
	}

#ifdef SWMTRACE_OUTPUT
	swmLogInfo("append payload : [%d]/[%d]/[%d] [%d]:[%d]:[%d] || weo1 : [%d] weo2 : [%d]  -> H:[%d] / T:[%d]\r\n", current_date_time->year, current_date_time->mon, current_date_time->day,
			current_date_time->hour, current_date_time->min, current_date_time->sec, afe_102->weo1_lvl_mV, afe_102->weo2_lvl_mV, agms_info->idx_head, agms_info->idx_tail);
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//

//==================================================================================================================================
//	payload packet data send
//	real data count is agms_info->idx_tail....data load count agms_info->bak_tail
//==================================================================================================================================
void send_payload_packet(uint8_t conidx)
{
	int loop;
	uint8_t year, mon, day;
	uint8_t hour, min, sec;

	uint16_t aeo, weo;
	uint16_t packet_count;
	uint16_t send_idx;
	uint16_t crc16=0;

	agms_info->bak_tail = agms_info->idx_tail;									// 응답을 받지 못하면 다시 재 전송을 하도록 하기 위해 응답 받은 경우에만

	memset(agms_payload_buff, 0, sizeof(agms_payload_buff));
	send_idx = packet_count = 0;

	agms_payload_buff[send_idx++] = PROTOCOL_HEAD;
	agms_payload_buff[send_idx++] = PROTOCOL_MODE;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID1;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID2;

	agms_payload_buff[send_idx++] = PROTOCOL_SEND_LSAD_ID;
	agms_payload_buff[send_idx++] = packet_count;

	load_data_time_payload(agms_info->bak_tail, &year, &mon, &day, &hour, &min, &sec);
	agms_payload_buff[send_idx++] = year;
	agms_payload_buff[send_idx++] = mon;
	agms_payload_buff[send_idx++] = day;
	agms_payload_buff[send_idx++] = hour;
	agms_payload_buff[send_idx++] = min;
	agms_payload_buff[send_idx++] = sec;

	loop = MAX_LOOP_COUNT;																															// just only loop time..60
	do {
		weo = agms_info->weo1_buff[agms_info->bak_tail];
		aeo = agms_info->weo2_buff[agms_info->bak_tail];

		agms_payload_buff[send_idx++] = HighByte(weo);
		agms_payload_buff[send_idx++] = LowByte(weo);

		agms_payload_buff[send_idx++] = HighByte(aeo);
		agms_payload_buff[send_idx++] = LowByte(aeo);

		packet_count++;

		agms_info->bak_tail++;
		agms_info->bak_tail%=LSAD_BUFFER_SIZE;

		if(agms_info->bak_tail==agms_info->idx_head) {												// end of buffer
			agms_info->run_mode = agms_info_SEND_STOP;											// stop
			break;
		}

		if(packet_count>=LSAD_PACKET_SIZE) {															// max packet size...50
			agms_info->run_mode = agms_info_SEND_NEXT;											// rest data send next time again
			break;
		}
	} while(loop-->0);																										// max loop 60


	agms_payload_buff[LSAD_PACKET_SIZE_POS] = packet_count;											// data length | Time | weo data | aeo data | .......

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
	agms_payload_buff[send_idx++] = HighByte(afe_102->wep1_lvl_mV);
	agms_payload_buff[send_idx++] = LowByte(afe_102->wep1_lvl_mV);

	agms_payload_buff[send_idx++] = HighByte(afe_102->wep2_lvl_mV);
	agms_payload_buff[send_idx++] = LowByte(afe_102->wep2_lvl_mV);

	agms_payload_buff[send_idx++] = HighByte(rsl15_info->temperature);
	agms_payload_buff[send_idx++] = LowByte(rsl15_info->temperature);

	agms_payload_buff[send_idx++] = HighByte(rsl15_info->vbat_lvl_mV);
	agms_payload_buff[send_idx++] = LowByte(rsl15_info->vbat_lvl_mV);


	crc16 = lsad_calc_crc16(send_idx);
	agms_payload_buff[send_idx++] = HighByte(crc16);
	agms_payload_buff[send_idx++] = LowByte(crc16);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//	data send..
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
	/* Send notification to peer device */
    GATTC_SendEvtCmd(
        conidx,
        GATTC_NOTIFY,
        0,
        GATTM_GetHandle(CUST_SVC0, CS_TX_VALUE_VAL0),
		send_idx,
		agms_payload_buff
    );


#ifdef SWMTRACE_OUTPUT
    swmLogInfo("send payload : [%d] -> ", packet_count);
    swmLogInfo("%02d/%02d/%02d %02d:%02d:%02d ", agms_payload_buff[6], agms_payload_buff[7], agms_payload_buff[8],
    		agms_payload_buff[9],agms_payload_buff[10],agms_payload_buff[11]);

    swmLogInfo("weo:[%02x][%02x][%02x][%02x]  send size : [%d]\r\n",
    		agms_payload_buff[12], agms_payload_buff[13], agms_payload_buff[14], agms_payload_buff[15],send_idx);
#endif


}

//==================================================================================================================================
//	payload packet data send
//==================================================================================================================================
void get_current_data_time_cmd(uint8_t conidx, uint8_t state)
{
	uint16_t send_idx;
	uint16_t crc16=0;

	memset(agms_payload_buff, 0, sizeof(agms_payload_buff));

	send_idx = 0;

	agms_payload_buff[send_idx++] = PROTOCOL_HEAD;
	agms_payload_buff[send_idx++] = PROTOCOL_MODE;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID1;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID2;

	agms_payload_buff[send_idx++] = PROTOCOL_REQUEST_RTC;
	agms_payload_buff[send_idx++] = state;

	agms_payload_buff[send_idx++] = HighByte(LOW_BATTERY_LEVEL);
	agms_payload_buff[send_idx++] = LowByte(LOW_BATTERY_LEVEL);

	agms_payload_buff[send_idx++] = HighByte(FIRMWARE_VERSION);
	agms_payload_buff[send_idx++] = LowByte(FIRMWARE_VERSION);

	agms_payload_buff[send_idx++] = HighByte(CUSTOMSS_NOTIFY_ON_TIME);
	agms_payload_buff[send_idx++] = LowByte(CUSTOMSS_NOTIFY_ON_TIME);

	crc16 = lsad_calc_crc16(send_idx);
	agms_payload_buff[send_idx++] = HighByte(crc16);
	agms_payload_buff[send_idx++] = LowByte(crc16);


#ifdef SWMTRACE_OUTPUT
	swmLogInfo("get_current_data_time_cmd : [%d]/[%d]\r\n!!\r\n", conidx, send_idx);
#endif

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//	data send..
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
	/* Send notification to peer device */
	GATTC_SendEvtCmd(
		conidx,
		GATTC_NOTIFY,
		0,
		GATTM_GetHandle(CUST_SVC0, CS_TX_VALUE_VAL0),
		send_idx,
		agms_payload_buff
	);


}



//====================================================================================================
//	DATA RECEIVE TIMEOUT
//====================================================================================================
void data_receive_timeout( uint8_t conidx, uint8_t state)
{
//	uint16_t i;
	uint16_t crc16;
	uint16_t send_idx=0;

	send_idx = 0;

	memset(agms_payload_buff, 0, sizeof(agms_payload_buff));

	agms_payload_buff[send_idx++] = PROTOCOL_HEAD;
	agms_payload_buff[send_idx++] = PROTOCOL_MODE;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID1;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID2;

	agms_payload_buff[send_idx++] = PROTOCOL_REQUEST_REBOOT;
	agms_payload_buff[send_idx++] = state;

	crc16 = lsad_calc_crc16(send_idx);
	agms_payload_buff[send_idx++] = HighByte(crc16);
	agms_payload_buff[send_idx++] = LowByte(crc16);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//	data send..
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
	/* Send notification to peer device */
	GATTC_SendEvtCmd(
		conidx,
		GATTC_NOTIFY,
		0,
		GATTM_GetHandle(CUST_SVC0, CS_TX_VALUE_VAL0),
		send_idx,
		agms_payload_buff
	);
}

//====================================================================================================
//	RESPONSE POWER OFF
//====================================================================================================
void response_power_off_cmd( uint8_t conidx)
{
	uint16_t crc16;
	uint16_t send_idx=0;

	send_idx = 0;
	memset(agms_payload_buff, 0, sizeof(agms_payload_buff));

	agms_payload_buff[send_idx++] = PROTOCOL_HEAD;
	agms_payload_buff[send_idx++] = PROTOCOL_MODE;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID1;
	agms_payload_buff[send_idx++] = PROTOCOL_COMPANY_ID2;

	agms_payload_buff[send_idx++] = RECEIVE_POWER_OFF;
	agms_payload_buff[send_idx++] = NO_ERROR;

	crc16 = lsad_calc_crc16(send_idx);
	agms_payload_buff[send_idx++] = HighByte(crc16);
	agms_payload_buff[send_idx++] = LowByte(crc16);


//----------------------------------------------------------------------------------------------------------------------------------------------------------------
//	data send..
//----------------------------------------------------------------------------------------------------------------------------------------------------------------
	/* Send notification to peer device */
	GATTC_SendEvtCmd(
		conidx,
		GATTC_NOTIFY,
		0,
		GATTM_GetHandle(CUST_SVC0, CS_TX_VALUE_VAL0),
		send_idx,
		agms_payload_buff
	);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	data receive event handler
//
void progress_receive_msg_event(uint8_t conidx, const uint8_t *dest_buff)
{
	uint8_t cmd_code;
	uint8_t cmd_state;

	if((dest_buff[0]==PROTOCOL_HEAD) && (dest_buff[1]==PROTOCOL_MODE)) {
        cmd_code = dest_buff[2];
        cmd_state= dest_buff[3];

        rsl15_info->adv_power_config = API_TRUE;														// advertising control

        switch(cmd_code) {

			case RECEIVE_LSAD_STATE_ID:
				switch(cmd_state) {
					case RECEIVE_NO_ERROR:
						agms_info->idx_tail = agms_info->bak_tail;										// receive is ok...tail position move

						if(agms_info->run_mode == agms_info_SEND_NEXT) {

							 ke_timer_set(APP_CUSS_AGMS_NEXT_SEND_DATA, KE_BUILD_ID(TASK_APP, conidx),TIMER_SETTING_S(1));	// after 2sec, data send
#ifdef SWMTRACE_OUTPUT
							 swmLogInfo("Next Data Send!!!  [%d] / [%d] \r\n", agms_info->idx_head, agms_info->idx_tail);
#endif
						}
						else {
#ifdef SWMTRACE_OUTPUT
							swmLogInfo("Data End!!!  [%d] / [%d] \r\n", agms_info->idx_head, agms_info->idx_tail);
#endif
						}

						break;

					case RECEIVE_CRC_ERROR:
						break;

					case RECEIVE_DATA_LENGTH_ERROR:
						break;

					case RECEIVE_RTC_TIME_ERROR:

						rsl15_info->date_time_update=DATE_TIME_UPDATE_FAIL;
						ke_timer_set(APP_CUSS_AGMS_REQ_DATE_TIME, KE_BUILD_ID(TASK_APP, conidx),TIMER_SETTING_S(3));	// 5sec interval

						break;

					default:
						break;
				}
				break;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	DATA RTC TIME RECEIVE MESSAGE
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case RECEIVE_REQUEST_RTC:
#ifdef SWMTRACE_OUTPUT
				swmLogInfo("Date Time Update Success!!! \r\n");
#endif

				set_current_date_time(dest_buff[4], dest_buff[5], dest_buff[6],	dest_buff[7], dest_buff[8], dest_buff[9]);
				rsl15_info->date_time_update=DATE_TIME_UPDATE_SUCCESS;

				break;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	NOTIFICATION TIME CHANGE
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case RECEIVE_NOTIFY_TIME:
#ifdef SWMTRACE_OUTPUT
				swmLogInfo("RECEIVE_NOTIFY_TIME!! \r\n");
#endif

				break;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	FOTA READY COMMAND
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case RECEIVE_FOTA_READY:
#ifdef SWMTRACE_OUTPUT
				swmLogInfo("RECEIVE_FOTA_READY!! \r\n");
#endif

				set_sys_fota_dfu(SYS_FOAT_DFU_RUN) ;

				break;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	POWER OFF COMMAND
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case RECEIVE_POWER_OFF:
#ifdef SWMTRACE_OUTPUT
				swmLogInfo("RECEIVE_POWER_OFF!! \r\n");
#endif

#if 0
				set_sys_fota_dfu(SYS_FOAT_DFU_RUN) ;
#else

				Write_Deep_Sleep_Code(DEEP_SLEEP_CODE_ENTER);						// ENTER POWER OFF WRITE

				/* Go to Deep Sleep */
				Switch_NoRetSleep_Mode();
#endif
				break;

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//	BASE INFORMATION REQUEST COMMAND
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
			case RECEIVE_REQUEST_INFORM:
#ifdef SWMTRACE_OUTPUT
				swmLogInfo("RECEIVE_REQUEST_INFORM!! \r\n");
#endif

				ke_timer_set(APP_CUSS_AGMS_REQ_BASIC_INFO, KE_BUILD_ID(TASK_APP, conidx),TIMER_SETTING_S(2));
				set_agms_base_info(BASE_INFO_RESPONSE);
				break;

			default:
				break;
        }
	}
}

