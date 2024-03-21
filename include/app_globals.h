/**
 * @file    app_globals.h
 * @brief   Header file for application general initialization.
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

#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H

/* ----------------------------------------------------------------------------
 * Defines
 * --------------------------------------------------------------------------*/
#define LowByte(w) ((uint8_t) ((w) & 0xff))
#define HighByte(w) ((uint8_t) ((w) >> 8))
#define makeWord(a, b)   ((uint16_t)(((uint8_t)(a)) | ((uint16_t)((uint8_t)(b))) << 8))


/* ----------------------------------------------------------------------------
 * user defines
 * --------------------------------------------------------------------------*/
#define BLE_PARAM_UPDATE_TIME	 	3
#define CUSTOMSS_NOTIFY_ON_TIME		10

#define FIRMWARE_VERSION			250
#define LOW_BATTERY_LEVEL			1460

#define LSAD_APPEND_READY			0
#define LSAD_APPEND_SUCCESS			1

#define GPIO_LOW	0
#define GPIO_HIGH	1

#define CGMS_DISABLE			0
#define CGMS_ENABLE				1

#define CGMS_REF_MODE    CGMS_DISABLE
#define CGMS_AEO_MODE    CGMS_DISABLE

#define CGMS_WEP1_DEFAULT		10000
#define CGMS_WEP2_DEFAULT	 	 9500

#define CGMS_AEO_DEFAULT		65500
#define CGMS_AEP_DEFAULT		65500


#define ADV_SCAN_STOP	0
#define ADV_SCAN_RUN	1

#define AMR_WAKEUP_FAIL			0
#define AMR_WAKEUP_SUCCESS	1


#define NOT_WAKEUP_FROM_SLLEP_MODE	0x55				//																			Not wakeup from SLEEP mode
#define AMR_WAKEUP_FROM_SLLEP_MODE	0x77				//																			AMR wakeup from SLEEP mode

#define AMR_WAKEUP_SLEEP_MODE_FAIL			0
#define AMR_WAKEUP_SLEEP_MODE_SUCCESS	1


#if 0
#define LSAD_PACKET_SIZE	30
#define LSAD_BUFFER_SIZE 	600
#else
#define LSAD_PACKET_SIZE	50
#define LSAD_BUFFER_SIZE 	1000
#endif

#define LSAD_SEND_SIZE 		240
#define LSAD_READ_SIZE 		20

#define agms_info_SEND_STOP		0
#define agms_info_SEND_NEXT	1

#define PROTOCOL_HEAD						0xA0
#define PROTOCOL_MODE						0x81
#define PROTOCOL_COMPANY_ID1		0x62
#define PROTOCOL_COMPANY_ID2		0x03

#define LSAD_PACKET_SIZE_POS		5


#define RECEIVE_LSAD_STATE_ID	1
#define RECEIVE_REQUEST_RTC		2

#define RECEIVE_FOTA_READY				0x10
#define RECEIVE_ANALOG_POWER    	0x11
#define RECEIVE_NOTIFY_TIME     			0x12
#define RECEIVE_REQUEST_INFORM  	0x13

#define RECEIVE_POWER_OFF		0x20
#define RECEIVE_CALI_RESET		0x30

#define NO_ERROR				0

#define AGMS_POWER_ON				0
#define AGMS_POWER_OFF			1

#define API_FALSE				0
#define API_TRUE					1

#define UPDATE_WAIT		0
#define UPDATE_TRUE		1

#define STRIPPING_NONE				0
#define STRIPPING_RUN				1
#define STRIPPING_STOP				2

#define STRIPPING_ENABLE		1


/* ----------------------------------------------------------------------------
 * defines
 * --------------------------------------------------------------------------*/
enum {
	DATE_TIME_UPDATE_NONE=0,
	DATE_TIME_UPDATE_FAIL,
	DATE_TIME_UPDATE_SUCCESS
};


enum {
	PROTOCOL_SEND_LSAD_ID=1,
	PROTOCOL_REQUEST_RTC,
	PROTOCOL_REQUEST_REBOOT
};

enum {
	RECEIVE_NO_ERROR=0,
	RECEIVE_CRC_ERROR,
	RECEIVE_DATA_LENGTH_ERROR,
	RECEIVE_RTC_TIME_ERROR
};

enum {
	REQUEST_COMMAND_NONE_=0,
	REQUEST_DATE_TIME,
	REQUEST_NEXT_LSAD_DATA,
	RESPONSE_BASIC_INFORM,
	RESPONSE_NOTIFICATION,
	RESPONSE_POWER_OFF
};

enum {
	BASE_INFO_NONE=0,
	BASE_INFO_INIT,
	BASE_INFO_RE_INIT,
	BASE_INFO_RESPONSE,
	BASE_INFO_RECONNECT
};


typedef struct _date_time{
	uint8_t year;
	uint8_t mon;
	uint8_t day;

	uint8_t hour;
	uint8_t min;
	uint8_t sec;

	uint8_t update;
	uint8_t sync;

	uint32_t base_sec;

} current_date_time_t;

extern current_date_time_t	_current_date_time;
extern current_date_time_t	*current_date_time;

//--------------------------------------------------------------------------------

typedef struct {
	uint8_t notifying;
	uint8_t cuss_ntf_timeout;
	uint8_t date_time_update;
	uint8_t adv_scan_mode;

	uint8_t is_connect_count;
	uint8_t send_init;
	uint8_t send_count;
	uint8_t param_update_cmd;

	uint8_t nvr3_update_cmd;
	uint8_t nvr3_update_count;
	uint8_t adv_power_config;
	uint8_t power_off_cmd;

	uint8_t agms_power_mode;
	uint8_t agms_adv_start;
	uint8_t dummy1;
	uint8_t dummy2;

	uint16_t vbat_lvl_mV;			// LSAD VALUE
	uint16_t temperature;

}rsl15_info_t;
extern rsl15_info_t _rsl15_info;
extern rsl15_info_t *rsl15_info;

typedef struct {
	uint8_t  stripping_update;
	uint8_t  stripping_dump;
	uint8_t  stripping_status;
	uint8_t  measure_update;

	uint16_t weo1_lvl_mV;
	uint16_t weo2_lvl_mV;
	uint16_t wep1_lvl_mV;
	uint16_t wep2_lvl_mV;

	uint16_t ref_lvl_mV;
	uint16_t vbat_lvl_mV;			// CEM102 VALUE

	uint16_t stripping_time;			// stripping time...600sec
	uint16_t stripping_tmout;


} afe_102_t;
extern afe_102_t	_afe_102;
extern afe_102_t	*afe_102;


//--------------------------------------------------------------------------------


typedef struct {
	uint32_t sec:6;
	uint32_t min:6;
	uint32_t hour:5;
	uint32_t day:5;
	uint32_t mon:4;
	uint32_t year:5;
} tyFLAGBITS;

typedef union {
	uint32_t bAll;
	tyFLAGBITS b;
} tyFLAG;

typedef struct {
	uint8_t run_mode;
	uint8_t power_off;

	uint8_t event_command;
	uint8_t dummy2;
	uint8_t dummy3;
	uint8_t dummy4;

	uint16_t idx_head;
	uint16_t idx_tail;
	uint16_t bak_tail;

	uint8_t read_buff[LSAD_READ_SIZE];			//

	uint16_t weo1_buff[LSAD_BUFFER_SIZE+4];
	uint16_t weo2_buff[LSAD_BUFFER_SIZE+4];
	tyFLAG	 date_time[LSAD_BUFFER_SIZE+4];			// date & time
} agms_info_t;

extern agms_info_t	_agms_info;
extern agms_info_t	*agms_info;

extern uint8_t agms_payload_buff[LSAD_SEND_SIZE];


/**
 * @brief       Structure Memory Location
 * @details
 */

extern void app_device_malloc_attach(void);

#endif /* APP_GLOBALS_H */
