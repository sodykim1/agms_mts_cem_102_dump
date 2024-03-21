/**
 * @file  app_customss.h
 * @brief Application-specific Bluetooth custom service server header file
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

#ifndef APP_CUSTOMSS_H
#define APP_CUSTOMSS_H

/* ----------------------------------------------------------------------------
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif    /* ifdef __cplusplus */

/* ----------------------------------------------------------------------------
 * Include files
 * --------------------------------------------------------------------------*/
#include <gattc_task.h>

/* ----------------------------------------------------------------------------
 * Defines
 * --------------------------------------------------------------------------*/
#define CS_TX_PERMISSIONS               (PERM(RD, ENABLE) \
                                         | PERM(WRITE_REQ, ENABLE) \
                                         | PERM(WRITE_COMMAND, ENABLE) \
                                         | PERM(RP, SEC_CON))
#define CS_RX_PERMISSIONS               (PERM(RD, ENABLE) | PERM(NTF, ENABLE))

/* Custom service UUIDs and characteristics */

/* Custom service UUIDs */
#define CS_SVC_UUID                     { 0x24, 0xdc, 0x0e, 0x6e, 0x01, 0x40, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }

/**
 * @brief       The UUIDs for the characteristics of custom service 0
 * @details     These 128-bit UUIDs are used during Bluetooth Low Energy
 *              communications to identify the various characteristics and data
 *              that belong to custom service 0.
 *              Refer to the ReadMe for details about the functions of each
 *              characteristic.
 */
#define CS_CHAR_TX_UUID                 { 0x24, 0xdc, 0x0e, 0x6e, 0x03, 0x40, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }

#define CS_CHAR_RX_UUID                 { 0x24, 0xdc, 0x0e, 0x6e, 0x02, 0x40, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }

//------------------------------------------------------------------------------------------------------

#define CS_CHAR_BAT_CEM102_UUID         { 0x24, 0xdc, 0x0e, 0x6e, 0x02, 0x60, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }
#define CS_CHAR_WE1_UUID                { 0x24, 0xdc, 0x0e, 0x6e, 0x03, 0x60, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }
#define CS_CHAR_WE2_UUID                { 0x24, 0xdc, 0x0e, 0x6e, 0x04, 0x60, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }
#define CS_CHAR_RE_UUID                 { 0x24, 0xdc, 0x0e, 0x6e, 0x05, 0x60, \
                                          0xca, 0x9e, 0xe5, 0xa9, 0xa3, 0x00, \
                                          0xb5, 0xf3, 0x93, 0xe0 }

#define CS_VBAT_MAX_LENGTH              2
#define CS_WE1_MAX_LENGTH               4
#define CS_WE2_MAX_LENGTH               4
#define CS_RE_MAX_LENGTH                4

#define CS_VALUE_MAX_LENGTH     		  (20)    /* Used by the TX and RX characteristics */


#define CS_CHAR_TX_NAME                 "TX_VALUE"
#define CS_CHAR_RX_NAME                 "RX_VALUE"


#define CS_VBAT_CEM102_CHAR_NAME        "CEM102 Battery (mV)"
#define CS_WE1_CHAR_NAME                "Working Electrode 1 (nA)"
#define CS_WE2_CHAR_NAME                "Working Electrode 2 (nA)"
#define CS_RE_CHAR_NAME                 "Reference Electrode (nA)"

/* Custom service ID */
/* Used in calculating attribute number for given custom service */
enum CUST_SVC_ID
{
    CUST_SVC0,
    CUST_SVC1,
};

enum CS0_att
{
    /* Service 0 */
    CS_SERVICE0,

    /* TX Characteristic in Service 0 */
	CS_TX_VALUE_CHAR0,      /* Characteristic */
	CS_TX_VALUE_VAL0,       /* Value */
	CS_TX_VALUE_CCC0,       /* Client Characteristic Configuration */
	CS_TX_VALUE_USR_DSCP0,  /* User Description */

    /* RX Characteristic in Service 0 */
	CS_RX_VALUE_CHAR0,
	CS_RX_VALUE_VAL0,
	CS_RX_VALUE_CCC0,
	CS_RX_VALUE_USR_DSCP0,

    /* Reference electrode Characteristic in Service 0 */
    CS_VBAT_CEM102_VALUE_CHAR0,
    CS_VBAT_CEM102_VALUE_VAL0,
    CS_VBAT_CEM102_VALUE_CCC0,
    CS_VBAT_CEM102_VALUE_USR_DSCP0,

    /* Working electrode 1 Characteristic in Service 0 */
    CS_WE1_VALUE_CHAR0,
    CS_WE1_VALUE_VAL0,
    CS_WE1_VALUE_CCC0,
    CS_WE1_VALUE_USR_DSCP0,

    /* Working electrode 2 Characteristic in Service 0 */
    CS_WE2_VALUE_CHAR0,
    CS_WE2_VALUE_VAL0,
    CS_WE2_VALUE_CCC0,
    CS_WE2_VALUE_USR_DSCP0,

    /* Reference electrode Characteristic in Service 0 */
    CS_RE_VALUE_CHAR0,
    CS_RE_VALUE_VAL0,
    CS_RE_VALUE_CCC0,
    CS_RE_VALUE_USR_DSCP0,

    /* Max number of services and characteristics */
    CS_NB0,
};

struct app_env_tag_cs
{
    /* From Bluetooth transfer buffer (used with TX characteristic) */
    uint8_t from_air_buffer[CS_VALUE_MAX_LENGTH];
    uint8_t from_air_cccd_value[2];

    /* To Bluetooth transfer buffer (used with RX characteristic) */
    uint8_t to_air_buffer[CS_VALUE_MAX_LENGTH];
    uint8_t to_air_cccd_value[2];


	/* To BLE WE 1 transfer buffer */
    uint8_t vbat_to_air_buffer[CS_VBAT_MAX_LENGTH];
    uint8_t vbat_to_air_cccd_value[2];

    /* To BLE WE 1 transfer buffer */
    uint8_t we1_to_air_buffer[CS_WE1_MAX_LENGTH];
    uint8_t we1_to_air_cccd_value[2];

    /* To BLE WE 2 transfer buffer */
    uint8_t we2_to_air_buffer[CS_WE2_MAX_LENGTH];
    uint8_t we2_to_air_cccd_value[2];

    /* To BLE RE transfer buffer */
    uint8_t re_to_air_buffer[CS_RE_MAX_LENGTH];
    uint8_t re_to_air_cccd_value[2];
};

enum custom_app_msg_id
{
    CUSTOMSS_NTF_TIMEOUT = TASK_FIRST_MSG(TASK_ID_APP) + 60,
    APP_CUSS_AGMS_REQ_DATE_TIME,
    APP_CUSS_AGMS_AFE_UPDATE,
    APP_CUSS_AGMS_NEXT_SEND_DATA,
    APP_CUSS_AGMS_REQ_BASIC_INFO,
    APP_CUSS_BLE_PARAM_UPDATE,
    APP_CUSS_BLE_POWER_ON
};

/* ----------------------------------------------------------------------------
 * Global variables and types
 * --------------------------------------------------------------------------*/

/* ----------------------------------------------------------------------------
 * Function prototype definitions
 * --------------------------------------------------------------------------*/
/**
 * @brief       Retrieve the custom service attribute database
 * @param[in]   att_db_cs_svc_id
 *                  Customer service ID to be retrieved.
 * @return      const struct att_db_desc*
 *                  The custom service attribute database.
 */
const struct att_db_desc * CUSTOMSS_GetDatabaseDescription(uint8_t att_db_cs_svc_id);

/**
 * @brief       Initialize custom service variables and environment
 */
void CUSTOMSS_Initialize(void);

/**
 * @brief       Notify CEM102 Battery to the central immediately
 */
void CUSTOMSS_Notify_CEM102Bat_Now(void);

void CUSTOMSS_Notify_CEM102_Update(void);

/**
 * @brief       Configure custom service to send periodic notifications
 * @param[in]   timeout    Timeout to be expired in unites of 10ms.
 *                         If set to 0, periodic notifications are disabled.
 */
void CUSTOMSS_NotifyOnTimeout(uint32_t timeout);

/**
 * @brief       Callback handler for all events related to the custom service
 * @param[in]   msg_id    The kernel message ID number.
 * @param[in]   param     The message parameter.
 * @param[in]   dest_id   The destination task ID number.
 * @param[in]   src_id    The source task ID number
 */
void CUSTOMSS_MsgHandler(ke_msg_id_t const msg_id, void const *param,
                         ke_task_id_t const dest_id, ke_task_id_t const src_id);


uint8_t App_BLE_CUSSReadWriteRequestCallback(uint8_t conidx, uint16_t attidx,
    uint16_t handle, uint8_t *dest_buffer, const uint8_t *src_buffer,
    uint16_t length, uint16_t operation, uint8_t hl_status,
    uint8_t *cfm_msg_instr);


void set_agms_base_info(uint8_t base_info);
uint8_t get_agms_base_info(void);

void set_agms_conidx(uint8_t conidx);
uint8_t get_agms_conidx(void);

/* ----------------------------------------------------------------------------
 * Close the 'extern "C"' block
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif    /* ifdef __cplusplus */

#endif    /* BLE_CUSTOMSS_H */
