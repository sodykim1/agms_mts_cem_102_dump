CEM102 Demo - Simple Firmware Demonstration Application
=======================================================

NOTE: If you use this sample application for your own purposes, follow the
      licensing agreement specified in `Software_Use_Agreement.rtf` in the
      home directory of the installed Software Development Kit (SDK).

Overview
--------
This sample project provides a demonstration of how to interface with a CEM102
device from an RSL15 with Sleep Mode. This sample demonstrates the following 
functionality:

 1. Going into and out of NO RETENTION Sleep Mode with GPIO0 as a switch
 
 2. Setting up the connections to a CEM102 device

 3. Initialization of the CEM102 library and device using the standard API

 4. Performing a continuous measurement using the CEM102 device
 
 5. Sending notifications: the application sends periodic notification of 
    RSL15 battery level, CEM102 battery level, and Working Electrode measured by CEM102, 
    to the phone. 

Upon boot-up, RSL15 initializes and starts the Bluetooth Low Energy task, and powers up CEM102. 
CEM102 initializes, then starts calibration. During calibration, the reference current is 
measured. Once this is complete, the CEM102 channel is configured and starts a calibration measurement. 
As a last step, channel gain is calculated. Once calibration is finished, CEM102 continuously 
measures the Working Electorode.

The calculation of channel gain is implemented using two simple state machines.
Two variables, `measure_status` and `calib_state`, are used for implementing these
state machines. At the top level, the application begins in calibration state. Once 
calibration is finished, the application changes to regular operation state. During calibration stages, 
these two state machines are moved in tandem while taking different measurements.
After calibration is finished, the `calib_state` state machine is used to move the device 
out of calibration state into regular operation state.

The state machine starts with `measure_status` and `calib_state` being set to
`MEASURE_IDLE` and `CALIB_INITIALIZED`, respectively. Once OTP and Trim values are
read, measure_status is moved to `MEASURE_INITIALIZED`. After this, the application
performs three different current measurements. These are readings of the reference
current, and of the current for each of the channels WE1 and WE2. These measurements are
used to calculate the channel gain for each channel.

For each of the three measurements, the state machine for `measure_status` is
set to `MEASURE_MEASURING` by the application. The SPI callback function moves this
state machine to `MEASURE_READING_RESULTS`, to indicate the availability of the
data from CEM102. The `calib_state` state machine contain three different pairs
of states to track the three different measurements. State transition from
`CALIB_REF_MEASURING` to `CALIB_REF_READING` is used to measure the reference
current. Transition from `CALIB_CH1_MEASURING` to `CALIB_CH1_READING` is used to
measure the current for the channel 1 (WE1). Transition from `CALIB_CH2_MEASURING`
to `CALIB_CH2_READING` is used to measure the current for the channel 2 (WE2).
During the three different measurements, `calib_state` is moved through
`CALIB_REF_READING`, `CALIB_CH1_READING` and `CALIB_CH2_READING` by the SPI
callback function.

After the three measurements have been completed, `calib_state` is set to `CALIB_DONE`.
This state change puts the application into regular operation state.
At this stage, `measure_status` is moved to back to `MEASURE_INITIALIZED` state.
 
The RSL15 device's Sleep Mode is supported by the Bluetooth Low Energy
library and the system library. In each loop of the main application, CEM102's operation, 
RTC Scheduler tasks, and the Bluetooth Low Energy activity's tasks are performed 
if necessary. Once the application can switch to Sleep Mode, Bluetooth Low Energy configurations 
and states are saved and the system is put into Sleep Mode; this demonstrates memory retention. 
The system is then woken up by the Bluetooth Low Energy baseband timer or RTC timer, 
or by an interrupt from CEM102. On wakeup, configurations and states are restored. 
The Bluetooth Low Energy connection with the central device, established before going into 
Sleep Mode, is resumed, and normal operations of the application are continued. When wakeup 
is caused by an RTC timeout, the system runs any tasks that are in READY state. After calculating 
the next sleep duration, the application programs the RTC timer and goes back to sleep until 
the next RTC timeout.

The scheduler in the sample code is presented using two tasks, as follows: 
- The first task starts the measurement of RSL15 VBAT with LSAD every two seconds. 
While measuremnent remains unfinished, the system is not allowed to go to sleep.
After 'LSAD_READS_NUM' measurement, the average is calculated. If it is different from 
the previous value, the notification is sent to the central.
- The second task reports the RSL15 VBAT level to the central every 20 seconds.

The tasks have 3 different states: READY, BLOCKED, and SUSPENDED. By default, when a task is created,
it is set to BLOCKED. In this state, the task waits for the RTC timer to expire. When the RTC timer 
is above the threshold, an interrupt is generated, the system wakes up, and the scheduler updates
all BLOCKED tasks' cycle counts in the queue. If the counter has matured, the tasks' state is
set to READY. The scheduler runs the READY tasks and changes them back to BLOCKED. 
If a task is set to SUSPENDED, it is ignored and its cycle count is not updated.
The tasks' states and arrival cycles can be changed with the functions `Scheduler_Set_TaskState()` 
and `Scheduler_Set_ArrivalCycle()`, respectively.

For more details and a figure, see the CEM102 Software Reference.

The application allows you to configure the Bluetooth Low Energy address type (public or private) and 
the location from which to read the Bluetooth Low Energy address. 
*   By default, the public address type is selected (`GAPM_ADDRESS_TYPE` is configured as 
        `GAPM_CFG_ADDR_PUBLIC` in `app.h`).
*   When the public address type is selected, the application reads the public address from a 
        location defined by `APP_BLE_PUBLIC_ADDR_LOC` in `app.h`. By default, `APP_BLE_PUBLIC_ADDR_LOC` 
        is configured to `BLE_PUBLIC_ADDR_LOC_MNVR`, which is a location in MNVR. However, any other 
        valid locations can be used as needed.
*   The Bluetooth Low Energy address type can be re-configured to private by changing `GAPM_ADDRESS_TYPE`
        from `GAPM_CFG_ADDR_PUBLIC` to `GAPM_CFG_ADDR_PRIVATE` in `app.h`.

The basic sequence of operations and event messages exchanged between the 
application and the Bluetooth stack is presented below: 

  APP <--> Bluetooth Low Energy Stack
  
  Startup

      step 1 --->  GAPM_ResetCmd() - app.c
      step 2 <---  GAPM_CMP_EVT / GAPM_RESET
      --->  GAPM_SetDevConfigCmd() - app_msg_handler.c
      step 3 <---  GAPM_CMP_EVT / GAPM_SET_DEV_CONFIG
      --->  BASS_ProfileTaskAddCmd() - app_msg_handler.c
      step 4 <---  GAPM_PROFILE_ADDED_IND / TASK_ID_BASS
      ----> GATTM_AddAttributeDatabase() / CUST_SVC0 - app_msg_handler.c
      step 5 <---  GATTM_ADD_SVC_RSP / CUST_SVC0 
      --->  GAPM_ActivityCreateAdvCmd() - app_msg_handler.c
      step 6 <---  GAPM_ACTIVITY_CREATED_IND
      --->  GAPM_SetAdvDataCmd() - app_msg_handler.c
      step 7 <---  GAPM_CMP_EVT / GAPM_SET_ADV_DATA 
      --->  GAPM_AdvActivityStart() - app_msg_handler.c

  Connection request / parameters update request / device info request
  
      step 8 <--- GAPC_CONNECTION_REQ_IND
      ---> GAPM_ResolvAddrCmd() - app_msg_handler.c
      ---> GAPC_ConnectionCfm() - app_msg_handler.c
      <--- GAPC_PARAM_UPDATE_REQ_IND
      ---> GAPC_ParamUpdateCfm() - app_msg_handler.c
      <--- GAPC_GET_DEV_INFO_REQ_IND
      ---> GAPC_GetDevInfoCfm() - app_msg_handler.c

  Pairing / Bonding request
  
      <--- GAPC_BOND_REQ_IND / GAPC_PAIRING_REQ
      ----> GAPC_BondCfm() - app_msg_handler.c
      <--- GAPC_BOND_REQ_IND / GAPC_LTK_EXCH
      ----> GAPC_BondCfm() - app_msg_handler.c
      <--- GAPC_BOND_REQ_IND / GAPC_IRK_EXCH
      ----> GAPC_BondCfm() - app_msg_handler.c
      <--- GAPC_BOND_REQ_IND / GAPC_CSRK_EXCH
      ----> GAPC_BondCfm() - app_msg_handler.c
      <--- GAPC_BOND_IND 
      ----> 

  Encrypt request
  
      <--- GAPC_ENCRYPT_REQ_IND
      ---> GAPC_EncryptCfm() - app_msg_handler.c
      <--- GAPC_ENCRYPT_IND 
      ---->

  Disconnection
  
      <--- GAPC_DISCONNECT_IND
      ---> GAPM_StartAdvertiseCmd() - app_msg_handler.c


The application has a NO_RETENTION_SLEEP button, defined as `GPIO_NO_RET_SLEEP_PIN`. 
By default, when the system boots up from Power-On-Reset, it enters RUN mode. 
When the NO_RETENTION_SLEEP button is pressed, it goes to NO_RETENTION_SLEEP, 
halting everything. When the button is pressed again, it wakes up from 
NO_RETENTION_SLEEP mode and the board wakes up in a reset state, starting 
over the initialization described earlier.

Hardware Requirements
---------------------
This application can be executed on a CEM102 Evaluation and Development Board,
or can be modified to operate on an RSL15 Evaluation and Development Board that 
is paired with a CEM102 device.

Importing a Project
-------------------
To import the sample code into your IDE workspace, refer to the 
*Getting Started Guide* for your IDE for more information.

Verification
------------
To verify that this application is functioning correctly, use a central
device such as the RSL15 Central Android/iOS App 
(https://play.google.com/store/apps/details?id=com.onsemi.central)
(https://apps.apple.com/us/app/rsl15-central/id1562731670?platform=iphone) 
or BLE Explorer (https://www.onsemi.com/rsl15) to scan and establish a 
connection with this peripheral device. 

This application uses the following GPIO pins:
- `DEBUG_SLEEP_GPIO` - To activate the SYSCLK and POWER MODE GPIO, set it to 
                       `DEBUG_SYSCLK_PWR_MODE`. NOTE: In `DEBUG_SYSCLK_PWR_MODE`,
                       jumpers 9-10 and 11-12 (GPIO7-TXD & GPIO8-RXD) on J31 must be removed
                       if you are using EVB 1.4.

- `SYSCLK_GPIO` - Outputs the system clock 
- `POWER_MODE_GPIO` - Indicates Power Mode     

To verify that this application is functioning correctly, trace all signals
as follows:
- Run Mode:
    - `SYSCLK_GPIO`: system clock is 3 or 8 MHz.
    - `POWER_MODE_GPIO`: low     
- Power Mode: 
    - `SYSCLK_GPIO`: off for the whole sleep duration
    - `POWER_MODE_GPIO`: high
    
The system keeps cycling through Run and Power Modes. The timing of this Run-Power
cycles depends on the configured duration of the advertising interval and the 
connection interval.
- The advertising interval is defined by `APP_ADV_INT_MIN` and `APP_ADV_INT_MAX` in `app.h`. 
By default, the advertising interval is set to 40 ms. 
- The connection interval update request is sent from this application to 
the central device; connection interval value is 375 ms, with a slave latency of 6.
It is up to the central device to accept the request. 

To verify the RTC Scheduler, this application uses the following GPIO Pins:
- `DEBUG_SLEEP_GPIO` - To activate the SYSCLK and POWER MODE GPIO, set it to 
                       `DEBUG_RTC_TASKS_MODE`.
- `BASS_RTC_TASK_GPIO` - Outputs the `APP_BASS_Measure_Task` activity

of the RTC Scheduler triggers every 2 seconds. 

To use swmTrace functionality, `DEBUG_SLEEP_GPIO` should be configured to 
`DEBUG_SWMTRACE_MODE`. When this mode is used, the application cannot go to sleep 
while swmTrace is printing. This mode should be only used when debugging is necessary.

Bluetooth Low Energy Abstraction
--------------------------------
This application takes advantage of the Bluetooth Low Energy Abstraction layer on top of 
the Bluetooth Low Energy stack. This provides an extra layer of simplicity and 
modularization to your firmware design, so that you can focus on your specific application.

Debug Catch Mode 
---------------- 
If the device goes into Low Power Mode or resets quickly, it might be difficult to connect 
the device to a debugger. You can hold the `DEBUG_CATCH_GPIO` low to enter Debug Catch Mode, 
which holds the program execution in a `while` loop to make it easier to connect to the debugger. 

To make use of Debug Catch Mode: 

1. Connect the `DEBUG_CATCH_GPIO` to ground (press and hold the SW1 pushbutton on the 
   evaluation board). 
2. Press the RESET button (this restarts the application, then enters Debug Catch Mode, 
   which holds program execution in a `while` loop). 
3. Connect the debugger so that you can re-program or erase the flash. Disconnect `DEBUG_CATCH_GPIO` 
   from ground (releasing SW1 pushbutton on the evaluation board).

***
  Copyright (c) 2023 Semiconductor Components Industries, LLC (d/b/a
  onsemi), All Rights Reserved
