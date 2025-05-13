#ifndef DEF_H_
#define DEF_H_

#ifndef __cplusplus

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>



#pragma region F405_DEFINES
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

/***************************************
 - pin configuration
***************************************/

/***************************************
 - version update
***************************************/
#define F_VER_V21 0 // old source remark
#define F_VER_V22 1 // update, 191119 , for multichannel fucntion
#define F_VER_V26 6 // update, 201214 , only single channel run modifying..

/***************************************
 - MODEL TYPE
***************************************/
#define MODEL_TYPE TYPE_DOUBLE
#define TYPE_SINGLE 1
#define TYPE_DOUBLE 2

#define EPS_TYPE TYPE_NEW_EPS
#define TYPE_BAS_EPS 1
#define TYPE_NEW_EPS 2

/***************************************
 - CONSTANT
***************************************/
#define X_AXIS 1
#define Y_AXIS 2

/***************************************
 - communication
***************************************/

#define STX 0x02
#define STX_IDX 0
#define CMD_DIMMING_READY 0xA7
/*#define CMD_STATE_REQ  			0xA9	*/
#define CMD_STATE_REQ_PC 0xB9
#define CMD_IDX 1
#define ETX 0x03

// EPS Control
#define CMD_POWER_ON 0xA0
#define CMD_POWER_OFF 0xAD
#define CMD_STATE_REQ 0xA9
#define CMD_DIMMING_50 0x32
#define CMD_DIMMING_100 0x64

// EPS State
#define STATE_OFF 0x30
#define STATE_ON 0x31
#define STATE_STABLE 0x32
#define STATE_NORMAL 0x33
#define STATE_OVER_CURRENT 0x35
#define STATE_OVER_TEMP 0x36
#define STATE_50 0x38
#define STATE_100 0x6A

#define BUFFER_SIZE 1024

// Command from PC
#define CMD_STEP_CONTROL 0xA7
/*#define CMD_LUT_WRITE   		0xAB	*/
#define CMD_LUT_WRITE 0xBB
#define CMD_EPS_INDIV_CONTROL 0xAD
#define CMD_MOTOR_INDIV_CONTROL 0xAF
#define CMD_TOTAL_INDIV_CONTROL 0xBD

// Motor Positon
#define MOT_POS_INIT 60 // Motor position initialize point
#define MOT_POS_MIN 0
#define MOT_POS_MAX 80

#define NONE_DATA 99

extern uint8_t DebugData[50];
extern uint8_t DebugCnt;

#pragma endregion

#endif

#endif
