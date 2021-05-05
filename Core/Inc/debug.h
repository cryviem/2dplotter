/*
 * debug.h
 *
 *  Created on: Apr 25, 2021
 *      Author: Admin
 */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

typedef enum {
	DB_NO_ERROR,
	DB_STATE_NOT_READY,
	DB_TOO_SMALL_BLOCK,
	DB_NO_FPGA_SLOT,
	DB_GCODE_DROPPED,
	DB_NUMOFERR
} db_errorcode_en;

void dbg_D0(void);
void dbg_D1(void);
void error_report(db_errorcode_en err);

#endif /* INC_DEBUG_H_ */
