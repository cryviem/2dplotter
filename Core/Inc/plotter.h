/*
 * plotter.h
 *
 *  Created on: Apr 9, 2021
 *      Author: hanguyen
 */

#ifndef PLOTTER_H
#define PLOTTER_H

#include "fpga.h"

typedef enum {
	FPGA_STATUS_CHANGE_MSG,
	FPGA_SEND_CMPLT_MSG,
	GCODE_CMD_RCV_NOTIF_MSG,
	NUM_OF_MSG
} msg_id_en;

typedef union {
	fpga_status_t			fpga_sts;
	uint8_t           		rawdara[4];
}msg_data_t;

typedef struct {
    msg_id_en               msgid;
    msg_data_t           	payload;
} msg_t;

void plotter_main(void);

#endif /* PLOTTER_H */
