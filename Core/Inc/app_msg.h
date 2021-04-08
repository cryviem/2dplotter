
#ifndef APP_MSG_H
#define APP_MSG_H

typedef enum {
	FPGA__STATUS_CHANGE_MSG,
	GCODE_UART_RCV_NOTIF,
	GCODE_EXECUTE_CMD,
	NUM_OF_MSG
} msg_id_en;

#endif /* APP_MSG_H */
