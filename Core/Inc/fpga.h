
#ifndef FPGA_H
#define FPGA_H

#include "app_msg.h"

typedef enum {
	FPGA_STATUS_READY,
	FPGA_STATUS_BUSY
} fpga_status_t;

typedef struct {
    msg_id_en               msgid;
    uint8_t           		payload[4];
} fpga_msg_t;

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi);
void fpga_button_pressed(void);
void fpga_transmit_task(void);

#endif /* FPGA_H */
