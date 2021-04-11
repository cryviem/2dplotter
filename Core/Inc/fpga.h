
#ifndef FPGA_H
#define FPGA_H
#include "app_common.h"
typedef enum {
	FPGA_STATUS_READY,
	FPGA_STATUS_BUSY
} fpga_status_t;

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi);
bool fpga_send(uint16_t* pdata, uint16_t len);

#endif /* FPGA_H */
