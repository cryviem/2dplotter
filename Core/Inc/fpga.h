
#ifndef FPGA_H
#define FPGA_H
#include "app_common.h"
#include "planner.h"

#define SIZE_OF_FPGA_PACKET			12
#define SIZE_OF_FPGA_BUFFER			20
#define FPGA_BUFFER_WR_THRESHOLD	16

typedef enum {
	FPGA_STATUS_READY,
	FPGA_STATUS_BUSY
} fpga_status_t;

typedef union {
	pl_block_t		pl_data;
	uint16_t		rawdata[SIZE_OF_FPGA_PACKET];
} fpga_packet_t;

typedef struct {
	fpga_packet_t 									packet[SIZE_OF_FPGA_BUFFER];
	uint8_t 										wptr;
	uint8_t 										rptr;
	uint8_t											load_cnt;
} fpga_buffer_t;

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi);
bool fpga_send_ready(void);
bool fpga_send(void);
bool fpga_wr_ready(void);
pl_block_t* fpga_wr_buff_start_pl(void);
void fpga_wr_buff_cmplt(void);
void fpga_rd_buff_cmplt(void);

#endif /* FPGA_H */
