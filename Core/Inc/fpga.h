
typedef enum {
	FPGA_STATUS_READY,
	FPGA_STATUS_BUSY
} fpga_status_t;

void fpga_sendcplt_cb(SPI_HandleTypeDef *hspi);
void fpga_button_pressed(void);
void fpga_transmit_task(void);
