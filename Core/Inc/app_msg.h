
#define MSG_MAX_SIZE		24

typedef enum {
	FPGA__STATUS_CHANGE_MSG,
	GCODE_UART_RCV_NOTIF,
	GCODE_EXECUTE_CMD,
	NUM_OF_MSG
} msg_id_en;

typedef enum {
	FPGA_ILDE,
	FPGA_BUSY
} fpga_status_en;

typedef union {
	fpga_status_en			fpga_sts;
	uint8_t					rawdata[MSG_MAX_SIZE];
} os_msg_data_t;

typedef struct {
    msg_id_en               msgid;
    os_msg_data_t           payload;
} msg_t;
