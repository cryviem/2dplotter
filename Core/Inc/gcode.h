/*
 * gcode.h
 *
 *  Created on: Mar 30, 2021
 *      Author: Admin
 */

#ifndef GCODE_H
#define GCODE_H

#define CMD_MAX_ITEM				5
#define GCODE_MAX_ITEM_SIZE			100
#define GCODE_MAX_BUFF_ITEM			4

#define CMD_STATUS_X_BIT			0x0001
#define CMD_STATUS_Y_BIT			0x0002
#define CMD_STATUS_I_BIT			0x0004
#define CMD_STATUS_J_BIT			0x0008
#define CMD_STATUS_F_BIT			0x0010
#define CMD_STATUS_P_BIT			0x0020

#define IS_FLAG_SET(value, flag)	((value & flag) == flag)

typedef enum
{
	CMD_G0,
	CMD_G1,
	CMD_G2,
	CMD_G3,
	CMD_G20,
	CMD_G21,
	CMD_G90,
	CMD_G91,
	CMD_M03,
	CMD_M05,
	CMD_M17,
	CMD_M18,
	CMD_D0,
	CMD_D1,
	CMD_INVALID
}gcode_cmd_en;

typedef struct{
    char*      		string;
    gcode_cmd_en 	cmdid;
}cmd_lut_t;

typedef struct{
    gcode_cmd_en 	cmdid;
    float			X;
    float			Y;
    float			I;
    float			J;
    float			F;
    uint16_t		flag;
}cmd_block_t;

typedef struct {
	uint8_t											data[GCODE_MAX_ITEM_SIZE];
	uint16_t 										actsize;
} item_t;

typedef struct {
	item_t 											item[GCODE_MAX_BUFF_ITEM];
	uint8_t 										wptr;
	uint8_t 										rptr;
	uint8_t											load_cnt;
} gcode_buffer_t;

bool gcode_receive(void);
void gcode_send_ok(void);
void gcode_wr_buff_cmplt(void);
uint8_t gcode_get_loaded(void);
uint8_t* gcode_rd_buff(void);
void gcode_rcv_event_cb(UART_HandleTypeDef *huart, uint16_t Pos);
int8_t gcode_parser(char *line, cmd_block_t* cmd_block);
void gcode_execute(cmd_block_t cmd_block);
#endif /* GCODE_H */
