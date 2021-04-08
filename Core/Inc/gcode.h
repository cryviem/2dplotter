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

#define CMD_STATUS_X_BIT			(uint16)0x0001
#define CMD_STATUS_Y_BIT			(uint16)0x0002
#define CMD_STATUS_I_BIT			(uint16)0x0004
#define CMD_STATUS_J_BIT			(uint16)0x0008
#define CMD_STATUS_F_BIT			(uint16)0x0010
#define CMD_STATUS_P_BIT			(uint16)0x0020

#define IS_FLAG_SET(value, flag)	((value & flag) == flag)

typedef enum
{
	CMD_G0,
	CMD_G1,
	CMD_G2,
	CMD_G3,
	CMD_G4,
	CMD_INVALID
}gcode_cmd_en;

typedef enum
{
	STATE_IDLE,
	STATE_EXECUTING
}gcode_state_en;

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
    float			P;
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
} ring_buffer_t;

typedef struct {
    msg_id_en               msgid;
    uint8_t           		payload[4];
} gcode_msg_t;

void gcode_rcv_event_cb(UART_HandleTypeDef *huart, uint16_t Pos);
void gcode_button_press(void);
void gcode_send_empty_msg(msg_id_en msgid);
void gcode_task(void);
#endif /* GCODE_H */
