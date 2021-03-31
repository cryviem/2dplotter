/*
 * gcode.h
 *
 *  Created on: Mar 30, 2021
 *      Author: Admin
 */

#ifndef INC_GCODE_H_
#define INC_GCODE_H_

#define CMD_MAX_ITEM				5
#define GCODE_MAX_BUFFER_SIZE		100

typedef enum
{
	CMD_G0,
	CMD_G1,
	CMD_G2,
	CMD_G3,
	CMD_G4,
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
    float			P;

}cmd_block_t;

void gcode_rcv_cplt_cb(UART_HandleTypeDef *huart);
void gcode_rcv_event_cb(UART_HandleTypeDef *huart, uint16_t Pos);
void gcode_button_press(void);
#endif /* INC_GCODE_H_ */
