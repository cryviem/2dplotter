/*
 * gcode.c
 *
 *  Created on: Mar 30, 2021
 *      Author: Admin
 */
#include "app_common.h"
#include <string.h>
#include "gcode.h"
#include "main.h"
#include "planner.h"
#include "plotter.h"

#define IS_NUMBER(x)					((x >= '0') && (x <= '9'))
#define DECIMAL_DIGIT_LIMIT				4


const char ack_resp[] = "ok";
const cmd_lut_t cmd_lut[CMD_INVALID] =
{
		{"G0",			CMD_G0},
		{"G1",			CMD_G1},
		{"G2",			CMD_G2},
		{"G3",			CMD_G3},
		{"G4",			CMD_G4}
};
const uint32_t decimal_factor[(DECIMAL_DIGIT_LIMIT + 1)] = {1, 10, 100, 1000, 10000};

gcode_buffer_t gcode_buff = {0};
cmd_block_t cmd_box = {0};
gcode_state_en gcode_state = STATE_IDLE;



static uint8_t split_line(char** list, uint8_t max, char* line, const char *delimeter);
static gcode_cmd_en search_cmd(char * str);
static int8_t param_extract(char* letter, int32_t* i32val, char* str);



/* split line to string by delimeter */
static uint8_t split_line(char** list, uint8_t max, char* line, const char *delimeter)
{
    uint8_t count = 0;
    char* rec = NULL;

    if ((NULL != list) && (max > 0) && (NULL != line) && (NULL != delimeter))
    {
    	list[count]= strtok_r (line, delimeter, &rec);
        while ((NULL != list[count]) && ((++count) < max))
        {
        	list[count]= strtok_r (NULL, delimeter, &rec);
        }
    }
    return (count);
}

/* search command from string */
static gcode_cmd_en search_cmd(char * str)
{
	gcode_cmd_en ret = CMD_INVALID;
	uint8_t i;

	for (i = 0; i < CMD_INVALID; i++)
	{
		if (0==strcmp(cmd_lut[i].string, (char*)str))
		{
			ret = cmd_lut[i].cmdid;
			break;
		}
	}

	return ret;
}

/* extract letter and float values from string */
static int8_t param_extract(char* letter, int32_t* i32val, char* str)
{
	uint16_t pointer = 0;
	int32_t	sign_value = 1;
	uint8_t decimal_cnt = 0;
	uint8_t is_decimal = 0;
	uint8_t num;
	char ch;
	uint32_t lu32val = 0;

	/* read first letter */
	ch = str[pointer];
	if ((ch < 'A') || (ch > 'Z'))
	{
		*letter = 0;
		return -1;
	}
	*letter = ch;
	pointer++;

	/* read trail float value */
	/* read sign character */
	ch = str[pointer];
	if (ch == '-')
	{
		sign_value = -1;
		pointer++;
	}
	else if (ch == '+')
	{
		pointer++;
	}

	/* read and convert number */
	while (str[pointer] != '\0')
	{
		ch = str[pointer];
		pointer++;

		if (IS_NUMBER(ch))
		{
			num = ch - '0';
			lu32val = (((lu32val << 2) + lu32val) << 1) + num;	//lu32val*10 + num

			if (is_decimal == 1)
				decimal_cnt++;

			if (decimal_cnt >= DECIMAL_DIGIT_LIMIT)
				break;
		}
		else if (ch == '.')
		{
			/* decimal point */
			is_decimal = 1;
		}
		else
		{
			/* stop on abnormal character */
			break;
		}
	}

	lu32val = ((lu32val * MM_TO_PULSE) / decimal_factor[decimal_cnt]);

	/* sign apply */
	*i32val = (int32_t)lu32val * sign_value;
	return 0;
}

int8_t gcode_parser(char *line, cmd_block_t* cmd_block)
{
    char* cmd_arg[CMD_MAX_ITEM];
    uint8_t cmd_arg_count = 0;
    uint8_t i;
    char letter = 0;
    char delimeter = ' ';
    int32_t i32val = 0;
    int8_t ret;

    cmd_arg_count = split_line(cmd_arg, CMD_MAX_ITEM, line, &delimeter);

    if (cmd_arg_count == 0)
    {
    	return -1;
    }

    cmd_block->cmdid = search_cmd(cmd_arg[0]);

    if (cmd_block->cmdid == CMD_INVALID)
    {
    	return -1;
    }

    for (i = 1; i < cmd_arg_count; i++)
    {
    	ret = param_extract(&letter, &i32val, cmd_arg[i]);
    	if (ret != 0)
    	{
    		return -1;
    	}

    	switch (letter)
    	{
    	case 'X':
    		cmd_block->X = (int16_t)i32val;
    		cmd_block->flag |= CMD_STATUS_X_BIT;
    		break;

    	case 'Y':
    		cmd_block->Y = (int16_t)i32val;
    		cmd_block->flag |= CMD_STATUS_Y_BIT;
    		break;

    	case 'I':
    		cmd_block->I = (int16_t)i32val;
    		cmd_block->flag |= CMD_STATUS_I_BIT;
    		break;

    	case 'J':
    		cmd_block->J = (int16_t)i32val;
    		cmd_block->flag |= CMD_STATUS_J_BIT;
    		break;

    	case 'F':
    		cmd_block->F = (uint16_t)(i32val / 60); /* i32val is pulse/min, need convert to pulse/s */
    		cmd_block->flag |= CMD_STATUS_F_BIT;
    		break;

    	case 'P':
    		cmd_block->P = (uint16_t)i32val;
    		cmd_block->flag |= CMD_STATUS_P_BIT;
    		break;
    	}
    }

    return 0;
}

bool gcode_receive(void)
{
	bool ret = false;
	if (gcode_buff.load_cnt < GCODE_MAX_BUFF_ITEM)
	{
		ret = (HAL_OK == HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)gcode_buff.item[gcode_buff.wptr].data, GCODE_MAX_ITEM_SIZE));
	}
	return ret;
}

void gcode_send_ok(void)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ack_resp, 3);
}

void gcode_rcv_event_cb(UART_HandleTypeDef *huart, uint16_t Pos)
{
	msg_t msg = {0};
	gcode_buff.item[gcode_buff.wptr].actsize = Pos;
	msg.msgid = GCODE_CMD_RCV_NOTIF_MSG;
	osMessageQueuePut(plotter_queueHandle, &msg,0,0);
}

void gcode_wr_buff_cmplt(void)
{
	/* add '\0' to make a complete string */
	gcode_buff.item[gcode_buff.wptr].data[gcode_buff.item[gcode_buff.wptr].actsize] = 0;
	gcode_buff.load_cnt++;
	gcode_buff.wptr = (gcode_buff.wptr + 1) % GCODE_MAX_BUFF_ITEM;
}

uint8_t* gcode_rd_buff(void)
{
	uint8_t* ret = NULL;

	if (gcode_buff.load_cnt > 0)
	{
		ret = gcode_buff.item[gcode_buff.rptr].data;
		gcode_buff.load_cnt--;
		gcode_buff.rptr = (gcode_buff.rptr + 1) % GCODE_MAX_BUFF_ITEM;
	}

	return ret;
}

uint8_t gcode_get_loaded(void)
{
	return gcode_buff.load_cnt;
}

void gcode_execute(cmd_block_t cmd_block)
{
	pos_t target_pos = {0, 0};
	pos_t center_pos = {0, 0};
	bool is_rapid = false;
	bool is_ccw = false;
	bool is_valid = false;

	/* feedrate update */
	if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_F_BIT))
	{
		pl_updatespeed(cmd_block.F);
	}

	switch (cmd_block.cmdid)
	{
	case CMD_G0:
		is_rapid = true;
	case CMD_G1:
		/* for G0 or G1, at least X or Y has to be available */
		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_X_BIT))
		{
			target_pos.x = pl_calc_dx(cmd_block.X);
			is_valid = true;
		}

		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_Y_BIT))
		{
			target_pos.y = pl_calc_dy(cmd_block.Y);
			is_valid = true;
		}

		if (true == is_valid)
		{
			pl_line(target_pos, is_rapid);
		}
		break;

	case CMD_G3:
		is_ccw = true;
	case CMD_G2:
		/* for G2 or G3, at least I or J has to be available */
		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_X_BIT))
		{
			target_pos.x = pl_calc_dx(cmd_block.X);
		}

		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_Y_BIT))
		{
			target_pos.y = pl_calc_dy(cmd_block.Y);
		}

		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_I_BIT))
		{
			center_pos.x = cmd_block.I;
			is_valid = true;
		}

		if (IS_FLAG_SET(cmd_block.flag, CMD_STATUS_J_BIT))
		{
			center_pos.y = cmd_block.J;
			is_valid = true;
		}

		if (true == is_valid)
		{
			pl_arc(target_pos, center_pos, is_ccw);
		}
		break;
	default:
		break;
	}
}
