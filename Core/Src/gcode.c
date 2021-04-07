/*
 * gcode.c
 *
 *  Created on: Mar 30, 2021
 *      Author: Admin
 */
#include <string.h>
#include "stm32f4xx_hal.h"
#include "gcode.h"
#include "main.h"
#include "app_msg.h"

#define IS_NUMBER(x)					((x >= '0') && (x <= '9'))
#define DECIMAL_DIGIT_LIMIT				4

ring_buffer_t gcode_buff = {0};
const char ack_resp[] = "ok";
cmd_block_t cmd_box = {0};

const cmd_lut_t cmd_lut[CMD_INVALID] =
{
		{"G0",			CMD_G0},
		{"G1",			CMD_G1},
		{"G2",			CMD_G2},
		{"G3",			CMD_G3},
		{"G4",			CMD_G4}
};

const float decimal_factor[(DECIMAL_DIGIT_LIMIT + 1)] = {1, 0.1, 0.01, 0.001, 0.0001};

void gcode_receive(void);
uint8_t split_line(char** list, uint8_t max, char* line, const char *delimeter);
gcode_cmd_en search_cmd(char * str);
int8_t param_extract(char* letter, float* fval, char* str);
int8_t gcode_parser(char *line, cmd_block_t* cmd_block);

/* split line to string by delimeter */
uint8_t split_line(char** list, uint8_t max, char* line, const char *delimeter)
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
gcode_cmd_en search_cmd(char * str)
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
int8_t param_extract(char* letter, float* fval, char* str)
{
	uint16_t pointer = 0;
	int8_t	sign_value = 1;
	uint8_t decimal_cnt = 0;
	uint8_t is_decimal = 0;
	uint8_t num;
	char ch;
	uint32_t lu32val = 0;
	float lfval = 0;

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

	lfval = lu32val;
	/* decimal apply */
	lfval *= decimal_factor[decimal_cnt];
	/* sign apply */
	*fval = sign_value*lfval;
	return 0;
}

int8_t gcode_parser(char *line, cmd_block_t* cmd_block)
{
    char* cmd_arg[CMD_MAX_ITEM];
    uint8_t cmd_arg_count = 0;
    uint8_t i;
    char letter = 0;
    char delimeter = ' ';
    float fval = 0;
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
    	ret = param_extract(&letter, &fval, cmd_arg[i]);
    	if (ret != 0)
    	{
    		return -1;
    	}

    	switch (letter)
    	{
    	case 'X':
    		cmd_block->X = fval;
    		break;

    	case 'Y':
    		cmd_block->Y = fval;
    		break;

    	case 'I':
    		cmd_block->I = fval;
    		break;

    	case 'J':
    		cmd_block->J = fval;
    		break;

    	case 'F':
    		cmd_block->F = fval;
    		break;

    	case 'P':
    		cmd_block->P = fval;
    		break;
    	}
    }

    return 0;
}


void gcode_rcv_event_cb(UART_HandleTypeDef *huart, uint16_t Pos)
{
	msg_t msg = {0};

	gcode_buff.item[gcode_buff.wptr].actsize = Pos;
	/* notify gcode task */
	msg.msgid = GCODE_UART_RCV_NOTIF;
	osMessageQueuePut(uart_queueHandle, &msg, 0, 0);
}

void gcode_button_press(void)
{

}

void gcode_task(void)
{
	msg_t msg = {0};
	osStatus_t ret;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)gcode_buff.item[gcode_buff.wptr].data, GCODE_MAX_ITEM_SIZE);

	while(1)
	{
		ret = osMessageQueueGet(uart_queueHandle, &msg, (void*)0, 5000 );
		if (osOK != ret)
		{
			/**/
		}
		else
		{
			switch (msg.msgid)
			{
			case GCODE_UART_RCV_NOTIF:
				/* add '\0' to make a complete string */
				gcode_buff.item[gcode_buff.wptr].data[gcode_buff.item[gcode_buff.wptr].actsize] = 0;
				gcode_buff.load_cnt++;
				gcode_buff.wptr = (gcode_buff.wptr + 1) % GCODE_MAX_BUFF_ITEM;
				gcode_receive();

				while (gcode_buff.load_cnt > 0)
				{
					gcode_parser((char*)gcode_buff.item[gcode_buff.rptr].data, &cmd_box);
					gcode_buff.load_cnt--;
					gcode_buff.rptr = (gcode_buff.rptr + 1) % GCODE_MAX_BUFF_ITEM;
				}

				break;

			default:
				break;
			}

			/*state machine execute */
		}

	}
}

void gcode_receive(void)
{
	if (gcode_buff.load_cnt < GCODE_MAX_BUFF_ITEM)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)gcode_buff.item[gcode_buff.wptr].data, GCODE_MAX_ITEM_SIZE);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)ack_resp, 3);
	}
}
