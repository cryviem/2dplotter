/*
 * plotter.c
 *
 *  Created on: Apr 9, 2021
 *      Author: hanguyen
 */
#include "app_common.h"
#include <string.h>
#include "main.h"
#include "planner.h"
#include "gcode.h"

void plotter_main(void)
{
	msg_t msg = {0};
	osStatus_t ret;

	/* start receiving gcode command */
	gcode_receive();

	while(1)
	{
		ret = osMessageQueueGet(plotter_queueHandle, &msg, (void*)0, 5000 );
		if (osOK != ret)
		{
			/**/
		}
		else
		{
			switch (msg.msgid)
			{
			case GCODE_CMD_RCV_NOTIF_MSG:
				gcode_wr_buff_cmplt();

				if(true == gcode_receive())
					gcode_send_ok();

				// call command execute

				break;

			case GCODE_CMD_EXECUTE_MSG:
				memset(&cmd_box, 0, sizeof(cmd_block_t));
				cmd_parser((char*)gcode_buff.item[gcode_buff.rptr].data, &cmd_box);
				cmd_execute(&cmd_box);
				gcode_buff.load_cnt--;
				gcode_buff.rptr = (gcode_buff.rptr + 1) % GCODE_MAX_BUFF_ITEM;
				if (true == able_to_work())
				{
					gcode_send_empty_msg(GCODE_EXECUTE_CMD);
				}
				else
				{
					gcode_state = STATE_IDLE;
				}
				break;

			case FPGA_STATUS_CHANGE_MSG:
				break;

			case FPGA_SEND_CMPLT_MSG:
				break;
			default:
				break;
			}

			/*state machine execute */
		}

	}
}

