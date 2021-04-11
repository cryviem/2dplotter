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
#include "plotter.h"

static bool precond_check(void);
static void plotter_work(void);

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
				plotter_work();
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

static void plotter_work(void)
{
	uint8_t* line = NULL;
	cmd_block_t cmd_block;

	while (precond_check())
	{
		line = gcode_rd_buff();
		memset(&cmd_block, 0, sizeof(cmd_block_t));
		if (0 != gcode_parser((char*)line, &cmd_block))
			continue;	/* ignore the below if cmd parse fail*/

		gcode_execute(cmd_block);
	}
}

static bool precond_check(void)
{
	bool ret = true;

	/* check if pending gcode command in buffer */
	if (gcode_get_loaded() == 0)
	{
		ret = false;
	}

	/* add more condition later */
	return ret;
}
