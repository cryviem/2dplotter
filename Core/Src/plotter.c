/*
 * plotter.c
 *
 *  Created on: Apr 9, 2021
 *      Author: hanguyen
 */
#include "app_common.h"
#include <string.h>
#include "planner.h"
#include "gcode.h"
#include "plotter.h"
#include "debug.h"

static bool precond_check(void);
static void plotter_work(void);

void plotter_main(void)
{
	msg_t msg = {0};
	osStatus_t ret;
	bool plot_should_work = false;
	bool gcode_buff_full = false;
	pl_init();
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
			plot_should_work = false;

			switch (msg.msgid)
			{
			case GCODE_CMD_RCV_NOTIF_MSG:
				gcode_wr_buff_cmplt();

				if(true == gcode_receive())
					gcode_send_ok();
				else
					gcode_buff_full = true;
				/* stucked: fix needed*/
				// call command execute
				plot_should_work = true;
				break;

			case FPGA_STATUS_CHANGE_MSG:
				if (true == fpga_send_ready())
					fpga_send();
				break;

			case FPGA_SEND_CMPLT_MSG:
				fpga_rd_buff_cmplt();

				if (true == fpga_send_ready())
					fpga_send();

				plot_should_work = true;
				break;
			default:
				break;
			}

			if (true == plot_should_work)
			{
				plotter_work();
				if (true == fpga_send_ready())
					fpga_send();

				if (true == gcode_buff_full)
				{
					if(true == gcode_receive())
					{
						gcode_send_ok();
						gcode_buff_full = false;
					}
				}
			}
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
		{
			error_report(DB_GCODE_DROPPED);
			continue;	/* ignore the below if cmd parse fail */
		}

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

	/* check if able to add more item to fpga buffer */
	if (false == fpga_wr_ready())
	{
		ret = false;
	}
	/* add more condition later */
	return ret;
}
