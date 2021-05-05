/*
 * debug.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Admin
 */
#include "app_common.h"
#include "fpga.h"
#include "planner.h"
#include "debug.h"

char err_str[10] = "error: 00";
#define DIGIT1					7
#define DIGIT0					8
#define NUMBER_OFFSET			30
void dbg_D0(void)
{
	pl_block_t* pblock = NULL;

	/* get slot */
	pblock = fpga_wr_buff_start_pl();
	if (NULL == pblock)
		return;

	/* DDA data fill */
	pblock->cmd = 1035;
	pblock->mode = 270;
	pblock->Px = 65533;
	pblock->Py = 376;
	pblock->Q = 65535;
	pblock->Fstart = 400;
	pblock->Fcruise = 1600;
	pblock->Fend = 400;
	pblock->Acc = 781;
	pblock->Stotal = 552;
	pblock->Sdec = 533;

	/* confirm data ready in buffer */
	fpga_wr_buff_cmplt();
}

void dbg_D1(void)
{
	pl_block_t* pblock = NULL;

	/* get slot */
	pblock = fpga_wr_buff_start_pl();
	if (NULL == pblock)
		return;

	/* DDA data fill */
	pblock->cmd = 1035;
	pblock->mode = 779;
	pblock->Px = 65535;
	pblock->Py = 0;
	pblock->Q = 65535;
	pblock->Fstart = 400;
	pblock->Fcruise = 1600;
	pblock->Fend = 400;
	pblock->Acc = 781;
	pblock->Stotal = 501;
	pblock->Sdec = 483;

	/* confirm data ready in buffer */
	fpga_wr_buff_cmplt();
}

void error_report(db_errorcode_en err)
{
	uint8_t tmp1, tmp2;

	if (err < DB_NUMOFERR)
	{
		tmp1 = (uint8_t)err;
		tmp2 = tmp1 % 10;
		err_str[DIGIT0] = tmp2 + NUMBER_OFFSET;
		tmp1 /= 10;
		tmp2 = tmp1 % 10;
		err_str[DIGIT1] = tmp2 + NUMBER_OFFSET;

		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)err_str, 9);
		pl_disable();
		fpga_disable();
		LED_RED_ON();
	}
}

