/*
 * debug.c
 *
 *  Created on: Apr 25, 2021
 *      Author: Admin
 */
#include "app_common.h"
#include "fpga.h"

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


