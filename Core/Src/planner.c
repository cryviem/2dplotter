/*
 * planner.c
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */
#include "app_common.h"
#include "planner.h"
#include "fpga.h"
#include "do_math.h"
pl_data_t	pl_box = {0};

void pl_init(void)
{
	pl_box.cur_pos.x = 0;
	pl_box.cur_pos.y = 0;
	pl_box.feedrate = PLANNER_DEFAULT_FEEDRATE;
	pl_box.accel = PLANNER_DEFAULT_ACCELERATE;
	pl_box.pos_ref = ABSOLUTE_POSITIONING;
	pl_box.state = PL_READY;
}

bool pl_is_absolute_coord(void)
{
	return (ABSOLUTE_POSITIONING == pl_box.pos_ref);
}

void pl_updatespeed(uint16_t spd)
{
	if (spd < PLANNER_MIN_FEEDRATE)
		pl_box.feedrate = PLANNER_MIN_FEEDRATE;
	else if (spd > PLANNER_MAX_FEEDRATE)
		pl_box.feedrate = PLANNER_MAX_FEEDRATE;
	else
		pl_box.feedrate = spd;
}

void pl_line(pos_t tar_pos, bool is_rapid_move)
{
	pl_block_t* pblock = NULL;
	uint16_t dx, dy;
	uint32_t u32val;

	/* get slot */
	pblock = fpga_wr_buff_start_pl();
	if (NULL == pblock)
		return;

	/* DDA data fill */
	pblock->mode = 0;

	if (tar_pos.x < 0)
	{
		pblock->mode |= X_DIR_BACKWARD;
		dx = (uint16_t)(-tar_pos.x);
	}
	else
	{
		dx = (uint16_t)tar_pos.x;
	}

	if (tar_pos.y < 0)
	{
		pblock->mode |= Y_DIR_BACKWARD;
		dy = (uint16_t)(-tar_pos.y);
	}
	else
	{
		dy = (uint16_t)tar_pos.y;
	}

	pblock->Px = dx;
	pblock->Py = dy;

	u32val = dx*dx + dy*dy;
	u32val = SquareRootRounded(u32val);
	pblock->Q = u32val;
	pblock->Stotal = u32val;

	/* speed planner calculation */
	/* this version use simple speed planner without overlap and look ahead algorithms,
	 * so start speed and end speed are fixed */




}

void pl_arc(pos_t tar_pos, pos_t center, bool is_ccw)
{

}

int16_t pl_calc_dx(int16_t x)
{
	int16_t ret = 0;

	if (true == pl_is_absolute_coord())
	{
		ret = x - pl_box.cur_pos.x;
	}
	else
	{
		ret = x;
	}

	return ret;
}

int16_t pl_calc_dy(int16_t y)
{
	int16_t ret = 0;

	if (true == pl_is_absolute_coord())
	{
		ret = y - pl_box.cur_pos.y;
	}
	else
	{
		ret = y;
	}

	return ret;
}

void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_deac_from)
{
	uint32_t u32val1, u32val2;

	u32val1 = (fmax*fmax - fstart*fstart) / (2*acc);
	u32val2 = (fmax*fmax - fend*fend) / (2*acc);

	if (d_total >= (u32val1 + u32val2))
	{
		*fcruise = fmax;
		*d_deac_from = d_total - u32val2;
	}
	else
	{
		/* find new fcruise */
		u32val1 = (2*acc*d_total + fstart*fstart + fend*fend) / 2;
		u32val1 = SquareRootRounded(u32val1);
		*fcruise = u32val1;
		/* continue */
	}
}
