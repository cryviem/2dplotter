/*
 * planner.c
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */
#include <math.h>
#include "app_common.h"
#include "planner.h"
#include "fpga.h"
#include "do_math.h"


pl_data_t	pl_box = {0};

static void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_dec_from);
static uint8_t find_quadrant(int32_t x, int32_t y);

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
	uint16_t u16val1, u16val2;
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
		u16val1 = (uint16_t)(-tar_pos.x);
	}
	else
	{
		u16val1 = (uint16_t)tar_pos.x;
	}

	if (tar_pos.y < 0)
	{
		pblock->mode |= Y_DIR_BACKWARD;
		u16val2 = (uint16_t)(-tar_pos.y);
	}
	else
	{
		u16val2 = (uint16_t)tar_pos.y;
	}

	pblock->Px = u16val1;
	pblock->Py = u16val2;

	u32val = u16val1*u16val1 + u16val2*u16val2;
	u32val = SquareRootRounded(u32val);
	pblock->Q = u32val;
	pblock->Stotal = u32val;

	/* speed planner calculation */
	/* this version use simple speed planner without overlap and look ahead algorithms,
	 * so start speed and end speed are fixed
	 * no different between G0 and G1, is_rapid_move is currently not used */

	speed_planner(PLANNER_MIN_FEEDRATE, pl_box.feedrate, PLANNER_MIN_FEEDRATE, pl_box.accel, u32val, &u16val1, &u16val2);

	pblock->Fstart = PLANNER_MIN_FEEDRATE;
	pblock->Fend = PLANNER_MIN_FEEDRATE;
	pblock->Fcruise = u16val1;
	pblock->Sdec = u16val2;
	pblock->Acc = (uint16_t)ACC_TO_N_FACTOR(pl_box.accel);
	pblock->cmd = FPGA_CMD_PLOTTER_MOVE;

	/* confirm data ready in buffer */
	fpga_wr_buff_cmplt();

	/* update current position */
	pl_box.cur_pos.x += tar_pos.x;
	pl_box.cur_pos.y += tar_pos.y;

}

void pl_arc(pos_t tar_pos, pos_t center, bool is_ccw)
{

	bool is_full_circle = false;
	uint32_t u32radius;
	int32_t x0, y0, x1, y1;
	uint8_t quadrant0, quadrant1;
	double angle0, angle1, delta_rad;

	/*
	 * start point S: x0 = -I; y0 = -J
	 * end point E: x1 = X - I; y1 = Y - J
	 * */
	x0 = -center.x;
	y0 = -center.y;
	quadrant0 = find_quadrant(x0, y0);
	angle0 = atan2((double)y0, (double)x0);
	if (angle0 < 0) angle0 += M_TWOPI;

	/* calculate radius
	 * r = sqrt(x^2 + y^2) */
	u32radius = x0*x0 + y0*y0;
	u32radius = SquareRootRounded(u32radius);

	if ((tar_pos.x == 0) && (tar_pos.y == 0))
	{
		is_full_circle = true;
		x1 = x0;
		y1 = y0;
		quadrant1 = quadrant0;
		angle1 = angle0;
	}
	else
	{
		x1 = tar_pos.x - center.x;
		y1 = tar_pos.y - center.y;
		quadrant1 = find_quadrant(x1, y1);
		angle1 = atan2((double)y1, (double)x1);
		if (angle1 < 0) angle1 += M_TWOPI;
	}

	if (true == is_ccw)
	{
		if ((false == is_full_circle) && (quadrant0 == quadrant1) && ((angle1 - angle0) > 0))
		{
			/* 1 block */
			delta_rad =
		}
		else
		{
			/* multiple blocks */
		}
	}
	else
	{
		if ((false == is_full_circle) && (quadrant0 == quadrant1) &&
			(((quadrant0 < 2) && (x0 < x1)) || ((quadrant0 >= 2) && (x1 < x0))))
		{
			/* 1 block */
		}
		else
		{
			/* multiple blocks */
		}
	}


}


static uint8_t find_quadrant(int32_t x, int32_t y)
{
	uint8_t ret;
	if (x >= 0)
	{
		if (y >= 0)
			ret = 0;
		else
			ret = 3;
	}
	else
	{
		if (y >= 0)
			ret = 1;
		else
			ret = 2;
	}
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

/* calculate speed plan for the block*/
static void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_dec_from)
{
	uint32_t u32val1, u32val2;

	/* s = (v^2 - v0^2) / 2a */
	/* calculate accelerate distance */
	u32val1 = (fmax*fmax - fstart*fstart) / acc;
	u32val1 >>= 1;
	/* calculate decelerate distance */
	u32val2 = (fmax*fmax - fend*fend) / acc;
	u32val2  >>= 1;

	if (d_total >= (u32val1 + u32val2))
	{
		/* for trapeziod plan, the distance of block must greater than sum of accelerate and decelerate distances */
		*fcruise = fmax;
		*d_dec_from = d_total - u32val2;
	}
	else
	{
		/* triangle plan */
		/* s = s_acc + s_dec
		 *   = (v^2 - v_start^2) / 2a    +    (v^2 - v_end^2) / 2a
		 *   = (2v^2 - v_start^2 - v_end^2) / 2a
		 * v = sqrt((2as + v_start^2 + v_end^2) / 2) */
		/* find new fcruise */
		u32val1 = ((acc << 1)*d_total + fstart*fstart + fend*fend) >> 1;
		u32val1 = SquareRootRounded(u32val1);
		*fcruise = u32val1;
		/* calculate new decelerate distance */
		u32val2 = (u32val1*u32val1 - fend*fend) / acc;
		u32val2 = u32val2 >> 1;
		*d_dec_from = d_total - u32val2;
		/* continue */
	}
}
