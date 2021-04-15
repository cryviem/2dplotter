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

void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_dec_from);

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
	pl_block_t* pblock = NULL;
	bool is_full_circle = false;
	uint32_t u32radius;
	double start_angle, end_angle;

	/* check if is full circle command */
	if ((tar_pos.x == 0) && (tar_pos.y == 0))
	{
		is_full_circle = true;
	}

	/* calculate radius
	 * r = sqrt(x^2 + y^2) */
	u32radius = center.x*center.x + center.y*center.y;
	u32radius = SquareRootRounded(u32radius);

	/* calculate the angle of start and end point
	 * consider C(0, 0)
	 * start point S: xs = -I; ys = -J
	 * end point E: xe = X - I; ye = Y - J*/
	start_angle = atan2((double)(-center.y), (double)(-center.x));
	if (start_angle < 0) start_angle += M_TWOPI;

	if (false == is_full_circle)
	{
		end_angle = atan2((double)(tar_pos.y-center.y), (double)(tar_pos.x-center.x));
		if (end_angle < 0) end_angle += M_TWOPI;
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
void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_dec_from)
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
