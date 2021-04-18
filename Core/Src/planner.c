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
static bool calc_point_on_arc(int32_t x, int32_t y, uint8_t* pinfo, double* pangle);

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
	int32_t x0, y0, x1, y1;
	uint8_t point0_info, point1_info;
	double point0_angle, point1_angle;

	/*
	 * start point S: x0 = -I; y0 = -J
	 * end point E: x1 = X - I; y1 = Y - J
	 * */
	x0 = -center.x;
	y0 = -center.y;

	if ((tar_pos.x == 0) && (tar_pos.y == 0))
	{
		is_full_circle = true;
	}
	else
	{
		x1 = tar_pos.x - center.x;
		y1 = tar_pos.y - center.y;
	}

	/* calculate radius
	 * r = sqrt(x^2 + y^2) */
	u32radius = x0*x0 + y0*y0;
	u32radius = SquareRootRounded(u32radius);

	/* check special points */

	calc_point_on_arc(x0, y0, &point0_info, &point0_angle);

	if (false == is_full_circle)
	{
		calc_point_on_arc(x1, y1, &point1_info, &point1_angle);
	}




}

/*
 * pinfo:
 * 0 - 3 quarants
 * 4 point A
 * 5 point B
 * 6 point C
 * 7 point D
 * others invalid*/
static bool calc_point_on_arc(int32_t x, int32_t y, uint8_t* pinfo, double* pangle)
{

	if ((x == 0) && (y == 0))
		return false;

	/* first check special points */
	if (x == 0)
	{
		if (y > 0)
		{
			/* point B */
			*pinfo = 5;
			*pangle = M_PI_2;
		}
		else
		{
			/* point D */
			*pinfo = 7;
			*pangle = M_PI+M_PI_2;
		}
	}
	else if (y == 0)
	{
		if (x > 0)
		{
			/* point A */
			*pinfo = 4;
			*pangle = 0;
		}
		else
		{
			/* point C */
			*pinfo = 6;
			*pangle = M_PI;
		}
	}
	else
	{
		/* not a special point */
		*pangle = atan2((double)y, (double)x);
		if (*pangle < 0) *pangle += M_TWOPI;

		if (*pangle < M_PI_2)
		{
			*pinfo = 0;
		}
		else if (*pangle < M_PI)
		{
			*pinfo = 1;
		}
		else if (*pangle < (M_PI+M_PI_2))
		{
			*pinfo = 2;
		}
		else if (*pangle < M_TWOPI)
		{
			*pinfo = 3;
		}
	}
	return true;
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
