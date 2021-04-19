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

#define QUARANT_1				0x01
#define QUARANT_2				0x02
#define QUARANT_3				0x04
#define QUARANT_4				0x08
#define POINT_A					0x10
#define POINT_B					0x20
#define POINT_C					0x40
#define POINT_D					0x80

#define IS_BITS_SET(value, flag)	(value & flag)? true:false

pl_data_t	pl_box = {0};

static void speed_planner(uint32_t fstart, uint32_t fmax, uint32_t fend, uint32_t acc, uint32_t d_total, uint16_t* fcruise, uint16_t* d_dec_from);
static bool calc_point_on_arc(int32_t x, int32_t y, uint8_t* pinfo, double* pangle);
static bool is_same_quarant(uint8_t info1, uint8_t info2);

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
	double point0_angle, point1_angle, delta_angle;

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

	if (false == is_ccw)
	{
		/* cw */
		if (false == is_full_circle)
		{
			if(true == is_same_quarant(point0_info, point1_info))
			{
				if (point0_info == POINT_A)
				{
					delta_angle = M_TWOPI - point1_angle;
				}
				else
				{
					delta_angle = point0_angle - point1_angle;
				}

				if(delta_angle > 0)
				{
					/* 1 block needed */
				}
			}
		}
	}


}


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
			*pinfo = POINT_B;
			*pangle = M_PI_2;
		}
		else
		{
			/* point D */
			*pinfo = POINT_D;
			*pangle = M_PI+M_PI_2;
		}
	}
	else if (y == 0)
	{
		if (x > 0)
		{
			/* point A */
			*pinfo = POINT_A;
			*pangle = 0;
		}
		else
		{
			/* point C */
			*pinfo = POINT_C;
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
			*pinfo = QUARANT_1;
		}
		else if (*pangle < M_PI)
		{
			*pinfo = QUARANT_2;
		}
		else if (*pangle < (M_PI+M_PI_2))
		{
			*pinfo = QUARANT_3;
		}
		else if (*pangle < M_TWOPI)
		{
			*pinfo = QUARANT_4;
		}
	}
	return true;
}

static bool is_same_quarant(uint8_t info1, uint8_t info2)
{
	bool ret = false;

	switch (info1)
	{
	case QUARANT_1:
		ret = IS_BITS_SET(info2, (POINT_A | POINT_B));
		break;
	case QUARANT_2:
		ret = IS_BITS_SET(info2, (POINT_B | POINT_C));
		break;
	case QUARANT_3:
		ret = IS_BITS_SET(info2, (POINT_C | POINT_D));
		break;
	case QUARANT_4:
		ret = IS_BITS_SET(info2, (POINT_D | POINT_A));
		break;
	case POINT_A:
		ret = IS_BITS_SET(info2, (QUARANT_1 | QUARANT_4 | POINT_B | POINT_D));
		break;
	case POINT_B:
		ret = IS_BITS_SET(info2, (QUARANT_1 | QUARANT_2 | POINT_A | POINT_C));
		break;
	case POINT_C:
		ret = IS_BITS_SET(info2, (QUARANT_2 | QUARANT_3 | POINT_B | POINT_D));
		break;
	case POINT_D:
		ret = IS_BITS_SET(info2, (QUARANT_3 | QUARANT_4 | POINT_C | POINT_A));
		break;
	default:
		break;
	}
	return ret;
}

uint16_t get_mode_from_quarant(uint8_t info, bool is_ccw)
{
	uint16_t ret = 0;
	switch (info)
	{
	case QUARANT_1:
		if (false == is_ccw)
			ret = (X_DIR_FORWARD + Y_DIR_BACKWARD + PX_DECREASE + PY_INCREASE);
		else
			ret = (X_DIR_BACKWARD + Y_DIR_FORWARD + PX_INCREASE + PY_DECREASE);
		break;
	case QUARANT_2:
		if (false == is_ccw)
			ret = (X_DIR_FORWARD + Y_DIR_FORWARD + PX_INCREASE + PY_DECREASE);
		else
			ret = (X_DIR_BACKWARD + Y_DIR_BACKWARD + PX_DECREASE + PY_INCREASE);
		break;
	case QUARANT_3:
		if (false == is_ccw)
			ret = (X_DIR_BACKWARD + Y_DIR_FORWARD + PX_DECREASE + PY_INCREASE);
		else
			ret = (X_DIR_FORWARD + Y_DIR_BACKWARD + PX_INCREASE + PY_DECREASE);
		break;
		break;
	case QUARANT_4:
		if (false == is_ccw)
			ret = (X_DIR_BACKWARD + X_DIR_BACKWARD + PX_INCREASE + PY_DECREASE);
		else
			ret = (X_DIR_FORWARD + Y_DIR_FORWARD + PX_DECREASE + PY_INCREASE);
		break;
	case POINT_A:
		ret = IS_BITS_SET(info2, (QUARANT_1 | QUARANT_4 | POINT_B | POINT_D));
		break;
	case POINT_B:
		ret = IS_BITS_SET(info2, (QUARANT_1 | QUARANT_2 | POINT_A | POINT_C));
		break;
	case POINT_C:
		ret = IS_BITS_SET(info2, (QUARANT_2 | QUARANT_3 | POINT_B | POINT_D));
		break;
	case POINT_D:
		ret = IS_BITS_SET(info2, (QUARANT_3 | QUARANT_4 | POINT_C | POINT_A));
		break;
	default:
		break;
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
