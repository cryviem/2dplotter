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

/* 0 is forward, 1 is backward for both
const uint16_t ccw_mode_table[4] = {0x010E, 0x030B, 0x020E, 0x000B};
const uint16_t cw_mode_table[4] = {0x020B, 0x000E, 0x010B, 0x030E};
*/
/* 0 is forward, 1 is backward for x, y reversed */
const uint16_t ccw_mode_table[4] = {0x030E, 0x010B, 0x000E, 0x020B};
const uint16_t cw_mode_table[4] = {0x000B, 0x020E, 0x030B, 0x010E};

pl_data_t	pl_box = {0};

static void speed_planner(float fstart, float fmax, float fend, float acc, float d_total, float* fcruise, float* d_dec_from);
static uint8_t find_quadrant(float x, float y);
static void build_arc_block(float x0, float y0, double radius, double angle, uint16_t mode);

void pl_init(void)
{
	pl_box.cur_pos.x = 0;
	pl_box.cur_pos.y = 0;
	pl_box.feedrate = PLANNER_DEFAULT_FEEDRATE;
	pl_box.accel = PLANNER_DEFAULT_ACCELERATE;
	pl_box.pos_ref = ABSOLUTE_POSITIONING;
	pl_box.state = PL_INVALID;
}

void pl_enable(void)
{
	pl_box.cur_pos.x = 0;
	pl_box.cur_pos.y = 0;
	pl_box.state = PL_READY;
}

void pl_disable(void)
{
	pl_box.state = PL_INVALID;
}

bool pl_is_absolute_coord(void)
{
	return (ABSOLUTE_POSITIONING == pl_box.pos_ref);
}

void pl_set_absolute_coord(void)
{
	pl_box.pos_ref = ABSOLUTE_POSITIONING;
}

void pl_set_relative_coord(void)
{
	pl_box.pos_ref = RELATIVE_POSITIONING;
}

void pl_updatespeed(float spd)
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
	float fval1, fval2, hypotval;

	/* check status */
	if (pl_box.state != PL_READY)
		/* machine is not ready */
		return;

	/* get slot */
	pblock = fpga_wr_buff_start_pl();
	if (NULL == pblock)
		/* no available block */
		return;

	/* check distances */
	fval1 = fabs(tar_pos.x);
	fval2 = fabs(tar_pos.y);
	if ((fval1 < PL_MIN_DISTANCE_TO_GO) && (fval2 < PL_MIN_DISTANCE_TO_GO))
		/* too small to move */
		return;

	/* DDA data fill */
	pblock->mode = 0;

	if (tar_pos.x < 0){
		pblock->mode |= X_DIR_BACKWARD;
	}
	else{
		pblock->mode |= X_DIR_FORWARD;
	}
	pblock->Px = (uint16_t)(fval1 * PL_MM_TO_PULSE);

	if (tar_pos.y < 0){
		pblock->mode |= Y_DIR_BACKWARD;
	}
	else{
		pblock->mode |= Y_DIR_FORWARD;
	}
	pblock->Py = (uint16_t)(fval2 * PL_MM_TO_PULSE);

	hypotval = hypot(fval1, fval2);
	pblock->Q = (uint16_t)(hypotval * PL_MM_TO_PULSE);
	pblock->Stotal = pblock->Q;

	/* speed planner calculation */
	/* this version use simple speed planner without overlap and look ahead algorithms,
	 * so start speed and end speed are fixed
	 * no different between G0 and G1, is_rapid_move is currently not used */

	speed_planner(PLANNER_MIN_FEEDRATE, pl_box.feedrate, PLANNER_MIN_FEEDRATE, pl_box.accel, hypotval, &fval1, &fval2);

	pblock->Fstart =(uint16_t) (PLANNER_MIN_FEEDRATE * PL_MM_TO_PULSE);
	pblock->Fend = (uint16_t) (PLANNER_MIN_FEEDRATE * PL_MM_TO_PULSE);
	pblock->Fcruise = (uint16_t) (fval1 * PL_MM_TO_PULSE);
	pblock->Sdec = (uint16_t) (fval2 * PL_MM_TO_PULSE);
	pblock->Acc = (uint16_t)ACC_TO_N_FACTOR((pl_box.accel*PL_MM_TO_PULSE));
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
	float x0, y0, x1, y1;
	uint8_t quadrant0, quadrant1, currennt_quad;
	double radius, angle0, angle1, delta_rad;

	if (pl_box.state != PL_READY)
		return;

	/*
	 * start point S: x0 = -I; y0 = -J
	 * end point E: x1 = X - I; y1 = Y - J
	 * */
	x0 = -center.x;
	y0 = -center.y;
	quadrant0 = find_quadrant(x0, y0);
	angle0 = atan2(y0, x0);
	if (angle0 < 0) angle0 += M_TWOPI;

	/* calculate radius
	 * r = sqrt(x^2 + y^2) */
	radius = hypot(x0, y0);

	if ((tar_pos.x < PL_ZERO_THRESHOLD) && (tar_pos.y < PL_ZERO_THRESHOLD))
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
			delta_rad = angle1 - angle0;
			build_arc_block(x0, y0, radius, delta_rad, ccw_mode_table[quadrant0]);
		}
		else
		{
			/* multiple blocks */
			/* first move to the end of start quadrant */
			switch (quadrant0)
			{
			case 0:		/* end of quad0 is B(0, R) */
				if (x0 > 0)		/* start point should not B*/
				{
					delta_rad = M_PI_2 - angle0;
					build_arc_block(x0, y0, radius, delta_rad, ccw_mode_table[0]);
				}
				break;
			case 1:		/* end of quad1 is C(-R, 0) */
				if (y0 > 0)		/* start point should not C*/
				{
					delta_rad = M_PI - angle0;
					build_arc_block(x0, y0, radius, delta_rad, ccw_mode_table[1]);
				}
				break;
			case 2:		/* end of quad2 is D(0, -R) */
				/*No chance of start point to be D, no check required */
				delta_rad = M_PI + M_PI_2 - angle0;
				build_arc_block(x0, y0, radius, delta_rad, ccw_mode_table[2]);
				break;
			case 3:		/* end of quad2 is A(R, 0) */
				/*No chance of start point to be A, no check required */
				delta_rad = M_TWOPI - angle0;
				build_arc_block(x0, y0, radius, delta_rad, ccw_mode_table[3]);
				break;
			default:
				break;
			}

			/* next quadrant */
			if (quadrant0 == 3) currennt_quad = 0;
			else currennt_quad = quadrant0 + 1;

			/* move across quadrants among start and stop */
			while (currennt_quad != quadrant1)
			{
				switch (currennt_quad)
				{
				case 0: /*A --> B*/
					build_arc_block(radius, 0, radius, M_PI_2, ccw_mode_table[0]);
					break;
				case 1:	/*B --> C*/
					build_arc_block(0, radius, radius, M_PI_2, ccw_mode_table[1]);
					break;
				case 2:	/*C --> D*/
					build_arc_block(radius, 0, radius, M_PI_2, ccw_mode_table[2]);
					break;
				case 3:	/*D --> A*/
					build_arc_block(0, radius, radius, M_PI_2, ccw_mode_table[3]);
					break;
				default:
					break;
				}

				/* next quadrant */
				if (currennt_quad == 3) currennt_quad = 0;
				else currennt_quad++;
			}

			/* move from begin of stop quadrant to the end point */
			switch (quadrant1)
			{
			case 0:	/*  A to end point */
				if (y1 > 0)	/* end point should not A*/
				{
					delta_rad = angle1;
					build_arc_block(radius, 0, radius, delta_rad, ccw_mode_table[0]);
				}
				break;
			case 1:	/*  B to end point */
				/*No chance of end point to be B, no check required */
				delta_rad = angle1 - M_PI_2;
				build_arc_block(0, radius, radius, delta_rad, ccw_mode_table[1]);
				break;
			case 2:
				/*  C to end point */
				/*No chance of end point to be C, no check required */
				delta_rad = angle1 - M_PI;
				build_arc_block(radius, 0, radius, delta_rad, ccw_mode_table[2]);
				break;
			case 3:	/*  D to end point */
				if (x1 > 0)	/* end point should not D*/
				{
					delta_rad = angle1 - M_PI - M_PI_2;
					build_arc_block(0, radius, radius, delta_rad, ccw_mode_table[3]);
				}
				break;
			default:
				break;
			}
		}
	}
	else
	{
		if ((false == is_full_circle) && (quadrant0 == quadrant1) && ((angle0 - angle1) > 0))
		{
			/* 1 block */
			delta_rad = angle0 - angle1;
			build_arc_block(x0, y0, radius, delta_rad, cw_mode_table[quadrant0]);
		}
		else
		{
			/* multiple blocks */
			/* first move to the end of start quadrant */
			switch (quadrant0)
			{
			case 0:		/* start point to A (R, 0) */
				if (y0 > 0)		/* start point should not A*/
				{
					delta_rad = angle0;
					build_arc_block(x0, y0, radius, delta_rad, cw_mode_table[0]);
				}
				break;
			case 1:		/* start point to B (0, R) */
				/*No chance of start point to be B, no check required */
				delta_rad = angle0 - M_PI_2;
				build_arc_block(x0, y0, radius, delta_rad, cw_mode_table[1]);
				break;
			case 2:		/* start point to C (-R, 0) */
				/*No chance of start point to be C, no check required */
				delta_rad = angle0 - M_PI;
				build_arc_block(x0, y0, radius, delta_rad, cw_mode_table[2]);
				break;
			case 3:		/* start point to D (0, -R) */
				/* start point should not D*/
				if (x0 > 0)
				{
					delta_rad = angle0 - M_PI - M_PI_2;
					build_arc_block(x0, y0, radius, delta_rad, cw_mode_table[3]);
				}
				break;
			default:
				break;
			}

			/* next quadrant */
			if (quadrant0 == 0) currennt_quad = 3;
			else currennt_quad = quadrant0 - 1;

			/* move across quadrants among start and stop */
			while (currennt_quad != quadrant1)
			{
				switch (currennt_quad)
				{
				case 0:	/*B --> A*/
					build_arc_block(0, radius, radius, M_PI_2, cw_mode_table[0]);
					break;
				case 1:	/*C --> B*/
					build_arc_block(radius, 0, radius, M_PI_2, cw_mode_table[1]);
					break;
				case 2: /*D --> C*/
					build_arc_block(0, radius, radius, M_PI_2, cw_mode_table[2]);
					break;
				case 3: /*A --> D*/
					build_arc_block(radius, 0, radius, M_PI_2, cw_mode_table[3]);
					break;
				default:
					break;
				}

				/* next quadrant */
				if (currennt_quad == 0) currennt_quad = 3;
				else currennt_quad--;
			}

			/* move from begin of stop quadrant to the end point */
			switch (quadrant1)
			{
			case 0:	/*  B to end point */
				if (x1 > 0)	/* end point should not B*/
				{
					delta_rad = M_PI_2 - angle1;
					build_arc_block(0, radius, radius, delta_rad, cw_mode_table[0]);
				}
				break;
			case 1:	/*  C to end point */
				if (y1 > 0)	/* end point should not C*/
				{
					delta_rad = M_PI - angle1;
					build_arc_block(radius, 0, radius, delta_rad, cw_mode_table[1]);
				}
				break;
			case 2:	/*  D to end point */
				/*No chance of end point to be D, no check required */
				delta_rad = M_PI + M_PI_2 - angle1;
				build_arc_block(0, radius, radius, delta_rad, cw_mode_table[2]);
				break;
			case 3:	/*  A to end point */
				/*No chance of end point to be A, no check required */
				delta_rad = M_TWOPI - angle1;
				build_arc_block(radius, 0, radius, delta_rad, cw_mode_table[3]);

				break;
			default:
				break;
			}
		}
	}

	/* update current position */
	pl_box.cur_pos.x += tar_pos.x;
	pl_box.cur_pos.y += tar_pos.y;

}

static uint8_t find_quadrant(float x, float y)
{
	uint8_t ret;
	if (x >= 0.0)
	{
		if (y >= 0.0)
			ret = 0;
		else
			ret = 3;
	}
	else
	{
		if (y >= 0.0)
			ret = 1;
		else
			ret = 2;
	}

	return ret;
}

static void build_arc_block(float x0, float y0, double radius, double angle, uint16_t mode)
{
	pl_block_t* pblock = NULL;
	float fval1, fval2, fdistance;
	/* get slot */
	pblock = fpga_wr_buff_start_pl();
	if (NULL == pblock)
		return;

	/* DDA data fill */
	pblock->mode = mode;
	fdistance = (float)(radius * angle);
	pblock->Stotal = (uint16_t)(fdistance * PL_MM_TO_PULSE);

	fval1 = fabs(x0);
	fval2 = fabs(y0);

	/* in case radius is too big, scale down all px py and Q to fix uint16*/
	if (radius > PL_MAX_FLOAT_TO_U16)
	{
		fval1 = fval1 * PL_MAX_FLOAT_TO_U16 / radius;
		fval2 = fval2 * PL_MAX_FLOAT_TO_U16 / radius;
		radius = PL_MAX_FLOAT_TO_U16;
	}

	pblock->Px = (uint16_t)(fval2 * PL_MM_TO_PULSE);
	pblock->Py = (uint16_t)(fval1 * PL_MM_TO_PULSE);
	pblock->Q = (uint16_t)(radius * PL_MM_TO_PULSE);

	/* speed planner calculation */
	/* this version use simple speed planner without overlap and look ahead algorithms,
	 * so start speed and end speed are fixed
	 * no different between G0 and G1, is_rapid_move is currently not used */

	speed_planner(PLANNER_MIN_FEEDRATE, pl_box.feedrate, PLANNER_MIN_FEEDRATE, pl_box.accel, fdistance, &fval1, &fval2);

	pblock->Fstart =(uint16_t)(PLANNER_MIN_FEEDRATE * PL_MM_TO_PULSE);
	pblock->Fend = (uint16_t)(PLANNER_MIN_FEEDRATE * PL_MM_TO_PULSE);
	pblock->Fcruise = (uint16_t)(fval1 * PL_MM_TO_PULSE);
	pblock->Sdec = (uint16_t)(fval2 * PL_MM_TO_PULSE);
	pblock->Acc = (uint16_t)ACC_TO_N_FACTOR((pl_box.accel*PL_MM_TO_PULSE));
	pblock->cmd = FPGA_CMD_PLOTTER_MOVE;

	/* confirm data ready in buffer */
	fpga_wr_buff_cmplt();
}

float pl_calc_dx(float x)
{
	float ret;

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

float pl_calc_dy(float y)
{
	float ret;

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
static void speed_planner(float fstart, float fmax, float fend, float acc, float d_total, float* fcruise, float* d_dec_from)
{
	double dval1, dval2, inversed_2_acc;

	inversed_2_acc = 2.0*acc;
	inversed_2_acc = 1 / inversed_2_acc;
	/* s = (v^2 - v0^2) / 2a */
	/* calculate accelerate distance */
	dval1 = (fmax*fmax - fstart*fstart) * inversed_2_acc;
	/* calculate decelerate distance */
	dval2 = (fmax*fmax - fend*fend) * inversed_2_acc;

	if (d_total >= (dval1 + dval2))
	{
		/* for trapeziod plan, the distance of block must greater than sum of accelerate and decelerate distances */
		*fcruise = fmax;
		*d_dec_from = (float)(d_total - dval2);
	}
	else
	{
		/* triangle plan */
		/* s = s_acc + s_dec
		 *   = (v^2 - v_start^2) / 2a    +    (v^2 - v_end^2) / 2a
		 *   = (2v^2 - v_start^2 - v_end^2) / 2a
		 * v = sqrt((2as + v_start^2 + v_end^2) / 2) */
		/* find new fcruise */
		dval1 = (2.0*acc)*d_total + fstart*fstart + fend*fend;
		dval1 *= 0.5;
		dval1 = sqrt(dval1);
		*fcruise = (float)dval1;
		/* calculate new decelerate distance */
		dval2 = (dval1*dval1 - fend*fend) * inversed_2_acc;

		*d_dec_from = (float)(d_total - dval2);
		/* continue */
	}
}
