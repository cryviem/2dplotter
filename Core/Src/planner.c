/*
 * planner.c
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */
#include "app_common.h"
#include "planner.h"

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

void pl_updspdmmpm(float mmpm)
{
	float tmp = mmpm / 60;
	if (tmp < PLANNER_MIN_FEEDRATE)
		tmp = PLANNER_MIN_FEEDRATE;
	if (tmp > PLANNER_MAX_FEEDRATE)
		tmp = PLANNER_MAX_FEEDRATE;
	pl_box.feedrate = tmp;
}

void pl_line(pos_t tar_pos, bool is_rapid_move)
{

}

void pl_arc(pos_t tar_pos, pos_t center, bool is_ccw)
{

}

float pl_calc_dx(float x)
{
	float ret = 0;

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
	float ret = 0;

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

