/*
 * planner.c
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */

#include "planner.h"

pl_data_t	pl_box = {0};

void planner_init(void)
{
	pl_box.cur_pos.x = 0;
	pl_box.cur_pos.y = 0;
	pl_box.feedrate = PLANNER_DEFAULT_FEEDRATE;
	pl_box.accel = PLANNER_DEFAULT_ACCELERATE;
	pl_box.pos_ref = ABSOLUTE_POSITIONING;
	pl_box.state = PL_READY;
}

void planner_updspdmmpm(float mmpm)
{
	float tmp = mmpm / 60;
	if (tmp < PLANNER_MIN_FEEDRATE)
		tmp = PLANNER_MIN_FEEDRATE;
	if (tmp > PLANNER_MAX_FEEDRATE)
		tmp = PLANNER_MAX_FEEDRATE;
	pl_box.feedrate = tmp;
}

