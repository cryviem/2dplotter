/*
 * planner.h
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */

#ifndef PLANNER_H
#define PLANNER_H

#include "stm32f4xx_hal.h"

#define PLANNER_MIN_FEEDRATE						5		/* mm/s */
#define PLANNER_MAX_FEEDRATE						60		/* mm/s */
/* PLANNER DEFAULT PARAMETERS */
#define PLANNER_DEFAULT_FEEDRATE					20		/* mm/s */
#define PLANNER_DEFAULT_ACCELERATE					800		/* mm/s2 */

#define ABSOLUTE_POSITIONING						0
#define RELATIVE_POSITIONING						1

typedef struct {
	float	x;		/* mm */
	float	y;		/* mm */
} pos_t;

typedef enum {
	PL_INVALID,
	PL_READY,
} pl_state_en;

typedef struct {
	pos_t	cur_pos;		/* mm */
	float	feedrate;		/* mm/s */
	float	accel;			/* mm/s2 */
	uint8_t	pos_ref;
	pl_state_en	state;
} pl_data_t;

#endif /* PLANNER_H */
