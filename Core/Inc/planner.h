/*
 * planner.h
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */

#ifndef PLANNER_H
#define PLANNER_H

#define PL_ZERO_THRESHOLD							0.0125f
#define PL_MM_TO_PULSE								80.0f	/* pulse/mm */
#define PL_MAX_FLOAT_TO_U16							819.1875f /* 65535 / PL_MM_TO_PULSE */
#define PL_MIN_DISTANCE_TO_GO						0.05f	/* mm */
#define PLANNER_MIN_FEEDRATE						5.0f	/* mm/s */
#define PLANNER_MAX_FEEDRATE						60.0f	/* mm/s */
/* PLANNER DEFAULT PARAMETERS */
#define PLANNER_DEFAULT_FEEDRATE					20.0f	/* mm/s */
#define PLANNER_DEFAULT_ACCELERATE					800.0f	/* mm/s2 */

#define ABSOLUTE_POSITIONING						0
#define RELATIVE_POSITIONING						1

#define FPGA_CLOCK_RATE								50000000	/*50 MHz*/
#define ACC_TO_N_FACTOR(x)							(FPGA_CLOCK_RATE / x)

#define X_DIR_FORWARD								0x0000;
#define X_DIR_BACKWARD								0x0100;
/* y is reversed */
#define Y_DIR_FORWARD								0x0200;
#define Y_DIR_BACKWARD								0x0000;

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

typedef struct {
	uint16_t	cmd;
	uint16_t	mode;
	uint16_t	Px;
	uint16_t	Py;
	uint16_t	Q;
	uint16_t	Fstart;
	uint16_t	Fcruise;
	uint16_t	Fend;
	uint16_t	Acc;
	uint16_t	Stotal;
	uint16_t	Sdec;
} pl_block_t;

void pl_init(void);
void pl_enable(void);
void pl_disable(void);
void pl_set_absolute_coord(void);
void pl_set_relative_coord(void);
bool pl_is_absolute_coord(void);
void pl_updatespeed(float spd);
float pl_calc_dx(float x);
float pl_calc_dy(float y);
void pl_line(pos_t tar_pos, bool is_rapid_move);
void pl_arc(pos_t tar_pos, pos_t center, bool is_ccw, bool is_circle);
#endif /* PLANNER_H */
