/*
 * planner.h
 *
 *  Created on: Apr 7, 2021
 *      Author: hanguyen
 */

#ifndef PLANNER_H
#define PLANNER_H

#define PLANNER_MIN_FEEDRATE						400		/* pulse/s */
#define PLANNER_MAX_FEEDRATE						5000	/* pulse/s */
/* PLANNER DEFAULT PARAMETERS */
#define PLANNER_DEFAULT_FEEDRATE					1500	/* pulse/s */
#define PLANNER_DEFAULT_ACCELERATE					64000	/* pulse/s2 */

#define ABSOLUTE_POSITIONING						0
#define RELATIVE_POSITIONING						1

#define X_DIR_FORWARD								(uint16_t)0x0000
#define X_DIR_BACKWARD								(uint16_t)0x0100
#define Y_DIR_FORWARD								(uint16_t)0x0000
#define Y_DIR_BACKWARD								(uint16_t)0x0200
#define PX_INCREASE									(uint16_t)0x0002
#define PX_DECREASE									(uint16_t)0x0003
#define PY_INCREASE									(uint16_t)0x0008
#define PY_DECREASE									(uint16_t)0x000C
#define FPGA_CLOCK_RATE								(uint32_t)50000000	/*50 MHz*/
#define ACC_TO_N_FACTOR(x)							(FPGA_CLOCK_RATE / x)
typedef struct {
	int16_t	x;		/* pulse */
	int16_t	y;		/* pulse */
} pos_t;

typedef enum {
	PL_INVALID,
	PL_READY,
} pl_state_en;

typedef struct {
	pos_t		cur_pos;		/* pulse */
	uint16_t	feedrate;		/* pulse/s */
	uint32_t	accel;			/* pulse/s2 */
	uint8_t		pos_ref;
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

void pl_updatespeed(uint16_t spd);
bool pl_is_absolute_coord(void);
int16_t pl_calc_dx(int16_t x);
int16_t pl_calc_dy(int16_t y);
void pl_line(pos_t tar_pos, bool is_rapid_move);
void pl_arc(pos_t tar_pos, pos_t center, bool is_ccw);
#endif /* PLANNER_H */
