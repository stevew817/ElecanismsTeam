/*************************************************************
*	MOTOR CONTROL LIBRARY
*	ELECANISMS MINIPROJECT 4
*
*	motor.h
*************************************************************/

#define ENC			&D[0]
#define SF			&D[1]
#define D2			&D[2]
#define D1			&D[3]
#define ENA			&D[4]
#define IN2			&D[5]
#define IN1			&D[6]
#define SLEW		&D[7]
#define INV			&D[8]

#define CUR			&A[0]
#define EMF			&A[1]
#define FB			&A[2]

#define MOTOR_OC	&oc1

void motor_setup();
void motor_turn_left();
void motor_turn_right();
void motor_stop();
void motor_start();
void motor_set_speed(uint16_t);