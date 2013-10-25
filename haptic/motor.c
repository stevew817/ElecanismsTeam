/*************************************************************
*	MOTOR CONTROL LIBRARY
*	ELECANISMS MINIPROJECT 4
*
*	motor.c
*
*	working:
*		D6 on the board is shot (stuck low)
*		So, set D5(IN2) high, use INV for direction control,
*		and use the disable pins for stopping the motor
*************************************************************/
#include <p24FJ128GB206.h>
#include "pin.h"
#include "common.h"
#include "motor.h"
#include "oc.h"

#define CUR &A[0]

uint16_t m_dir;
/* ----------------------------------------------------------
*	MOTOR CONTROL CODE
-----------------------------------------------------------*/
void motor_setup() {
	//configure pins
	pin_digitalIn(SF);
	pin_digitalIn(ENC);
	pin_digitalOut(D1);
	pin_digitalOut(D2);
	pin_digitalOut(ENA);
	pin_digitalOut(IN1);
	pin_digitalOut(IN2);
	pin_digitalOut(SLEW);
	pin_digitalOut(INV);
	
	pin_analogIn(FB);
	pin_analogIn(EMF);
	pin_analogIn(CUR);
	
	//set default values to turn on the motor driver
	//both motor inputs are high: motor not turning
	pin_clear(IN1);
	pin_set(IN2);
	//fast slew rate (don't care about power consumption atm)
	pin_set(SLEW);
	//disable D1 shutdown
	//keep D2 shutdown
	pin_clear(D1);
	pin_clear(D2);
	//left rotation
	pin_clear(INV);
	//finally, enable the whole chip
	pin_set(ENA);
	
	//setup the PWM-ing
	oc_pwm(MOTOR_OC, D2, NULL, 230, 0xFFFF);
}

void motor_turn_left() {
	pin_clear(INV);
}

void motor_turn_right() {
	pin_set(INV);
}

void motor_stop() {
	motor_set_speed(0x0000);
}

void motor_start() {
	motor_set_speed(0xFFFF);
}

void motor_set_speed(uint16_t speed) {
	pin_write(D2, speed);
}

uint16_t motor_get_speed() {
	return pin_read(D2);
}

uint16_t motor_get_direction() {
	m_dir = pin_read(CUR);
	if (m_dir >= 0x8000) {
		return 1;
	}
	else
		return 0;
}

