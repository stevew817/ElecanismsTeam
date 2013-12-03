/**********************************************
*
*	Brushless motor timing functions for PIC24 Olin College board
*
*	Adapted from the BruGi project, code.google.com/p/brushless-gimbal
*
*	2013, Steven Cooreman
***********************************************/
#include <p24FJ128GB206.h>
#include <math.h>
#include <stdint.h>
#include "common.h"
#include <stdio.h>
#include <libpic30.h>
#include "oc.h"
#include "timer.h"
#include <stdbool.h>
#include "externvariables.h"
#include "brushless.h"

void setupMotors(void) {
	//initialize for PWM operation
	oc_pwm(MOT_0_A_OC, MOT_0_A_PIN, MOT_TIM, 32000., 0);
	oc_pwm(MOT_0_B_OC, MOT_0_B_PIN, MOT_TIM, 32000., 0);
	oc_pwm(MOT_0_C_OC, MOT_0_C_PIN, MOT_TIM, 32000., 0);
	oc_pwm(MOT_1_A_OC, MOT_1_A_PIN, MOT_TIM, 32000., 0);
	oc_pwm(MOT_1_B_OC, MOT_1_B_PIN, MOT_TIM, 32000., 0);
	oc_pwm(MOT_1_C_OC, MOT_1_C_PIN, MOT_TIM, 32000., 0);
	oc_pwm(LOOP_OC, LOOP_PIN, MOT_TIM, 0.001, 0x8FFF);
	
	timer_stop(MOT_TIM);
	
	//Set to dual compare for centered PWM
	*(MOT_0_A_OC->OCxCON1) = (*(MOT_0_A_OC->OCxCON1) & 0xFFF8) | 0x0005;
	*(MOT_0_B_OC->OCxCON1) = (*(MOT_0_B_OC->OCxCON1) & 0xFFF8) | 0x0005;
	*(MOT_0_C_OC->OCxCON1) = (*(MOT_0_C_OC->OCxCON1) & 0xFFF8) | 0x0005;
	*(MOT_1_A_OC->OCxCON1) = (*(MOT_1_A_OC->OCxCON1) & 0xFFF8) | 0x0005;
	*(MOT_1_B_OC->OCxCON1) = (*(MOT_1_B_OC->OCxCON1) & 0xFFF8) | 0x0005;
	*(MOT_1_C_OC->OCxCON1) = (*(MOT_1_C_OC->OCxCON1) & 0xFFF8) | 0x0005;
	
	//!!!!!!!!!!!!!!!!synchronize to timer 1!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	*(MOT_0_A_OC->OCxCON2) = 0x000B;
	*(MOT_0_B_OC->OCxCON2) = 0x000B;
	*(MOT_0_C_OC->OCxCON2) = 0x000B;
	*(MOT_1_A_OC->OCxCON2) = 0x000B;
	*(MOT_1_B_OC->OCxCON2) = 0x000B;
	*(MOT_1_C_OC->OCxCON2) = 0x000B;
	
	*(MOT_0_A_OC->OCxTMR) = 0;
	*(MOT_0_B_OC->OCxTMR) = 0;
	*(MOT_0_C_OC->OCxTMR) = 0;
	*(MOT_1_A_OC->OCxTMR) = 0;
	*(MOT_1_B_OC->OCxTMR) = 0;
	*(MOT_1_C_OC->OCxTMR) = 0;
	
	//set timer to BruGi emulation
	//32khz PWM, means 512 ticks => 512*32000 = 16 MHz
	timer_setFreq(MOT_TIM, 32000.);
	poke(MOT_TIM->PRx, 511);
	timer_start(MOT_TIM);
	
	//set up motor update callback
	//timer_every(MOT_UPD_TIM, 0.001f, &update_motors_callback);
	timer_setPeriod(MOT_UPD_TIM, 0.001);
    timer_start(MOT_UPD_TIM);
	
}

inline void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint8_t* pwmSin)
{
  uint8_t posStep;

  if (motorNumber == 0)
  {
    posStep = (MotorPos >> 3) & 0xFF;
	
	set_motor(MOT_0_A_OC, pwmSin[posStep]);
	set_motor(MOT_0_B_OC, pwmSin[(posStep + 85) & 0xFF]);
	set_motor(MOT_0_C_OC, pwmSin[(posStep + 170) & 0xFF]);
  }
 
  if (motorNumber == 1)
  {
    posStep = (MotorPos >> 3) & 0xFF;
    
	set_motor(MOT_1_A_OC, pwmSin[posStep]);
	set_motor(MOT_1_B_OC, pwmSin[(posStep + 85) & 0xFF]);
	set_motor(MOT_1_C_OC, pwmSin[(posStep + 170) & 0xFF]);
  }
}



void fastMoveMotor(uint8_t motorNumber, int dirStep,uint8_t* pwmSin)
{
  if (motorNumber == 0)
  {
    currentStepMotor0 += dirStep;
    currentStepMotor0 &= 0xFF;
	
	set_motor(MOT_0_A_OC, pwmSin[currentStepMotor0]);
	set_motor(MOT_0_B_OC, pwmSin[(currentStepMotor0 + 85)  & 0xFF]);
	set_motor(MOT_0_C_OC, pwmSin[(currentStepMotor0 + 170) & 0xFF]);
  }
 
  if (motorNumber == 1)
  {
    currentStepMotor1 += dirStep;
    currentStepMotor1 &= 0xFF;
	//printf("MOTOR: %d\t%d\t%d\t", pwmSin[(uint8_t)currentStepMotor1], pwmSin[(uint8_t)currentStepMotor1 + 85], pwmSin[(uint8_t)currentStepMotor1 + 170]);
	//AAAAAARGH! casting only the vaiable to uint8_t does not make it overflow from 255->0...
	//Need to cast the total sum to uint8_t (or cut off the upper bits...)
	
	//It worked fine for the motor 1 because the overflow went innto the next memory addresses... which were the same sinusoid array over again for the roll motor.
	set_motor(MOT_1_A_OC, pwmSin[currentStepMotor1]);
	set_motor(MOT_1_B_OC, pwmSin[(currentStepMotor1 + 85)  & 0xFF]);
	set_motor(MOT_1_C_OC, pwmSin[(currentStepMotor1 + 170) & 0xFF]);
	//printf("\n");
  }
}

inline void set_motor(_OC *channel, uint8_t value) {
	
	*(channel->OCxR)  = (uint16_t) (255 - (uint16_t)value);
	*(channel->OCxRS) = (uint16_t) (255 + (uint16_t)value);
	//printf("%d\t%d\t", *(channel->OCxR), *(channel->OCxRS));
}

inline void MotorOff(uint8_t motorNumber, uint8_t* pwmSin)
{
  if (motorNumber == 0)
  {
    set_motor(MOT_0_A_OC, pwmSin[0]);
	set_motor(MOT_0_B_OC, pwmSin[0]);
	set_motor(MOT_0_C_OC, pwmSin[0]);
  }
 
  if (motorNumber == 1)
  {
    set_motor(MOT_1_A_OC, pwmSin[0]);
	set_motor(MOT_1_B_OC, pwmSin[0]);
	set_motor(MOT_1_C_OC, pwmSin[0]);
  }
}

void calcSinusArray(uint8_t maxPWM, uint8_t *array)
{
	volatile int i;
	for(i=0; i<N_SIN; i++)
	{
		//    array[i] = maxPWM / 2.0 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
		array[i] = 128 + sin(2.0 * i / N_SIN * 3.14159265) * maxPWM / 2.0;
	}  
}

void recalcMotorStuff()
{
  calcSinusArray(config.maxPWMmotorPitch,pwmSinMotorPitch);
  calcSinusArray(config.maxPWMmotorRoll,pwmSinMotorRoll);
}

void update_motors_callback(_TIMER *self) {
	//printf("UPDATE!\n");
	if (enableMotorUpdates == true) {
		// move pitch motor
		MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, pwmSinMotorPitch); 
		// move roll motor
		MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, pwmSinMotorRoll);
	}
	// update event
	motorUpdate = true;
}

void motorTest() {
	volatile int i;
	#define MOT_DEL 10
	__delay_ms(10 * 32);
	// Move Motors to ensure function
	for(i=0; i<2550; i++) { fastMoveMotor(config.motorNumberPitch, 1,pwmSinMotorPitch); __delay_ms(2);  }
	for(i=0; i<2550; i++) { fastMoveMotor(config.motorNumberPitch, -1,pwmSinMotorPitch); __delay_ms(2);  }
	__delay_ms(200 * 32);
	for(i=0; i<512; i++) { 
		fastMoveMotor(config.motorNumberRoll, 1, pwmSinMotorRoll); 
		__delay_ms(2);  }
	for(i=0; i<2550; i++) { fastMoveMotor(config.motorNumberRoll, -1,pwmSinMotorRoll); __delay_ms(2);  } 
	__delay_ms(1000);
}