/***************************************************************************
*	Elecanisms 2013 Project:
*	Servo control using USB and a Python GUI
*	Geeta, Sarah and Steven
***************************************************************************/
#define MAINFILE

#include <p24FJ128GB206.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "variables.h"
#include "config.h"
#include "common.h"
#include <libpic30.h>
#include "pin.h"
#include "uart.h"
#include "spi.h"
#include "ui.h"
#include "oc.h"
#include "timer.h"

#include <stdio.h>
#include "imu.h"
#include "brushless.h"
#include "attitude.h"
#include "serialcommands.h"


#define HELLO       0   // Vendor request that prints "Hello World!"
#define SET_VALS    1   // Vendor request that receives 2 unsigned integer values
#define GET_VALS    2   // Vendor request that returns 2 unsigned integer values
#define PRINT_VALS  3   // Vendor request that prints 2 unsigned integer values 

#define LED_TIM     &timer3
#define LED         &led1

#define FREQ_PIN	&D[8]

#define POUT_FREQ 25     // rate of ACC print output in Hz, 25 Hz is default
#define LOCK_TIME_SEC 5  // gimbal fast lock time at startup 

uint8_t g_xout_l;
uint8_t g_xout_h;
uint8_t a_xout;
uint8_t g_name;

volatile uint32_t i;
volatile int accelval[3];
int16_t	gyrovals[3];
int16_t accelvals[3];


#include "orientationRoutines.h"

char* inttobinarystr(int16_t number) {
	static char string[17];
	uint8_t i = 0;
	for(i = 16; i > 0; i--) {
		string[16-i] = 0x30 + ((number >> (i-1)) & 1);
	}
	string[16] = 0;
	return string;
}


int16_t main(void) {
	int32_t pitchPIDVal;
	int32_t rollPIDVal;
	static int32_t pitchErrorSum;
	static int32_t rollErrorSum;
	static int32_t pitchErrorOld;
	static int32_t rollErrorOld;

	static char pOutCnt = 0;
	static int stateCount = 0;
    //initialize all system clocks
    init_clock();
    //initialize serial communications
    init_uart();
    //initialize pin driving library (to be able to use the &D[x] defs)
    init_pin();
    //initialize the UI library
    init_ui();
    //initialize the timer module
    init_timer();
    //initialize the OC module (used by the servo driving code)
    init_oc();
	//Initialise the SPI for the IMU
	init_spi();

	//printf("initializing IMU\n");
    //initialize imu module
    imu_init();

    //Set LED off
    led_off(LED);
	//Set main loop frequency check pin
	pin_digitalOut(FREQ_PIN);
	pin_clear(FREQ_PIN);
    //Configure blinking rate for LED when connected
    timer_setPeriod(LED_TIM, 0.2);
    timer_start(LED_TIM);
	
	//printf("Gyro ID: %d, Acc ID: %d\n", gyro_read(WHO_AM_I), accel_read(I2CADD));
	accel_set_measure_mode();
	gyro_set_measure_mode();
	
	
	//Re-calibrate the accelerometer, even after soft reset
		accelval[0] = 0;
		accelval[1] = 0;
		accelval[2] = 0;
	accel_calibrate((int *)accelval);
	
	//		GYRO CALIBRATION HAS BEEN DONE BY HAND
	accel_read_xyz((int *)accelval);
		// CALIBRATION VALUES HAND-CALCULATED
		accelval[0] = -accelval[0];
		accelval[1] = -accelval[1];
		accelval[2] = 64-accelval[2];
	accel_calibrate((int *)accelval);
	
	
	
	//printf("Starting motor init sequence...\n");
	setDefaultParameters();
	recalcMotorStuff();
	setupMotors();
	//motorTest();
	
	
	
	initResolutionDevider();
	initIMU();
	//printf("Setting orientation...\n");
	initSensorOrientation();
	//printf("Calibrating Gyro...\n");
	gyroOffsetCalibration();
	initPIDs();
	
    //printf("starting main loop\n");
	timer_setPeriod(&timer5, 1.0f);
    while (1) {
        
		accel_read_xyz((int *)accelval);
		//printf("%d,\t%d,\t%d\n", accelval[0], accelval[1], accelval[2]);
		
        //blink the LED
        if (timer_flag(LED_TIM)) {
            timer_lower(LED_TIM);
            led_toggle(LED);
        }
		
		//check for 1000hz update
		if(timer_flag(MOT_UPD_TIM)) {
		
			timer_lower(MOT_UPD_TIM);			
			
			if (enableMotorUpdates == true) {
			// move pitch motor
			MoveMotorPosSpeed(config.motorNumberPitch, pitchMotorDrive, pwmSinMotorPitch); 
			// move roll motor
			MoveMotorPosSpeed(config.motorNumberRoll, rollMotorDrive, pwmSinMotorRoll);
			}
			// update event
			motorUpdate = true;
		}
	
        
        //Read from gyro
        //gyro_get_measurements(&(gyrovals[0]), &(gyrovals[1]), &(gyrovals[2]));
		//accel_get_measurements(&(accelvals[0]), &(accelvals[1]), &(accelvals[2]));
		
		//printf("Gyro X: %s\n", inttobinarystr(gyrovals[0]));
		//printf("Accel: %d,%d, %d\n", accelvals[0], accelvals[1], accelvals[2]);
		//__delay_ms(200);
		
		if (motorUpdate == true) // loop runs with motor ISR update rate (1000Hz)
		{
			pin_toggle(FREQ_PIN);
			timer_start(&timer5);
			motorUpdate = false;

			// update IMU data            
			readGyros();   // t=386us

			if (config.enableGyro) updateGyroAttitude(); // t=260us
			if (config.enableACC) updateACCAttitude(); // t=146us

			getAttiduteAngles(); // t=468us

			//****************************
			// pitch PID
			//****************************

			// t=94us
			pitchPIDVal = ComputePID(DT_INT_MS, angle[PITCH], pitchAngleSet*1000, &pitchErrorSum, &pitchErrorOld, pitchPIDpar.Kp, pitchPIDpar.Ki, pitchPIDpar.Kd);
			// motor control
			pitchMotorDrive = pitchPIDVal * config.dirMotorPitch;


			//****************************
			// roll PID
			//****************************
			// t=94us
			rollPIDVal = ComputePID(DT_INT_MS, angle[ROLL], rollAngleSet*1000, &rollErrorSum, &rollErrorOld, rollPIDpar.Kp, rollPIDpar.Ki, rollPIDpar.Kd);

			// motor control
			rollMotorDrive = rollPIDVal * config.dirMotorRoll;
			
			//printf("PITCH %ld\t", pitchMotorDrive);
			//printf("ROLL %ld\n", rollMotorDrive);
			//****************************
			// slow rate actions
			//****************************
			switch (count) {
			case 1:
			  readACC(ROLL); break;
			case 2:
			  readACC(PITCH); break;
			case 3:
			  readACC(YAW); break;
			case 4:
			  updateACC(); break;
			case 5:
			  break;
			case 6:
			  // gimbal state transitions 
			  switch (gimState)
			  {
				case GIM_IDLE :
				  // wait 2 sec to settle ACC, before PID controlerbecomes active 
				  stateCount++;
				  if (stateCount >= LOOPUPDATE_FREQ/10*2)  
				  {
					printf("gimbal unlocked!\n");
					gimState = GIM_UNLOCKED;
					stateCount = 0;
				  }
				  break;
				case GIM_UNLOCKED :
				  // allow PID controller to settle on ACC position
				  stateCount++;
				  if (stateCount >= LOOPUPDATE_FREQ/10*LOCK_TIME_SEC) 
				  {
					printf("gimbal LOCKED ON!\n");
					gimState = GIM_LOCKED;
					stateCount = 0;
				  }
				  break;
				case GIM_LOCKED :
				  // normal operation
				  break;
			  }
			  // gimbal state actions 
			  switch (gimState) {
				case GIM_IDLE :
				  enableMotorUpdates = false;
				  setACCFastMode(true);
				  break;
				case GIM_UNLOCKED :
				  enableMotorUpdates = true;
				  setACCFastMode(true);
				  break;
				case GIM_LOCKED :
				  enableMotorUpdates = true;
				  setACCFastMode(false);
				  break;
			  }
			  break;
			case 7:
			  // RC Pitch function
			  break;
			case 8:
			  break;
			case 9:
			  // regular ACC output
			  pOutCnt++;
			  if (pOutCnt == (LOOPUPDATE_FREQ/10/POUT_FREQ))
			  {
				// 600 us
				if(config.accOutput==true){ 
				
					printf("%ld", angle[PITCH]); printf(" ACC ");printf("%ld\n", angle[ROLL]); //printf("%d\t%d\t%d\t%d\n", accADC[PITCH], accADC[ROLL], gyroADC[PITCH], gyroADC[ROLL]);
					
				}
				pOutCnt = 0;
			  }
			  break;
			case 10:    
			  count=0;
			  break;
			default:
			  break;
			}
			count++;
			timer_stop(&timer5);
			//printf("this cycle took %d us\n", timer_read(&timer5) * 16);
			//****************************
			// check RC channel timeouts
			//****************************

			//checkRcTimeouts();

			//****************************
			// Evaluate Serial inputs 
			//****************************
			serial_readSerial();
		}
    }
}
