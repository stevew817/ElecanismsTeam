/**********************************************
*	header file
*
*	Brushless motor timing functions for PIC24 Olin College board
*
*	Adapted from the BruGi project, code.google.com/p/brushless-gimbal
*
*	2013, Steven Cooreman
***********************************************/

//MOTOR TIMER
#define MOT_TIM			(&timer1)
#define MOT_UPD_TIM		(&timer2)

//MOTOR OUTPUT OC's
#define MOT_0_A_OC		(&oc1)
#define MOT_0_B_OC		(&oc2)
#define MOT_0_C_OC		(&oc3)
#define MOT_1_A_OC		(&oc4)
#define MOT_1_B_OC		(&oc5)
#define MOT_1_C_OC		(&oc6)
#define LOOP_OC			(&oc9)

//MOTOR OUTPUT PINS
#define MOT_0_A_PIN		&D[1]
#define MOT_0_B_PIN		&D[0]
#define MOT_0_C_PIN		&D[2]
#define MOT_1_A_PIN		&D[10]
#define MOT_1_B_PIN		&D[11]
#define MOT_1_C_PIN		&D[12]

#define LOOP_PIN		&D[4]

// Number of sinus values for full 360 deg.
// NOW FIXED TO 256 !!!
// Reason: Fast Motor Routine using uint8_t overflow for stepping
#define N_SIN 256

#define CC_FACTOR	32						// how many PWM cycles per update loop
#define MOTORUPDATE_FREQ 1000                // in Hz, 1000 is default // 1,2,4,8 for 32kHz, 1,2,4 for 4kHz
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ    // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ)      // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ) 

void setupMotors(void);
inline void MoveMotorPosSpeed(uint8_t motorNumber, int MotorPos, uint8_t* pwmSin);
void fastMoveMotor(uint8_t motorNumber, int dirStep,uint8_t* pwmSin);
inline void set_motor(_OC *channel, uint8_t value);
inline void MotorOff(uint8_t motorNumber, uint8_t* pwmSin);
void calcSinusArray(uint8_t maxPWM, uint8_t *array);
void recalcMotorStuff();
void update_motors_callback(_TIMER *self);
void motorTest();