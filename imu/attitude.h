/**********************************************
*	header file
*
*	IMU filtering for Olin college PIC24F board
*
*	Adapted from the BruGi project, code.google.com/p/brushless-gimbal
*
*	2013, Steven Cooreman
***********************************************/

/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 40
#endif

#define ACC_1G 16384.0f


void initSensorOrientationDefault();
void swap_char(char * a, char * b);
void swap_int(int * a, int * b);
void initSensorOrientation();
void setACCFastMode (bool fastMode);
void initIMU();
void rotateV(struct fp_vector *v,float* delta);
void readGyros();
void readACC(axisDef axis);
void updateGyroAttitude();
void updateACC();
void updateACCAttitude();
void getAttiduteAngles();
int32_t ComputePID(int32_t, int32_t, int32_t, int32_t*, int32_t*, int32_t, int16_t, int32_t);