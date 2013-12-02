/*************************/
/* EXTERNALS FOR VARS    */
/*************************/
#define RC_DATA_SIZE  2
#define RC_DATA_PITCH 0
#define RC_DATA_ROLL  1

#define MOTORUPDATE_FREQ 1000                // in Hz, 1000 is default // 1,2,4,8 for 32kHz, 1,2,4 for 4kHz
#define LOOPUPDATE_FREQ MOTORUPDATE_FREQ    // loop control sample rate equals motor update rate
#define DT_FLOAT (1.0/LOOPUPDATE_FREQ)      // loop controller sample period dT
#define DT_INT_MS (1000/MOTORUPDATE_FREQ)

typedef struct 
{
uint8_t vers;
uint8_t versEEPROM;
int32_t gyroPitchKp; 
int32_t gyroPitchKi;   
int32_t gyroPitchKd;
int32_t gyroRollKp;
int32_t gyroRollKi;
int32_t gyroRollKd;
int16_t accTimeConstant;
int8_t  mpuLPF;             // mpu LPF 0..6, 0=fastest(256Hz) 6=slowest(5Hz)
int16_t angleOffsetPitch;   // angle offset, deg*100
int16_t angleOffsetRoll;
uint8_t nPolesMotorPitch;
uint8_t nPolesMotorRoll;
int8_t dirMotorPitch;
int8_t dirMotorRoll;
uint8_t motorNumberPitch;
uint8_t motorNumberRoll;
uint8_t maxPWMmotorPitch;
uint8_t maxPWMmotorRoll;
int8_t minRCPitch;
int8_t maxRCPitch;
int8_t minRCRoll;
int8_t maxRCRoll;
int16_t rcGain;
int16_t rcLPF;             // low pass filter for RC absolute mode, units=1/10 sec
bool rcModePPM;            // RC mode, true=common RC PPM channel, false=separate RC channels 
int8_t rcChannelPitch;     // input channel for pitch
int8_t rcChannelRoll;      // input channel for roll
int16_t rcMid;             // rc channel center ms
bool rcAbsolute;
bool accOutput;
bool enableGyro;           // enable gyro attitude update
bool enableACC;            // enable acc attitude update
bool axisReverseZ;
bool axisSwapXY;
uint8_t crc8;
} config_t;

extern config_t config;

extern void recalcMotorStuff();
extern void initPIDs();

extern void setDefaultParameters();

typedef struct PIDdata {
  int32_t   Kp, Ki, Kd;
} PIDdata_t;

extern PIDdata_t pitchPIDpar,rollPIDpar;

extern void initPIDs(void);

typedef uint8_t crc;

/*************************/
/* Variables             */
/*************************/



// motor drive

extern uint8_t pwmSinMotorPitch[256];
extern uint8_t pwmSinMotorRoll[256];

extern int currentStepMotor0;
extern int currentStepMotor1;
extern bool motorUpdate; 

extern int8_t pitchDirection;
extern int8_t rollDirection;

extern int freqCounter;

extern int pitchMotorDrive;
extern int rollMotorDrive;

// control motor update in ISR
extern bool enableMotorUpdates;


// Variables for MPU6050
extern float gyroPitch;
extern float gyroRoll; //in deg/s

extern float resolutionDevider;
extern int16_t x_val;
extern int16_t y_val;
extern int16_t z_val;

extern float PitchPhiSet;
extern float RollPhiSet;

extern int count;

// RC control

typedef struct {
 uint32_t microsRisingEdge;
 uint32_t microsLastUpdate;
 uint16_t rx;
 bool     update;
 bool     valid;
 float    rcSpeed;
 float    setpoint;
} rcData_t;

extern rcData_t rcData[RC_DATA_SIZE];

extern float rcLPF_tc;

// Gimbal State
typedef enum {
 GIM_IDLE=0,      // no PID
 GIM_UNLOCKED,    // PID on, fast ACC
 GIM_LOCKED       // PID on, slow ACC
} gimStateType;

extern gimStateType gimState;
extern int stateCount;


//*************************************
//
//  IMU
//
//*************************************
typedef struct flags_struct {
  uint8_t SMALL_ANGLES_25 : 1;
} flags_t;

extern flags_t flags;

typedef enum {
  ROLL,
  PITCH,
  YAW
} axisDef;

typedef struct fp_vector {
  float X;
  float Y;
  float Z;
} t_fp_vector_def;

typedef union {
  float   A[3];
  t_fp_vector_def V;
} t_fp_vector;



//********************
// sensor orientation
//********************
typedef struct sensorAxisDef {
  char idx;
  int  dir;
} t_sensorAxisDef;

typedef struct sensorOrientationDef {
  t_sensorAxisDef Gyro[3];
  t_sensorAxisDef Acc[3];
} t_sensorOrientationDef;

extern t_sensorOrientationDef sensorDef;

// gyro calibration value
extern int16_t gyroOffset[3];

extern float gyroScale;

extern int32_t accSmooth[3];
extern int16_t gyroADC[3];
extern int16_t accADC[3];

extern t_fp_vector EstG;

extern float accLPF[3];
extern int32_t accMag;

extern float AccComplFilterConst;  // filter constant for complementary filter

extern int16_t acc_25deg;      //** TODO: check

extern int32_t angle[2];  // absolute angle inclination in multiple of 0.01 degree    180 deg = 18000

extern float pitchAngleSet;
extern float rollAngleSet;

// DEBUG only
extern uint32_t stackTop;
extern uint32_t stackBottom;

extern uint32_t heapTop;
extern uint32_t heapBottom;
