/**********************************************
*
*	Gimbal serial commands for PIC24 Olin College board
*
*	Adapted from the BruGi project, code.google.com/p/brushless-gimbal
*
*	2013, Steven Cooreman
***********************************************/

#include <p24FJ128GB206.h>
#include <math.h>
#include <stdint.h>
#include "common.h"
#include <libpic30.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "oc.h"
#include "timer.h"
#include <stdbool.h>
#include "externvariables.h"
#include "serialcommands.h"
#include <string.h>
#include "uart.h"

//#define SERIALCOMMAND_DEBUG

extern void recalcMotorStuff();
extern void initPIDs();
extern void initIMU();
//extern void initMPUlpf();
extern void initSensorOrientation();
extern void setDefaultParameters();
extern void updateAllParameters();
extern void gyroOffsetCalibration();


// list of all config parameters
// to be accessed by par command
const t_configDef configListPGM[] = {
  {"vers",             UINT8, &config.vers,             NULL},

  {"gyroPitchKp",      INT32, &config.gyroPitchKp,      &initPIDs},
  {"gyroPitchKi",      INT32, &config.gyroPitchKi,      &initPIDs},
  {"gyroPitchKd",      INT32, &config.gyroPitchKd,      &initPIDs},
  {"gyroRollKp",       INT32, &config.gyroRollKp,       &initPIDs},
  {"gyroRollKi",       INT32, &config.gyroRollKi,       &initPIDs},
  {"gyroRollKd",       INT32, &config.gyroRollKd,       &initPIDs},
  {"accTimeConstant",  INT16, &config.accTimeConstant,  &initIMU},
  {"mpuLPF",           INT8,  &config.mpuLPF,           NULL}, //initMPUlpf
  
  {"angleOffsetPitch", INT16, &config.angleOffsetPitch, NULL},
  {"angleOffsetRoll",  INT16, &config.angleOffsetRoll,  NULL},
  
  {"dirMotorPitch",    INT8,  &config.dirMotorPitch,    NULL},
  {"dirMotorRoll",     INT8,  &config.dirMotorRoll,     NULL},
  {"motorNumberPitch", UINT8, &config.motorNumberPitch, NULL},
  {"motorNumberRoll",  UINT8, &config.motorNumberRoll,  NULL},
  {"maxPWMmotorPitch", UINT8, &config.maxPWMmotorPitch, &recalcMotorStuff},
  {"maxPWMmotorRoll",  UINT8, &config.maxPWMmotorRoll,  &recalcMotorStuff},

  {"minRCPitch",       INT8,  &config.minRCPitch,        NULL},
  {"maxRCPitch",       INT8,  &config.maxRCPitch,        NULL},
  {"minRCRoll",        INT8,  &config.minRCRoll,         NULL},
  {"maxRCRoll",        INT8,  &config.maxRCRoll,         NULL},
  {"rcGain",           INT16, &config.rcGain,            NULL},
  {"rcLPF",            INT16, &config.rcLPF,             NULL},

  {"rcModePPM",        BOOL,  &config.rcModePPM,         NULL},
  {"rcChannelPitch",   INT8,  &config.rcChannelPitch,    NULL},
  {"rcChannelRoll",    INT8,  &config.rcChannelRoll,     NULL},
  {"rcMid",            INT16, &config.rcMid,             NULL},
  {"rcAbsolute",       BOOL,  &config.rcAbsolute,        NULL},
  
  {"accOutput",        BOOL,  &config.accOutput,         NULL},

  {"enableGyro",       BOOL,  &config.enableGyro,        NULL},
  {"enableACC",        BOOL,  &config.enableACC,         NULL},

  {"axisReverseZ",     BOOL,  &config.axisReverseZ,      &initSensorOrientation},
  {"axisSwapXY",       BOOL,  &config.axisSwapXY,        &initSensorOrientation},
  
  {"", BOOL, NULL, NULL} // terminating NULL required !!
};


// ******************************** SERIAL PORT HANDLING PROCEDURES *************************************

SerialCommandCallback_t commandList[COMMAND_COUNT];   // Actual definition for command/handler array
uint8_t commandCount = 0;

char delim[2] = {' ', '\0'}; // null-terminated list of character to be used as delimeters for tokenizing (default " ")
const char term1 = '\n';     // Character that signals end of command (default '\n')
const char term2 = '\r';

char buffer[SERIALCOMMAND_BUFFER + 1] = {'\0', 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; // Buffer of stored characters while waiting for terminator character
uint8_t bufPos = 0;                        // Current position in the buffer
char *last = NULL;                         // State variable used by strtok_r during processing

// Pointer to the default handler function
void (*defaultHandler)(const char *) = NULL;


/**
 * Adds a "command" and a handler function to the list of available commands.
 * This is used for matching a found token in the buffer, and gives the pointer
 * to the handler function to deal with it.
 */
void serial_addCommand(const char *command, void (*function)()) {
  #ifdef SERIALCOMMAND_DEBUG
    printf("Adding command (%d): %s\n", commandCount, command);
  #endif

  //no dynamic memory for us...
  //commandList = (SerialCommandCallback *) realloc(commandList, (commandCount + 1) * sizeof(SerialCommandCallback));
  strncpy(commandList[commandCount].command, command, SERIALCOMMAND_MAXCOMMANDLENGTH);
  commandList[commandCount].function = function;
  commandCount++;
}

/**
 * This sets up a handler to be called in the event that the received command string
 * isn't in the list of commands.
 */
void serial_setDefaultHandler(void (*function)(const char *)) {
  defaultHandler = function;
}

/**
 * Clear the input buffer.
 */
inline void serial_clearBuffer() {
  buffer[0] = '\0';
  bufPos = 0;
}

/**
 * Retrieve the next token ("word" or "argument") from the command buffer.
 * Returns NULL if no more tokens exist.
 */
char * serial_next() {
  return strtok(NULL, delim);
}

/**
 * This checks the Serial stream for characters, and assembles them into a buffer.
 * When the terminator character (default '\n') is seen, it starts parsing the
 * buffer for a prefix command, and calls handlers setup by addCommand() member
 */
void serial_readSerial() {
  int i = 0, j = 0;
  char * command = NULL;
  bool matched = false;
  
  while (uart1.RXbuffer.count > 0) {
    char inChar = uart_getc(&uart1);   // Read single available character, there may be more waiting
    #ifdef SERIALCOMMAND_DEBUG
      uart_putc(&uart1, inChar);   // Echo back to serial stream
    #endif

    if ((inChar == term1) || (inChar == term2)) {     // Check for the terminator (default '\r') meaning end of command
      #ifdef SERIALCOMMAND_DEBUG
        printf("Received: %s\n", buffer);
      #endif

      command = strtok(buffer, delim);   // Search for command at start of buffer
      if (command != NULL) {
        matched = false;
        for (i = 0; i < commandCount; i++) {
          #ifdef SERIALCOMMAND_DEBUG
            printf("Comparing [%s] to [%s]\n", command, commandList[i].command);
          #endif

          // Compare the found command against the list of known commands for a match
          for (j = 0; command[j] != '\0'; j++)   // as no strnicmp exists ...
            command[j] = (char)tolower(command[j]);
          if (strncmp(command, commandList[i].command, SERIALCOMMAND_MAXCOMMANDLENGTH) == 0) {
            #ifdef SERIALCOMMAND_DEBUG
              printf("Matched Command: %s\n", command);
            #endif

            // Execute the stored handler function for the command
            (*commandList[i].function)();
            matched = true;
            break;
          }
        }
        if (!matched && (defaultHandler != NULL)) {
          (*defaultHandler)(command);
        }
      }
      // Serial.println(F("BruGi> ")); // TODO: BruGi prompt string 
      serial_clearBuffer();
    }
    else if (isprint(inChar)) {     // Only printable characters into the buffer
      if (bufPos < SERIALCOMMAND_BUFFER) {
        buffer[bufPos++] = inChar;  // Put character into buffer
        buffer[bufPos] = '\0';      // Null terminate
      } else {
        #ifdef SERIALCOMMAND_DEBUG
          printf("Line buffer is full - increase SERIALCOMMAND_BUFFER\n");
        #endif
		
		//Avoid a buffer crash
		serial_clearBuffer();
      }
    }
  }
}



// ******************************** COMMAND HANDLING PROCEDURES *****************************************
// find Config Definition for named parameter

t_configDef configDef;
t_configUnion configUnion;

t_configDef * getConfigDef(char * name) {

  bool found = false; 
  unsigned char idx = 0;
  t_configDef * p = (t_configDef *)configListPGM;

  while (true) {
	p = &(configListPGM[idx]);
    if (p->address == NULL) break;
    if (strncmp(p->name, name, CONFIGNAME_MAX_LEN) == 0) {
      found = true;
      break;
   }
   idx++; 
  }
  if (found) 
      return p;
  else 
      return NULL;
}


// print single parameter value
void printConfig(t_configDef * def) {
  if (def != NULL) {
    printf("%s", def->name);
    printf(" ");
    switch (def->type) {
      case BOOL   : printf("%d", *(bool *)(def->address)); break;
      case UINT8  : printf("%u", *(uint8_t *)(def->address)); break;
      case UINT16 : printf("%u", *(uint16_t *)(def->address)); break;
      case UINT32 : printf("%lu", *(uint32_t *)(def->address)); break;
      case INT8   : printf("%d", *(int8_t *)(def->address)); break;
      case INT16  : printf("%d", *(int16_t *)(def->address)); break;
      case INT32  : printf("%ld", *(int32_t *)(def->address)); break;
    }
    printf("\n");
  } else {
    printf("ERROR: illegal parameter\n");    
  }
}

// write single parameter with value
void writeConfig(t_configDef * def, int32_t val) {
  if (def != NULL) {
    switch (def->type) {
      case BOOL   : *(bool *)(def->address)     = val; break;
      case UINT8  : *(uint8_t *)(def->address)  = val; break;
      case UINT16 : *(uint16_t *)(def->address) = val; break;
      case UINT32 : *(uint32_t *)(def->address) = val; break;
      case INT8   : *(int8_t *)(def->address)   = val; break;
      case INT16  : *(int16_t *)(def->address)  = val; break;
      case INT32  : *(int32_t *)(def->address)  = val; break;
    }
    // call update function
    if (def->updateFunction != NULL) def->updateFunction();
  } else {
    printf("ERROR: illegal parameter\n");    
  }
}


// print all parameters
void printConfigAll(t_configDef * p) {
  unsigned char idx = 0;
  while (true) {
    if (p[idx].address == NULL) break;
    printConfig(&(p[idx]));
    idx++; 
  }
  printf("done.\n");
}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod() {

  char * paraName = NULL;
  char * paraValue = NULL;
  
  int32_t val = 0;

  if ((paraName = serial_next()) == NULL) {
    // no command parameter, print all config parameters
    printConfigAll((t_configDef *)configListPGM);
  } else if ((paraValue = serial_next()) == NULL) {
    // one parameter, print single parameter
    printConfig(getConfigDef(paraName));
  } else {
    // two parameters, set specified parameter
    val = atol(paraValue);
	#ifdef SERIALCOMMAND_DEBUG
         printf("Setting %s to %ld\n", paraName, val);
    #endif
    writeConfig(getConfigDef(paraName), val);
  }
}
//************************************************************************************


void updateAllParameters() {
  recalcMotorStuff();
  initPIDs();
  initIMU();
  //initMPUlpf();
  initSensorOrientation();
  //initRCPins();
  //initRC();
}

void setDefaultParametersAndUpdate() {
  setDefaultParameters();
  updateAllParameters();
}


void transmitUseACC()  // TODO: remove obsolete command
{
   printf("1\n");  // dummy for bl_tool compatibility ;-)
}

void toggleACCOutput()
{
  int temp = atoi(serial_next());
  if(temp==1)
    config.accOutput = true;
  else
    config.accOutput = false;
}

void setUseACC() // TODO: remove obsolete command
{
  int temp = atoi(serial_next());
}

void transmitRCConfig()
{
  printf("%d\n", config.minRCPitch);
  printf("%d\n", config.maxRCPitch);
  printf("%d\n", config.minRCRoll);
  printf("%d\n", config.maxRCRoll);
}

void transmitRCAbsolute()
{
  printf("%d\n", config.rcAbsolute);
}

void setRCGain()
{
    config.rcGain = atoi(serial_next());
}

void transmitRCGain()
{
  printf("%d\n", config.rcGain);
}

void setRcMode()
{
    config.rcModePPM = atoi(serial_next());
    config.rcChannelPitch = atoi(serial_next());
    config.rcChannelRoll = atoi(serial_next());
    //initRCPins();
}

void transmitRcMode()
{
  printf("%d\n", config.rcModePPM);
  printf("%d\n", config.rcChannelPitch);
  printf("%d\n", config.rcChannelRoll);
}

void setRCAbsolute()
{
  int temp = atoi(serial_next());
  if(temp==1)
  {
    config.rcAbsolute = true;
  }
  else
  {
    config.rcAbsolute = false;
  }
  rcData[RC_DATA_PITCH].setpoint = 0.0;
  rcData[RC_DATA_ROLL].setpoint  = 0.0;
  rcData[RC_DATA_PITCH].rcSpeed  = 0.0;
  rcData[RC_DATA_ROLL].rcSpeed   = 0.0;
}

void setRCConfig()
{
  config.minRCPitch = atoi(serial_next());
  config.maxRCPitch = atoi(serial_next());
  config.minRCRoll = atoi(serial_next());
  config.maxRCRoll = atoi(serial_next());
}

void transmitSensorOrientation()
{
  printf("%d\n", config.axisReverseZ);
  printf("%d\n", config.axisSwapXY);
}

void writeEEPROM()
{
  /*
  bool old = config.accOutput;
  config.accOutput = false; // do not save enabled OAC output mode 
  config.crc8 = crcSlow((crc *)&config, sizeof(config)-1); // set proper CRC 
  EEPROM_writeAnything(0, config);
  config.accOutput = old; 
  */
  
}

void readEEPROM()
{
/*
  EEPROM_readAnything(0, config); 
  if (config.crc8 == crcSlow((crc *)&config, sizeof(config)-1))
  { 
    updateAllParameters();
  } else {
    // crc failed intialize directly here, as readEEPROM is void
    Serial.print(F("EEPROM CRC failed, initialize EEPROM"));
    setDefaultParameters();
    writeEEPROM();
  }
  */
}

void transmitActiveConfig()
{
  printf("%d\n", config.vers);
  printf("%ld\n", config.gyroPitchKp);
  printf("%ld\n", config.gyroPitchKi);
  printf("%ld\n", config.gyroPitchKd);
  printf("%ld\n", config.gyroRollKp);
  printf("%ld\n", config.gyroRollKi);
  printf("%ld\n", config.gyroRollKd);
  printf("%d\n", config.accTimeConstant);
  printf("%u\n", config.nPolesMotorPitch);
  printf("%u\n", config.nPolesMotorRoll);
  printf("%d\n", config.dirMotorPitch);
  printf("%d\n", config.dirMotorRoll);
  printf("%u\n", config.motorNumberPitch);
  printf("%u\n", config.motorNumberRoll);
  printf("%u\n", config.maxPWMmotorPitch);
  printf("%u\n", config.maxPWMmotorRoll);
}


void setPitchPID()
{
  config.gyroPitchKp = atol(serial_next());
  config.gyroPitchKi = atol(serial_next());
  config.gyroPitchKd = atol(serial_next());
  initPIDs();
}

void setRollPID()
{
  config.gyroRollKp = atol(serial_next());
  config.gyroRollKi = atol(serial_next());
  config.gyroRollKd = atol(serial_next());
  initPIDs();
}

void setMotorPWM()
{
  config.maxPWMmotorPitch = atoi(serial_next());
  config.maxPWMmotorRoll = atoi(serial_next());
  recalcMotorStuff();
}

void gyroRecalibrate()
{
  //mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
  gyroOffsetCalibration();
  //initMPUlpf();
  printf("recalibration: done\n");
}

void setMotorDirNo()
{
  config.dirMotorPitch = atoi(serial_next());
  config.dirMotorRoll = atoi(serial_next());
  config.motorNumberPitch = atoi(serial_next());
  config.motorNumberRoll = atoi(serial_next());
}

void setSensorOrientation()
{
  config.axisReverseZ = atoi(serial_next());
  config.axisSwapXY = atoi(serial_next());

  initSensorOrientation();
  
}

void printHelpUsage()
{
  printf("This gives you a list of all commands with usage:\n");
  printf("Explanations are in brackets(), use integer values only !\n\n");
  //printf("\n");
  printf("these are the preferred commands, use them for new GUIs !!\n\n");
  //printf("\n");
  printf("SD    (Set Defaults)\n");
  printf("WE    (Writes active config to eeprom)\n");
  printf("RE    (Restores values from eeprom to active config)\n");  
  printf("GC    (Recalibrates the Gyro Offsets)\n");
  printf("par <parName> <parValue>   (general parameter read/set command)\n");
  printf("    example usage:\n");
  printf("       par                     ... list all config parameters\n");
  printf("       par gyroPitchKi         ... list gyroPitchKi\n");
  printf("       par gyroPitchKi 12000   ... set gyroPitchKi to 12000\n\n");
  //printf("\n");
  printf("these commands are intendend for commandline users and compatibilty with 049 GUI\n");
  printf("TC    (transmits all config values in eeprom save order)\n");     
  printf("SP gyroPitchKp gyroPitchKi gyroPitchKd    (Set PID for Pitch)\n");
  printf("SR gyroRollKp gyroRollKi gyroRollKd    (Set PID for Roll)\n");
  printf("SE maxPWMmotorPitch maxPWMmotorRoll     (Used for Power limitiation on each motor 255=high, 1=low)\n");
  printf("SM dirMotorPitch dirMotorRoll motorNumberPitch motorNumberRoll\n");
  printf("SSO reverseZ swapXY (set sensor orientation)\n");
  printf("TSO   (Transmit sensor orientation)\n");
  printf("TRC   (transmitts RC Config)\n");
  printf("SRC minRCPitch maxRCPitch minRCRoll maxRCRoll (angles -90..90)\n");
  printf("SCA rcAbsolute (1 = true, RC control is absolute; 0 = false, RC control is proportional)\n");
  printf("SRG rcGain (set RC gain)\n");
  printf("SRM modePPM channelPitch channelRoll (set RC mode: modePPM 1=PPM 0=single channels, channelPitch/Roll = channel assignment 0..7)\n");
  printf("TCA   (Transmit RC control absolute or not)\n");
  printf("TRG   (Transmit RC gain)\n");
  printf("TRM   (Transmit RC mode\n");
  printf("UAC useACC (1 = true, ACC; 0 = false, DMP)\n");
  printf("TAC   (Transmit ACC status)\n");
  printf("OAC accOutput (Toggle Angle output in ACC mode: 1 = true, 0 = false)\n\n");
  //printf("\n");
  printf("HE     (print this output)\n\n");
  //printf("\n");
  printf("Note: command input is case-insensitive, commands are accepted in both upper/lower case\n");
}

void unrecognized(const char *command) 
{
  printf("What? type in HE for Help ...\n");
}


void setSerialProtocol()
{
  // Setup callbacks for SerialCommand commands
  serial_addCommand("sd", setDefaultParametersAndUpdate);   
  serial_addCommand("we", writeEEPROM);   
  serial_addCommand("re", readEEPROM); 
  serial_addCommand("par", parameterMod);
  serial_addCommand("gc", gyroRecalibrate);

  serial_addCommand("tc", transmitActiveConfig);
  serial_addCommand("sp", setPitchPID);
  serial_addCommand("sr", setRollPID);
  serial_addCommand("se", setMotorPWM);
  serial_addCommand("sm", setMotorDirNo);
  serial_addCommand("sso", setSensorOrientation);
  serial_addCommand("tso", transmitSensorOrientation);
  serial_addCommand("trc", transmitRCConfig);
  serial_addCommand("src", setRCConfig);
  serial_addCommand("srg", setRCGain);
  serial_addCommand("srm", setRcMode);  
  serial_addCommand("trm", transmitRcMode);  
  serial_addCommand("sca", setRCAbsolute);
  serial_addCommand("tca", transmitRCAbsolute);
  serial_addCommand("trg", transmitRCGain);
  serial_addCommand("uac", setUseACC);
  serial_addCommand("tac", transmitUseACC);
  serial_addCommand("oac", toggleACCOutput);

  serial_addCommand("he", printHelpUsage);
  serial_setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}