/**********************************************
*	HEADER FILE
*
*	serial commands for PIC24 Olin College board
*
*	Adapted from the BruGi project, code.google.com/p/brushless-gimbal
*
*	2013, Steven Cooreman
***********************************************/

//************************************************************************************
// Serial port access routines
//************************************************************************************

// Size of the input buffer in bytes (maximum length of one command plus arguments)
#define SERIALCOMMAND_BUFFER 32
// Maximum length of a command including the terminating null
#define SERIALCOMMAND_MAXCOMMANDLENGTH 4
// Maximum amount of commands we're going to store
#define COMMAND_COUNT 30

void serial_addCommand(const char *command, void(*function)());  // Add a command to the processing dictionary.
void serial_setDefaultHandler(void (*function)(const char *));   // A handler to call when no valid command received.

void serial_readSerial();    // Main entry point.
inline void serial_clearBuffer();   // Clears the input buffer.
char *serial_next();         // Returns pointer to next token found in command buffer (for getting arguments to commands).

// Command/handler dictionary
typedef struct SerialCommandCallback {
  char command[SERIALCOMMAND_MAXCOMMANDLENGTH + 1];
  void (*function)();
} SerialCommandCallback_t;            // Data structure to hold Command/Handler function key-value pairs




//************************************************************************************
// general config parameter access routines
//************************************************************************************

// types of config parameters
typedef enum {
  BOOL,
  INT8,
  INT16,
  INT32,
  UINT8,
  UINT16,
  UINT32
} confType_t;

#define CONFIGNAME_MAX_LEN 17
typedef struct configDef {
  char name[CONFIGNAME_MAX_LEN];  // name of config parameter
  confType_t type;                  // type of config parameters
  void * address;                 // address of config parameter
  void (* updateFunction)(void);  // function is called when parameter update happens
} t_configDef;



// access decriptor as array of bytes as well
typedef union {
  t_configDef   c;
  char          bytes[sizeof(t_configDef)];
} t_configUnion;




// find Config Definition for named parameter
t_configDef * getConfigDef(char * name);
// print single parameter value
void printConfig(t_configDef * def);
// write single parameter with value
void writeConfig(t_configDef * def, int32_t val);
// print all parameters
void printConfigAll(t_configDef * p);
//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod();

void updateAllParameters();

void setDefaultParametersAndUpdate();


void transmitUseACC();
void toggleACCOutput();

void setUseACC();

void transmitRCConfig();

void transmitRCAbsolute();
void setRCGain();
void transmitRCGain();
void setRcMode();
void transmitRcMode();
void setRCAbsolute();
void setRCConfig();
void transmitSensorOrientation();
void writeEEPROM();
void readEEPROM();
void transmitActiveConfig();
void setPitchPID();
void setRollPID();
void setMotorPWM();
void gyroRecalibrate();
void setMotorDirNo();
void setSensorOrientation();
void printHelpUsage();
void unrecognized(const char *command) ;
void setSerialProtocol();