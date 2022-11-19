// ConsoleCommands.c
// This is where you add commands:
//      1. Add a protoype
//          static eCommandResult_T ConsoleCommandVer(const char buffer[]);
//      2. Add the command to mConsoleCommandTable
//          {"ver", &ConsoleCommandVer, HELP("Get the version string")},
//      3. Implement the function, using ConsoleReceiveParam<Type> to get the parameters from the buffer.

#include <string.h>
#include "consoleCommands.h"
#include "console.h"
#include "consoleIo.h"
#include "version.h"
#include "filters.h"

extern uint8_t DISPLAY_IMU;
extern uint8_t DISPLAY_SEMG;
extern uint8_t  WINPLOTTER;
static eCommandResult_T ConsoleCommandComment(const char buffer[]);
static eCommandResult_T ConsoleCommandVer(const char buffer[]);
static eCommandResult_T ConsoleCommandHelp(const char buffer[]);
static eCommandResult_T ConsoleCommandParamExampleInt16(const char buffer[]);
static eCommandResult_T ConsoleCommandParamExampleHexUint16(const char buffer[]);
static eCommandResult_T ConsoleCommandFilter(const char buffer[]);
static eCommandResult_T ConsoleCommandImuToggleData(const char buffer[]);
static eCommandResult_T ConsoleCommandSemgToggleData(const char buffer[]);
static eCommandResult_T ConsoleCommandWinPlotToggle(const char buffer[]);
static const sConsoleCommandTable_T mConsoleCommandTable[] =
{
    {";", &ConsoleCommandComment, HELP("Comment! You do need a space after the semicolon. ")},
    {"help", &ConsoleCommandHelp, HELP("Lists the commands available")},
    {"ver", &ConsoleCommandVer, HELP("Get the version string")},
    {"int", &ConsoleCommandParamExampleInt16, HELP("How to get a signed int16 from params list: int -321")},
    {"u16h", &ConsoleCommandParamExampleHexUint16, HELP("How to get a hex u16 from the params list: u16h aB12")},
    {"filter", &ConsoleCommandFilter, HELP("Filter sensor data. Type 'filter help'.")},
    {"imu_toggle_data", &ConsoleCommandImuToggleData, HELP("Toggles data display for IMU")},
    {"semg_toggle_data", &ConsoleCommandSemgToggleData, HELP("Toggles data display for SEMG")},
    {"WINPLOT_toggle_data", &ConsoleCommandWinPlotToggle, HELP("Toggles data output format(winPlot")},

    CONSOLE_COMMAND_TABLE_END // must be LAST
};

static eCommandResult_T ConsoleCommandComment(const char buffer[])
{
    // do nothing
    IGNORE_UNUSED_VARIABLE(buffer);
    return COMMAND_SUCCESS;
}

static eCommandResult_T ConsoleCommandHelp(const char buffer[])
{
    uint32_t i;
    uint32_t tableLength;
    eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

    tableLength = sizeof(mConsoleCommandTable) / sizeof(mConsoleCommandTable[0]);
    for ( i = 0u ; i < tableLength - 1u ; i++ )
    {
        ConsoleIoSendString(mConsoleCommandTable[i].name);
#if CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
        ConsoleIoSendString(" : ");
        ConsoleIoSendString(mConsoleCommandTable[i].help);
#endif // CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
        ConsoleIoSendString(STR_ENDLINE);
    }
    return result;
}

static eCommandResult_T ConsoleCommandParamExampleInt16(const char buffer[])
{
    int16_t parameterInt;
    eCommandResult_T result;
    result = ConsoleReceiveParamInt16(buffer, 1, &parameterInt);
    if ( COMMAND_SUCCESS == result )
    {
        ConsoleIoSendString("Parameter is ");
        ConsoleSendParamInt16(parameterInt);
        ConsoleIoSendString(" (0x");
        ConsoleSendParamHexUint16((uint16_t)parameterInt);
        ConsoleIoSendString(")");
        ConsoleIoSendString(STR_ENDLINE);
    }
    return result;
}
static eCommandResult_T ConsoleCommandParamExampleHexUint16(const char buffer[])
{
    uint16_t parameterUint16;
    eCommandResult_T result;
    result = ConsoleReceiveParamHexUint16(buffer, 1, &parameterUint16);
    if ( COMMAND_SUCCESS == result )
    {
        ConsoleIoSendString("Parameter is 0x");
        ConsoleSendParamHexUint16(parameterUint16);
        ConsoleIoSendString(STR_ENDLINE);
    }
    return result;
}

static eCommandResult_T ConsoleCommandVer(const char buffer[])
{
    eCommandResult_T result = COMMAND_SUCCESS;

    IGNORE_UNUSED_VARIABLE(buffer);

    ConsoleIoSendString(VERSION_STRING);
    ConsoleIoSendString(STR_ENDLINE);
    return result;
}

static eCommandResult_T ConsoleCommandFilter(const char buffer[]){
    //char parameterString[15];

    char str[16];

    const sFilterCommandTable_T* filterTable;

    uint32_t cmdIndex;

    int32_t  paramfound=NOT_FOUND;
    eCommandResult_T result;


    filterTable = FilterCommandsGetTable();
    cmdIndex = 0u;

    result = ConsoleReceiveParamString(buffer, 1, str);

     while ( ( NULL != filterTable[cmdIndex].name ) && ( NOT_FOUND == paramfound ) )
     {
         if ( FilterParamMatch(filterTable[cmdIndex].name, str) )
         {
             result = filterTable[cmdIndex].execute(str);
             if ( COMMAND_SUCCESS != result )
             {
                 ConsoleIoSendString("Error: ");
                 ConsoleIoSendString(str);

                 ConsoleIoSendString("Help: ");
                 ConsoleIoSendString(filterTable[cmdIndex].help);
                 ConsoleIoSendString(STR_ENDLINE);

             }
             paramfound = cmdIndex;
         }
         else
         {
             cmdIndex++;

         }
     }
     if ( NOT_FOUND == paramfound )
     {
             ConsoleIoSendString("Param not found.");
             ConsoleIoSendString(STR_ENDLINE);
     }

     return result;
 }

static eCommandResult_T ConsoleCommandImuToggleData(const char buffer[])
{
    eCommandResult_T result = COMMAND_SUCCESS;

     IGNORE_UNUSED_VARIABLE(buffer);

     if (DISPLAY_IMU){
         DISPLAY_IMU = 0;
         ConsoleIoSendString("NOT displaying IMU data!");

     }
     else{
         DISPLAY_IMU = 1;
         ConsoleIoSendString("Displaying IMU data!");
     }

     ConsoleIoSendString(STR_ENDLINE);
     return result;
}
static eCommandResult_T ConsoleCommandSemgToggleData(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;

     IGNORE_UNUSED_VARIABLE(buffer);

     if (DISPLAY_SEMG){
         DISPLAY_SEMG = 0;
         ConsoleIoSendString("NOT displaying SEMG data!");

     }
     else{

         DISPLAY_SEMG = 1;
         ConsoleIoSendString("Displaying SEMG data!");
     }

     ConsoleIoSendString(STR_ENDLINE);
     return result;
}
static eCommandResult_T ConsoleCommandWinPlotToggle(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;

     IGNORE_UNUSED_VARIABLE(buffer);

     if (WINPLOTTER){
         WINPLOTTER = 0;
         ConsoleIoSendString("Display CSV format!");

     }
     else{

         WINPLOTTER = 1;
         ConsoleIoSendString("Output WINPLOT format!");
     }

     ConsoleIoSendString(STR_ENDLINE);
     return result;
}



const sConsoleCommandTable_T* ConsoleCommandsGetTable(void)
{
    return (mConsoleCommandTable);
}



