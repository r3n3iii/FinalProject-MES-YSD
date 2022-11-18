/*
 * filters.c
 *
 *  Created on: Nov. 15, 2022
 *      Author: Rene
 */
#include "filters.h"

#include <string.h>
#include <stdbool.h>

#define EMGBUFF         64
#define LOG2EMGBUFF     6   //log2(EMGBUFF)

#define IMUBUFF         16
#define LOG2IMUBUFF     4 //log2(IMUBUFF)

uint_fast16_t  bSemg00[EMGBUFF], bSemg01[EMGBUFF],bSemg02[EMGBUFF], bSemg03[EMGBUFF];
                /*,bSemg04[EMGBUFF],bSemg05[EMGBUFF], \
               bSemg06[EMGBUFF],bSemg07[EMGBUFF];*/
int16_t bImu00[IMUBUFF], bImu01[IMUBUFF],bImu02[IMUBUFF], bImu03[IMUBUFF], bImu04[IMUBUFF], bImu05[IMUBUFF];

fProcessData semg_mav(MsgSemgData *data);
fProcessData imu_mav(MsgMPU6050Data *data);

static eCommandResult_T FilterParamSemgRaw(const char buffer[]);
static eCommandResult_T FilterParamSemgMovingAvg(const char buffer[]);
static eCommandResult_T FilterParamHelp(const char buffer[]);
static eCommandResult_T FilterParamSemgLp_4Hz(const char buffer[]);
static eCommandResult_T FilterParamIMURaw(const char buffer[]);
static eCommandResult_T FilterParamIMUMovingAvg(const char buffer[]);
static eCommandResult_T FilterParamRPY(const char buffer[]);
static eCommandResult_T FilterParamQuaternions(const char buffer[]);


static const sFilterCommandTable_T mFilterParamTable[] =
{
    {"semg_raw", &FilterParamSemgRaw, HELP("Default. Output raw semg data.")},
    {"semg_mav", &FilterParamSemgMovingAvg, HELP("Applying moving average to semg data")},
    {"help", &FilterParamHelp, HELP("Lists the filters available")},
    {"semg_lp_4hz", &FilterParamSemgLp_4Hz, HELP("Apply 4hz lowpass filter to sEmg signals")},
    {"imu_raw", &FilterParamIMURaw, HELP("Default. Output raw IMU data.")},
    {"imu_mav", &FilterParamIMUMovingAvg, HELP("Applying moving average to imu data")},
    {"imu_rpy", &FilterParamRPY, HELP("Output IMU data in Roll pitch yaw")},
    {"imu_quat", &FilterParamQuaternions, HELP("Output IMU data in quaternions")},

    FILTER_COMMAND_TABLE_END // must be LAST
};

static eCommandResult_T FilterParamSemgRaw(const char buffer[])
{
    eCommandResult_T result = COMMAND_SUCCESS;
    filter_semg_ptr=&semg_raw;
    ConsoleIoSendString("SEMG Filter: No filter. RAW output");
    ConsoleIoSendString(STR_ENDLINE);
    return result;
}
static eCommandResult_T FilterParamSemgMovingAvg(const char buffer[]){

    eCommandResult_T result = COMMAND_SUCCESS;
    filter_semg_ptr=&semg_mav;
    ConsoleIoSendString("SEMG Filter: Moving Average 64 samples ");
    ConsoleIoSendString(STR_ENDLINE);
    return result;
}

static eCommandResult_T FilterParamHelp(const char buffer[]){
    uint32_t i;
        uint32_t tableLength;
        eCommandResult_T result = COMMAND_SUCCESS;

        IGNORE_UNUSED_VARIABLE(buffer);

        tableLength = sizeof(mFilterParamTable) / sizeof(mFilterParamTable[0]);
        for ( i = 0u ; i < tableLength - 1u ; i++ )
        {
            ConsoleIoSendString(mFilterParamTable[i].name);
    #if CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
            ConsoleIoSendString(" : ");
            ConsoleIoSendString(mFilterParamTable[i].help);
    #endif // CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
            ConsoleIoSendString(STR_ENDLINE);
        }
        return result;
}
static eCommandResult_T FilterParamSemgLp_4Hz(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;


    return result;
}
static eCommandResult_T FilterParamIMURaw(const char buffer[])
{
    eCommandResult_T result = COMMAND_SUCCESS;
    filter_imu_ptr=&imu_raw;
    ConsoleIoSendString("IMU Filter: No filter. RAW output");
    ConsoleIoSendString(STR_ENDLINE);
    return result;
}

static eCommandResult_T FilterParamIMUMovingAvg(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;
    filter_imu_ptr=&imu_mav;
    ConsoleIoSendString("IMU Filter: Moving Average 64 samples ");
    ConsoleIoSendString(STR_ENDLINE);
    return result;
}
static eCommandResult_T FilterParamRPY(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;


    return result;
}

static eCommandResult_T FilterParamQuaternions(const char buffer[]){
    eCommandResult_T result = COMMAND_SUCCESS;


    return result;
}
const sFilterCommandTable_T* FilterCommandsGetTable(void)
{
    return (mFilterParamTable);
}

uint32_t FilterParamMatch(const char* name, const char *buffer)
{
    uint32_t i = 0u;
    uint32_t result = 0u; // match

    if ( buffer[i] == name[i] )
    {
        result = 1u;
        i++;
    }

    while ( ( 1u == result ) &&
        ( i < CONSOLE_COMMAND_MAX_COMMAND_LENGTH )  &&
        ( buffer[i] != PARAMETER_SEPARATER ) &&
        ( buffer[i] != LF_CHAR ) &&( buffer[i] != CR_CHAR ) &&
        ( buffer[i] != (char) NULL_CHAR )
        )
    {
        if ( buffer[i] != name[i] )
        {
            result = 0u;
        }
        i++;
    }

    return result;
}

fProcessData semg_mav(MsgSemgData *data){
    static uint8_t index = 0;
    static uint8_t flag = 0;
    static uint_fast16_t sum[4] = {0, 0, 0, 0};
    static bool is_full = 0;
    uint8_t i;
    if  (flag == 0){

        memset(bSemg00, 0x00, EMGBUFF * sizeof(uint_fast16_t));
        memset(bSemg01, 0x00, EMGBUFF * sizeof(uint_fast16_t));
        memset(bSemg02, 0x00, EMGBUFF * sizeof(uint_fast16_t));
        memset(bSemg03, 0x00, EMGBUFF * sizeof(uint_fast16_t));
        flag = 1;
    }

    if ( !is_full)
    {
        bSemg00[index]=  data->emgRaw[0];
        bSemg01[index]=  data->emgRaw[1];
        bSemg02[index]=  data->emgRaw[2];
        bSemg03[index]=  data->emgRaw[3];

        sum[0] +=  bSemg00[index];
        sum[1] +=  bSemg01[index];
        sum[2] +=  bSemg02[index];
        sum[3] +=  bSemg03[index];

        index++;
        if (index == EMGBUFF)
        {
            is_full = 1;
        }
        return WAITING;
     }

     if ( index == EMGBUFF )
     {
          index = 0;
     }
     sum[0] -=  bSemg00[index];
     sum[1] -=  bSemg01[index];
     sum[2] -=  bSemg02[index];
     sum[3] -=  bSemg03[index];

     bSemg00[index]=  data->emgRaw[0];
     bSemg01[index]=  data->emgRaw[1];
     bSemg02[index]=  data->emgRaw[2];
     bSemg03[index]=  data->emgRaw[3];

     sum[0] +=  bSemg00[index];
     sum[1] +=  bSemg01[index];
     sum[2] +=  bSemg02[index];
     sum[3] +=  bSemg03[index];


     for (i=0 ; i< 4; i++)
     {
         data->emgRaw[i]= (sum[i]>> LOG2EMGBUFF);

     }
     index ++ ;


    return DONE;

}
fProcessData semg_raw(MsgSemgData *data){

    return DONE;

}
fProcessData imu_raw(MsgMPU6050Data *data){

    return DONE;

}
fProcessData imu_mav(MsgMPU6050Data *data){
    static uint8_t index = 0;
    static uint8_t flag = 0;
    static int16_t sum[6] = {0, 0, 0, 0, 0, 0};
    static bool is_full = 0;

    if  (flag == 0){

        memset(bImu00, 0x00, IMUBUFF * sizeof(int16_t));  memset(bImu01, 0x00, IMUBUFF * sizeof(int16_t));
        memset(bImu02, 0x00, IMUBUFF * sizeof(int16_t));  memset(bImu03, 0x00, IMUBUFF * sizeof(int16_t));
        memset(bImu04, 0x00, IMUBUFF * sizeof(int16_t));  memset(bImu05, 0x00, IMUBUFF * sizeof(int16_t));
        flag = 1;
    }

    if ( !is_full)
    {
        bImu00[index]=  data->accelerometer[0];
        bImu01[index]=  data->accelerometer[1];
        bImu02[index]=  data->accelerometer[2];
        bImu03[index]=  data->gyro[0];
        bImu04[index]=  data->gyro[1];
        bImu05[index]=  data->gyro[2];

        sum[0] +=  bImu00[index];
        sum[1] +=  bImu01[index];
        sum[2] +=  bImu02[index];
        sum[3] +=  bImu03[index];
        sum[4] +=  bImu04[index];
        sum[5] +=  bImu05[index];

        index++;
        if (index == IMUBUFF)
        {
            is_full = 1;
        }
        return WAITING;
     }

     if ( index == EMGBUFF )
     {
          index = 0;
     }
     sum[0] -=  bImu00[index];
     sum[1] -=  bImu01[index];
     sum[2] -=  bImu02[index];
     sum[3] -=  bImu03[index];
     sum[4] -=  bImu04[index];
     sum[5] -=  bImu05[index];

     bImu00[index]=  data->accelerometer[0];
     bImu01[index]=  data->accelerometer[1];
     bImu02[index]=  data->accelerometer[2];
     bImu03[index]=  data->gyro[0];
     bImu04[index]=  data->gyro[1];
     bImu05[index]=  data->gyro[2];

     sum[0] +=  bImu00[index];
     sum[1] +=  bImu01[index];
     sum[2] +=  bImu02[index];
     sum[3] +=  bImu03[index];
     sum[4] +=  bImu04[index];
     sum[5] +=  bImu05[index];


    data->accelerometer[0]  = (sum[0] >> LOG2IMUBUFF);
    data->accelerometer[1]  = (sum[1] >> LOG2IMUBUFF);
    data->accelerometer[2]  = (sum[2] >> LOG2IMUBUFF);
    data->gyro[0]           = (sum[3] >> LOG2IMUBUFF);
    data->gyro[1]           = (sum[4] >> LOG2IMUBUFF);
    data->gyro[2]           = (sum[5] >> LOG2IMUBUFF);

    return DONE;

}
void imu_rpy(MsgMPU6050Data data){

}
