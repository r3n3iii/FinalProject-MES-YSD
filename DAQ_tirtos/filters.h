/*
 * filters.h
 *
 *  Created on: Nov. 15, 2022
 *      Author: Rene
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include <arm_math.h>
#include "console.h"
#include "consoleCommands.h"
#include "consoleIo.h"
#include "myMailbox.h"

#define FILTER_COMMAND_TABLE_END {NULL, NULL, HELP("")}

/*
#define N 101

float32_t h[N] = {
    3.5981E-03,3.8670E-03,4.1411E-03,4.4201E-03,4.7035E-03,4.9910E-03,
    5.2822E-03,5.5767E-03,5.8739E-03,6.1736E-03,6.4751E-03,6.7780E-03,
    7.0819E-03,7.3862E-03,7.6905E-03,7.9942E-03,8.2969E-03,8.5980E-03,
    8.8970E-03,9.1934E-03,9.4868E-03,9.7765E-03,1.0062E-02,1.0343E-02,
    1.0619E-02,1.0889E-02,1.1153E-02,1.1410E-02,1.1660E-02,1.1903E-02,
    1.2138E-02,1.2364E-02,1.2582E-02,1.2790E-02,1.2988E-02,1.3177E-02,
    1.3355E-02,1.3522E-02,1.3678E-02,1.3823E-02,1.3956E-02,1.4078E-02,
    1.4187E-02,1.4284E-02,1.4368E-02,1.4440E-02,1.4498E-02,1.4544E-02,
    1.4577E-02,1.4597E-02,1.4603E-02,1.4597E-02,1.4577E-02,1.4544E-02,
    1.4498E-02,1.4440E-02,1.4368E-02,1.4284E-02,1.4187E-02,1.4078E-02,
    1.3956E-02,1.3823E-02,1.3678E-02,1.3522E-02,1.3355E-02,1.3177E-02,
    1.2988E-02,1.2790E-02,1.2582E-02,1.2364E-02,1.2138E-02,1.1903E-02,
    1.1660E-02,1.1410E-02,1.1153E-02,1.0889E-02,1.0619E-02,1.0343E-02,
    1.0062E-02,9.7765E-03,9.4868E-03,9.1934E-03,8.8970E-03,8.5980E-03,
    8.2969E-03,7.9942E-03,7.6905E-03,7.3862E-03,7.0819E-03,6.7780E-03,
    6.4751E-03,6.1736E-03,5.8739E-03,5.5767E-03,5.2822E-03,4.9910E-03,
    4.7035E-03,4.4201E-03,4.1411E-03,3.8670E-03,3.5981E-03
};*/

typedef enum {
    DONE = 0,
    WAITING = 1,
    ERROR   = -1

}fProcessData;
typedef eCommandResult_T(*FilterCommand_T)(const char buffer[]);

typedef struct sFilterCommandStruct
{
    const char* name;
    FilterCommand_T execute;
#if CONSOLE_COMMAND_MAX_HELP_LENGTH > 0
    char help[CONSOLE_COMMAND_MAX_HELP_LENGTH];
#else
    uint8_t junk;
#endif // CONSOLE_COMMAND_MAX_HELP_LENGTH
} sFilterCommandTable_T;

fProcessData semg_raw(MsgSemgData *data);
fProcessData imu_raw(MsgMPU6050Data *data);

const sFilterCommandTable_T* FilterCommandsGetTable(void);
uint32_t FilterParamMatch(const char* name, const char *buffer);

fProcessData (*filter_semg_ptr)(MsgSemgData *data);
fProcessData (*filter_imu_ptr)(MsgMPU6050Data *data);


#endif /* FILTERS_H_ */
