// ----------------------------------------------------------------------------
// myMailbox.h
// ----------------------------------------------------------------------------

#ifndef MYMAILBOX_H_
#define MYMAILBOX_H_
#define __FPU_PRESENT 1
#include <ti/sysbios/knl/Mailbox.h>                                             // For mailbox APIs
//#include <arm_math.h>


#define EMG_CHANNELS 8

//***** Global Variables ******************************************************
typedef struct MsgSemgData{
    uint_fast16_t  emgRaw[EMG_CHANNELS];
} MsgSemgData;

typedef struct MsgMPU6050Data{
    int16_t accelerometer[3];
    int16_t gyro[3];
 //   int16_t *temp;
} MsgMPU6050Data;

typedef struct MsgPrintData{
    char strData[100];
    uint8_t num_char;
} MsgPrintData;

#endif /* MYMAILBOX_H_ */

