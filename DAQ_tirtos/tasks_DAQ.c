/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>


/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
// #include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

#include <math.h>
#include <string.h>

#include <stdint.h>


/* Board Header file */

#include "console.h"
#include "myAdc.h"
#include "myGpio.h"
#include "myUart.h"
#include "mpu6050.h"
#include "ti_drivers_config.h"
#include "printf.h"
#include "myMailbox.h"

#define EMGBUFF         20

/*-----------------GLOBAL VARIABLES--------------------*/
int16_t accelerometer[3], gyro[3];

float  adcEMG0[EMGBUFF], adcEMG1[EMGBUFF];
uint16_t i_emg=0;

//console Variables
uint8_t mReceiveBuffer[100];
uint8_t recvd_data;
uint8_t count = 0;
uint8_t reception_complete = FALSE;

UART_Handle uart0,uart1;

/*------------------- SEMAPHORES ----------------------*/
extern  Semaphore_Handle    SEM_cli,  SEM_semg, SEM_printData, SEM_mpu6050,
                            SEM_printRAW_sEMG, SEM_printRAW_mpu6050;

extern Mailbox_Handle      mbHandle_semg, mbHandle_mpu6050, mbHandle_printData;

/*-----------=---- FUNCTION PROTOTYPES ----------------*/
static float convertToFloat(uint16_t result);

/*
 *  ======== fnTaskCLI ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
void fnTaskCLI(UArg arg0, UArg arg1)
{

    ConsoleInit();

    while (1) {
            Semaphore_pend(SEM_cli, BIOS_WAIT_FOREVER); // when a character is received via UART, the interrupt handler will release the binary semaphore

            while (reception_complete != TRUE){
                       UART_read(uart0, &recvd_data, 1);
                   }
               reception_complete = FALSE;
               ConsoleProcess();

            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0); // LED B - visual clue that we've received a request over USB
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0); // LED B - visual clue off
        }
}

 /*  ======== fnTaskSEMG ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
void fnTaskSEMG_getData(UArg arg0, UArg arg1)
{
    MsgSemgData semg;
    float  adcEmg_pr0[EMGBUFF], adcEmg_pr1[EMGBUFF];
    memset(adcEmg_pr0, 0x00, EMGBUFF * sizeof(float));
    memset(adcEmg_pr1, 0x00, EMGBUFF * sizeof(float));



       while (1)
       {
           Semaphore_pend(SEM_semg, BIOS_WAIT_FOREVER);                              // Wait for semaphore from HWI callback function

           uint64_t status = 0;
           while((status & ADC_INT1)==0)                           //Spin here until ADC finish conversion for EMG channels
           {
               MAP_ADC14_toggleConversionTrigger();
               status = MAP_ADC14_getEnabledInterruptStatus();
               MAP_ADC14_clearInterruptFlag(status);
           }

           //ADC_MEM0 = EmgSensor on P5.4, ADC_MEM1 = EmgSensor on P5.0

           /*adcEmg_pr0[i_emg] = convertToFloat(MAP_ADC14_getResult(ADC_MEM0)-8191);            //Converting EMG signal to float an mV
           adcEmg_pr1[i_emg] = convertToFloat(MAP_ADC14_getResult(ADC_MEM1)-8191);*/

           semg.emgPlot[0] = convertToFloat(MAP_ADC14_getResult(ADC_MEM0)-8191);            //Converting EMG signal to float an mV
           semg.emgPlot[1] = convertToFloat(MAP_ADC14_getResult(ADC_MEM1)-8191);
           semg.emgPlot[2] = convertToFloat(MAP_ADC14_getResult(ADC_MEM2)-8191);
           semg.emgPlot[3] = convertToFloat(MAP_ADC14_getResult(ADC_MEM3)-8191);
          /* semg.emgPlot[4] = convertToFloat(MAP_ADC14_getResult(ADC_MEM4)-8191);
           semg.emgPlot[5] = convertToFloat(MAP_ADC14_getResult(ADC_MEM5)-8191);
           semg.emgPlot[6] = convertToFloat(MAP_ADC14_getResult(ADC_MEM6)-8191);
           semg.emgPlot[7] = convertToFloat(MAP_ADC14_getResult(ADC_MEM7)-8191);*/

           Mailbox_post(mbHandle_semg, &semg, BIOS_WAIT_FOREVER);//Print raw data over serial

          /*
           i_emg++;
           if (i_emg==EMGBUFF)
           {
              memcpy(&adcEMG0, &adcEmg_pr0, sizeof(adcEmg_pr0));                   //coping buffer emg to a global variable buffer
              memcpy(&adcEMG1, &adcEmg_pr1, sizeof(adcEmg_pr1));                   //to use in RMS task function
              i_emg=0;
              Semaphore_post(SEM_printData);                                           //Making EmgRms task ready to run
           }*/
       }
}

void fnTaskSemg_process()
{

    MsgSemgData semg_rcv;
    MsgPrintData semgFormat_msg;


   // UART_write(uart1, "DAQ streaming channel for sEMG and MPU6050 data\r\n", sizeof("DAQ streaming channel for sEMG and MPU6050 data\r\n"));

        while(1)
    {

     Mailbox_pend(mbHandle_semg, &semg_rcv, BIOS_WAIT_FOREVER);
     //num_char = sprintf(str, "RAW EMG: %.5f, %.5f,\r\n",semg_rcv.emgPlot[0],semg_rcv.emgPlot[1]);
     /*num_char = sprintf(str, "RAW EMG: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\r\n", semg_rcv.emgPlot[0], \
                                                                     semg_rcv.emgPlot[1],semg_rcv.emgPlot[2],semg_rcv.emgPlot[3], \
                                                                     semg_rcv.emgPlot[4],semg_rcv.emgPlot[5],semg_rcv.emgPlot[6], \
                                                                     semg_rcv.emgPlot[7]);*/
    semgFormat_msg.num_char = sprintf(semgFormat_msg.strData, "EMG: %.5f, %.5f, %.5f, %.5f\r\n", semg_rcv.emgPlot[0], \
                                                                          semg_rcv.emgPlot[1],semg_rcv.emgPlot[2],semg_rcv.emgPlot[3]/*, \
                                                                          semg_rcv.emgPlot[4],semg_rcv.emgPlot[5],semg_rcv.emgPlot[6], \
                                                                          semg_rcv.emgPlot[7]*/);
     Mailbox_post(mbHandle_printData, &semgFormat_msg, BIOS_WAIT_FOREVER);
     //UART_write(uart1, str, num_char);
    }
}
/*  ======== fnTaskMpu6050 ========
*  Task for this function is created statically. See the project's .cfg file.
*/
void fnTaskMpu6050_getData(UArg arg0, UArg arg1)
{
    MsgMPU6050Data mpu6050_tx;

    while(1)
    {
       Semaphore_pend(SEM_mpu6050, BIOS_WAIT_FOREVER);

      // MPU6050_GetAccelData(mpu6050_tx.accelerometer);
       MPU6050_GetAccelData(accelerometer);
       mpu6050_tx.accelerometer[0]= accelerometer[0];
       mpu6050_tx.accelerometer[1]= accelerometer[1];
       mpu6050_tx.accelerometer[2]= accelerometer[2];

      // MPU6050_GetGyroData(mpu6050_tx.gyro);
       MPU6050_GetGyroData(gyro);
       mpu6050_tx.gyro[0] = gyro[0];
       mpu6050_tx.gyro[1] = gyro[1];
       mpu6050_tx.gyro[2] = gyro[2];
     //  mpu6050_tx.temp = temp;

       Mailbox_post(mbHandle_mpu6050, &mpu6050_tx, BIOS_WAIT_FOREVER);
    }
}

void fnTaskMpu6050_process(UArg arg0, UArg arg1)
{
    MsgMPU6050Data mpu6050_rcv;
    MsgPrintData mpu6050Format_msg;
    while(1)
    {
        Mailbox_pend(mbHandle_mpu6050, &mpu6050_rcv, BIOS_WAIT_FOREVER);

        mpu6050Format_msg.num_char = sprintf(mpu6050Format_msg.strData, "IMU: %d, %d, %d, %d, %d, %d\r\n",  \
                                             mpu6050_rcv.accelerometer[0],mpu6050_rcv.accelerometer[1], mpu6050_rcv.accelerometer[2],\
                                             mpu6050_rcv.gyro[0],   mpu6050_rcv.gyro[1],mpu6050_rcv.gyro[2]);

       //UART_write(uart1, strBuff, num_chars);
       Mailbox_post(mbHandle_printData, &mpu6050Format_msg, BIOS_WAIT_FOREVER);
    }
}

void fnTaskPrintData(UArg arg0, UArg arg1){
    MsgPrintData dataFormat_rcvd;
    while(1){
        Mailbox_pend(mbHandle_printData, &dataFormat_rcvd, BIOS_WAIT_FOREVER);
        UART_write(uart1, dataFormat_rcvd.strData, dataFormat_rcvd.num_char);
    }
}

//*****************************************************************************
// Convert to float Function
//*****************************************************************************
static float convertToFloat(uint16_t result)
{
    int32_t temp;

    if(0x8000 & result)
    {
        temp = (result >> 2) | 0xFFFFC000;
        return ((temp * 2.5f) / 8191);
    }
    else
        return ((result >> 2)*2.5f) / 8191;
}


//*****************************************************************************
// UART0 CALLBACK FUNCTION
//*****************************************************************************
void UART00_IRQHandler(UART_Handle handle, void *buffer, size_t num)
{
    if(recvd_data == '\n'){
            reception_complete = TRUE;
            mReceiveBuffer[count++] = '\n';
        }else
        {

            mReceiveBuffer[count++] = recvd_data;
        }
}

