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
#include "filters.h"
#include "sdCard.h"
//#define EMGBUFF         64

/*-----------------GLOBAL VARIABLES--------------------*/
int16_t accelerometer[3], gyro[3];

extern fProcessData (*filter_semg_ptr)(MsgSemgData *data);
extern fProcessData (*filter_imu_ptr)(MsgMPU6050Data *data);


uint16_t i_emg=0;

//console Variables
uint8_t mReceiveBuffer[100];
uint8_t recvd_data;
uint8_t count = 0;
uint8_t reception_complete = FALSE;

UART_Handle uart0,uart1;

/*-----------------SD write VARIABLES--------------------*/
int bufsize(char *buf)
{
    int i = 0;
    while(*buf++ != '\0') i++;
    return i;
}
void bufclear(void)
{
    int i;
    for(i=0; i<1024; i++)
    {
        buffer[i] = '\0';
    }

}
const char inputfile[] = "fat:"STR(DRIVE_NUM)":input.txt";
const char outputfile[] = "fat:"STR(DRIVE_NUM)":output.txt";

const char textarray[] = \
"***********************************************************************\n"
"0         1         2         3         4         5         6         7\n"
"01234567890123456789012345678901234567890123456789012345678901234567890\n"
"This is some text to be inserted into the inputfile if there isn't\n"
"already an existing file located on the media.\n"
"If an inputfile already exists, or if the file was already once\n"
"generated, then the inputfile will NOT be modified.\n"
"***********************************************************************\n";



/* Set this to the current UNIX time in seconds */
/*const struct timespec ts = {
    .tv_sec = 1469647026,
    .tv_nsec = 0
};*/

/* File name prefix for this filesystem for use with TI C RTS */
char fatfsPrefix[] = "fat";

unsigned char cpy_buff[CPY_BUFF_SIZE];

/*------------------- SEMAPHORES ----------------------*/
extern  Semaphore_Handle    SEM_cli,  SEM_semg, SEM_printData, SEM_mpu6050,
                            SEM_printRAW_sEMG, SEM_printRAW_mpu6050, SEM_sdCard_Save;

extern Mailbox_Handle       mbHandle_semg, mbHandle_mpu6050, mbHandle_printData;
extern Task_Handle          HandleTaskSDCard;

/*-----------=---- FUNCTION PROTOTYPES ----------------*/
static float convertToFloat(uint16_t result);



/*
 *  ======== fnTaskCLI ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
void fnTaskCLI(UArg arg0, UArg arg1)
{

    ConsoleInit();

    while (1)
    {
        Semaphore_pend(SEM_cli, BIOS_WAIT_FOREVER);
        if (reception_complete != TRUE){
            UART_read(uart0, &recvd_data, 1);
        }
        else{
           reception_complete = FALSE;
           ConsoleProcess();
        }

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

           semg.emgRaw[0] = MAP_ADC14_getResult(ADC_MEM0)-8191;            //Converting EMG signal to float an mV
           semg.emgRaw[1] = MAP_ADC14_getResult(ADC_MEM1)-8191;
           semg.emgRaw[2] = MAP_ADC14_getResult(ADC_MEM2)-8191;
           semg.emgRaw[3] = MAP_ADC14_getResult(ADC_MEM3)-8191;

           Mailbox_post(mbHandle_semg, &semg, BIOS_WAIT_FOREVER);//Print raw data over serial
       }
}

void fnTaskSemg_process()
{
    MsgSemgData semg_rcv;
    MsgPrintData semgFormat_msg;

    filter_semg_ptr=&semg_raw;

    UART_write(uart1, "DAQ streaming channel for sEMG and MPU6050 data\r\n", sizeof("DAQ streaming channel for sEMG and MPU6050 data\r\n"));

    while(1)
    {

        Mailbox_pend(mbHandle_semg, &semg_rcv, BIOS_WAIT_FOREVER);


        if (filter_semg_ptr(&semg_rcv) == DONE){

            semgFormat_msg.num_char = sprintf(semgFormat_msg.strData, "EMG: %.5f, %.5f, %.5f, %.5f\r\n", convertToFloat(semg_rcv.emgRaw[0]), \
                                              convertToFloat(semg_rcv.emgRaw[1]),convertToFloat(semg_rcv.emgRaw[2]),convertToFloat(semg_rcv.emgRaw[3]));

            Mailbox_post(mbHandle_printData, &semgFormat_msg, BIOS_WAIT_FOREVER);
        }
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
    filter_imu_ptr=&imu_raw;

    while(1)
    {
        Mailbox_pend(mbHandle_mpu6050, &mpu6050_rcv, BIOS_WAIT_FOREVER);
        if (filter_imu_ptr(&mpu6050_rcv) == DONE){

            mpu6050Format_msg.num_char = sprintf(mpu6050Format_msg.strData, "IMU: %d, %d, %d, %d, %d, %d\r\n",  \
                                             mpu6050_rcv.accelerometer[0],mpu6050_rcv.accelerometer[1], mpu6050_rcv.accelerometer[2],\
                                             mpu6050_rcv.gyro[0],   mpu6050_rcv.gyro[1],mpu6050_rcv.gyro[2]);

            Mailbox_post(mbHandle_printData, &mpu6050Format_msg, BIOS_WAIT_FOREVER);
        }
    }
}

void fnTaskPrintData(UArg arg0, UArg arg1){
    MsgPrintData dataFormat_rcvd;
    while(1){
        Mailbox_pend(mbHandle_printData, &dataFormat_rcvd, BIOS_WAIT_FOREVER);
        UART_write(uart1, dataFormat_rcvd.strData, dataFormat_rcvd.num_char);
    }
}

/*
 *  ======== fnTaskSDCardSave ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
void fnSDCardSave(UArg arg0, UArg arg1){

    SDFatFS_Handle sdfatfsHandle;

    /* Variables for the CIO functions */
    FILE *src, *dst;

    char strData[100];

    /* Variables to keep track of the file copy progress */
    unsigned int bytesRead = 0;
    unsigned int bytesWritten = 0;
    unsigned int filesize;
    unsigned int totalBytesCopied = 0;

    /* Return variables */
    int result;
    /* Call driver init functions */
    SDFatFS_init();

    /* add_device() should be called once and is used for all media types */
    add_device(fatfsPrefix, _MSA, ffcio_open, ffcio_close, ffcio_read,
        ffcio_write, ffcio_lseek, ffcio_unlink, ffcio_rename);

    /* Initialize real-time clock */
   // clock_settime(CLOCK_REALTIME, &ts);
   // Semaphore_pend(SEM_sdCard_Save, BIOS_WAIT_FOREVER);
    /* Mount and register the SD Card */
    sdfatfsHandle = SDFatFS_open(CONFIG_SD_0, DRIVE_NUM);
    if (sdfatfsHandle == NULL) {
     //   UART_write(uart0, "Error starting the SD card\n", sizeof("Error starting the SD card\n"));
        //while (1);
        Task_block(HandleTaskSDCard);

    }
    else {

       // UART_write(uart0, "Drive %u is mounted\n", DRIVE_NUM);
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
           // Semaphore_post(SEM_cli);
        }else
        {

            mReceiveBuffer[count++] = recvd_data;
        }

}

//*****************************************************************************
// gpioButtonFxn0
//
// Callback function for the GPIO interrupt on Board_GPIO_BUTTON0
//
// Notes:
//   - CONFIG_GPIO_BUTTON_0 is connected to Switch 1 (SW1) on the LaunchPad
//   - The "index" argument is not used by this function, but it references
//     which GPIO input was triggered as defined by the GPIOName provided
//     in the project's board specific header file
//*****************************************************************************
void gpioButtonFxn0(uint_least8_t index)
{
    GPIO_toggle(CONFIG_GPIO_LED_0); // Toggle LED

    Semaphore_post(SEM_sdCard_Save);
}

