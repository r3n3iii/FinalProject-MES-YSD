/*
 * MP6050.c
 *
 *  Created on: Oct 14, 2022
 *      Author: Rene
 */
#include "mpu6050.h"
#include "ti_drivers_config.h"
//defined function prototypes
#include <ti/drivers/I2C.h>

#define MPU6050_ADDRESS 0x68

I2C_Params mpu6050Params;
I2C_Handle mpu6050Handle;
I2C_Transaction mpu6050Transaction = {0};



void MPU6050_I2C_init(void){
    I2C_init();

    I2C_Params_init(&mpu6050Params);
    mpu6050Params.bitRate = I2C_400kHz;
    mpu6050Params.transferMode = I2C_MODE_BLOCKING;
    mpu6050Handle = I2C_open(CONFIG_I2C_0, &mpu6050Params);
}


void MPU6050_Reset(){
    uint8_t writeData[] = {0x6B, 0x00}; //0x6b is the power management register RM for mpu6050 page 40, and 0x00 is to reset
    mpu6050Transaction.slaveAddress = MPU6050_ADDRESS;
    mpu6050Transaction.writeBuf = writeData;
    mpu6050Transaction.writeCount = 2;
    I2C_transfer(mpu6050Handle, &mpu6050Transaction);
}
void MPU6050_GetAccelData(int16_t accelerometer[3]){

    uint8_t writeData;
    uint8_t readData[6];

    writeData =  0x3B; //accelerometer starting address
    mpu6050Transaction.slaveAddress = MPU6050_ADDRESS;
    mpu6050Transaction.writeBuf = &writeData;
    mpu6050Transaction.writeCount = 1;
    mpu6050Transaction.readBuf = readData;
    mpu6050Transaction.readCount = 6;
    I2C_transfer(mpu6050Handle, &mpu6050Transaction);

    accelerometer[0] = (readData[0] << 8)|(readData[1]); //x-axis data only
    accelerometer[1] = (readData[2] << 8)|(readData[3]); //y-axis data only
    accelerometer[2] = (readData[4] << 8)|(readData[5]); //z-axis data only

}


void MPU6050_GetGyroData(int16_t gyro[3]){

    uint8_t writeData;
    uint8_t readData[6];

    writeData =  0x43;
    mpu6050Transaction.slaveAddress = MPU6050_ADDRESS;
    mpu6050Transaction.writeBuf = &writeData;
    mpu6050Transaction.writeCount = 1;
    mpu6050Transaction.readBuf = readData;
    mpu6050Transaction.readCount = 6;
    I2C_transfer(mpu6050Handle, &mpu6050Transaction);

   gyro[0] = ((readData[0] << 8)| readData[1]); //x-axis data only
   gyro[1] = ((readData[2] << 8)| readData[3]); //y-axis data only
   gyro[2] = ((readData[4] << 8)| readData[5]); //z-axis data only

}

void MPU6050_GetTempData(int16_t *temp){

    uint8_t writeData;
    uint8_t readData[6];

     //read data from temperature of the chip
     writeData =  0x41;
     mpu6050Transaction.slaveAddress = MPU6050_ADDRESS;
     mpu6050Transaction.writeBuf = &writeData;
     mpu6050Transaction.writeCount = 1;
     mpu6050Transaction.readBuf = readData;
     mpu6050Transaction.readCount = 2;

     I2C_transfer(mpu6050Handle, &mpu6050Transaction);

     *temp = ((readData[0] << 8)|readData[1]); //temperature data only

}


